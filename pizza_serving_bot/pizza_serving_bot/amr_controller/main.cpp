#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <QTimer>
#include <QPushButton>
#include <QSpacerItem>
#include <QSizePolicy>
#include <QMessageBox>
#include <QSvgRenderer>
#include <QPixmap>
#include <QPainter>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/qos.hpp>
#include <pizza_order_msgs/srv/goal_loc.hpp>

#include <vector>
#include <string>
#include <memory>
#include <chrono>

// 필요한 경우 네임스페이스 사용
using std::placeholders::_1;
using std::placeholders::_2;

// 테이블 좌표 구조체 정의
struct TableCoordinate {
    int table_number;
    double x;
    double y;
};

// 테이블 좌표 데이터
std::vector<TableCoordinate> table_coordinates = {
    {0, 0.0, -0.51},
    {1, 2.5, 2.0},   // 오케이
    {2, -1.0379695892333984, -0.5310251712799072},
    {3, 2.3, -1.5},  // 오케이
    {4, 0.0048611038364470005, -0.5726232528686523},
    {5, 0.5, 2.2},   // 오케이
    {6, -0.8, -2.0}
};

// AMRNode 클래스 정의
class AMRNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalLoc = pizza_order_msgs::srv::GoalLoc;

    AMRNode();
    void set_order_display_app(QWidget* app);

    // 상태 정보
    int table_number_;
    int order_id_;
    std::string status_;

    // 주문 정보 표시 앱
    QWidget* order_display_app_;

    // 초기 위치 설정
    void set_initial_pose(double x, double y, double yaw);
    void set_initial_pose_from_table(int table_number);

    // 목표 위치로 이동
    void send_goal(int table_number, bool return_to_initial = false);

private:
    void goal_loc_callback(
        const std::shared_ptr<GoalLoc::Request> request,
        std::shared_ptr<GoalLoc::Response> response);

    void goal_response_callback(
        std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future,
        int table_number, bool return_to_initial);

    void goal_result_callback(
        const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result,
        int table_number, bool return_to_initial);

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 액션 클라이언트
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    // 퍼블리셔 및 구독자
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

    // 서비스 서버
    rclcpp::Service<GoalLoc>::SharedPtr goal_loc_service_;

    // 로봇의 현재 위치
    geometry_msgs::msg::Point current_position_;

    // 초기 위치
    TableCoordinate initial_position_;

    // 팝업 메시지 박스
    QMessageBox* popup_msg_;
};

AMRNode::AMRNode()
    : Node("amr_node"),
      table_number_(-1),
      order_id_(-1),
      status_("Waiting"),
      order_display_app_(nullptr),
      popup_msg_(nullptr)
{
    // 초기 위치 설정
    initial_position_ = {0, 0.0, -0.51};
    RCLCPP_INFO(this->get_logger(), "초기 위치 설정: 테이블 번호 %d", initial_position_.table_number);

    // 액션 클라이언트 생성
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // QoS 설정
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.depth(10);

    // 초기 위치 퍼블리셔
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", qos_profile);

    // 위치 정보 구독
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos_profile, std::bind(&AMRNode::pose_callback, this, _1));

    // 서비스 서버 생성
    goal_loc_service_ = this->create_service<GoalLoc>(
        "goal_loc",
        std::bind(&AMRNode::goal_loc_callback, this, _1, _2));

    // 초기 위치 설정
    set_initial_pose_from_table(0);
}

void AMRNode::set_order_display_app(QWidget* app)
{
    order_display_app_ = app;
}

void AMRNode::set_initial_pose(double x, double y, double yaw)
{
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock()->now();

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = yaw;
    msg.pose.pose.orientation.w = 1.0;

    initial_pose_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Initial pose set to x=%f, y=%f, yaw=%f", x, y, yaw);
}

void AMRNode::set_initial_pose_from_table(int table_number)
{
    auto it = std::find_if(table_coordinates.begin(), table_coordinates.end(),
                           [table_number](const TableCoordinate& coord) {
                               return coord.table_number == table_number;
                           });
    if (it != table_coordinates.end())
    {
        set_initial_pose(it->x, it->y, 0.0);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Table number %d does not exist for initial pose.", table_number);
    }
}

void AMRNode::goal_loc_callback(
    const std::shared_ptr<GoalLoc::Request> request,
    std::shared_ptr<GoalLoc::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "AMR Server is ready.");
    RCLCPP_INFO(this->get_logger(), "goal_loc: table_number=%d, order_id=%d, status=%s",
                request->table_number, request->order_id, request->status.c_str());

    try
    {
        auto it = std::find_if(table_coordinates.begin(), table_coordinates.end(),
                               [request](const TableCoordinate& coord) {
                                   return coord.table_number == request->table_number;
                               });

        if (it == table_coordinates.end())
        {
            RCLCPP_ERROR(this->get_logger(), "테이블 번호 %d에 해당하는 좌표를 찾을 수 없습니다.", request->table_number);
            response->send_result = false;
            return;
        }

        // 좌표 정보 기록
        table_number_ = request->table_number;
        order_id_ = request->order_id;
        status_ = request->status;

        // 로그로 테이블 좌표 출력
        RCLCPP_INFO(this->get_logger(), "목표 테이블 위치: 테이블 %d, X: %f, Y: %f",
                    table_number_, it->x, it->y);

        // 실제 좌표를 response에도 포함
        response->send_result = true;
        response->x = it->x;
        response->y = it->y;

        // UI 갱신 (Qt에서 호출)
        if (order_display_app_)
        {
            order_display_app_->update();  // QWidget의 update 함수 호출
            QPushButton* start_button = order_display_app_->findChild<QPushButton*>("start_button");
            if (start_button)
            {
                start_button->setEnabled(true);
            }
        }

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "목표 위치 처리 중 오류 발생: %s", e.what());
        response->send_result = false;
    }
}

void AMRNode::send_goal(int table_number, bool return_to_initial)
{
    auto it = std::find_if(table_coordinates.begin(), table_coordinates.end(),
                           [table_number](const TableCoordinate& coord) {
                               return coord.table_number == table_number;
                           });
    if (it == table_coordinates.end())
    {
        RCLCPP_ERROR(this->get_logger(), "테이블 번호 %d에 해당하는 좌표가 없습니다.", table_number);
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = it->x;
    goal_msg.pose.pose.position.y = it->y;
    goal_msg.pose.pose.orientation.w = 1.0;

    action_client_->wait_for_action_server();
    auto future_goal_handle = action_client_->async_send_goal(
        goal_msg,
        std::bind(&AMRNode::goal_response_callback, this, _1, table_number, return_to_initial));

    RCLCPP_INFO(this->get_logger(), "목표가 전송되었습니다: 테이블 번호=%d, x=%f, y=%f",
                table_number, it->x, it->y);

    // 버튼 비활성화
    if (order_display_app_)
    {
        QPushButton* start_button = order_display_app_->findChild<QPushButton*>("start_button");
        if (start_button)
        {
            start_button->setEnabled(false);
        }
    }

    if (table_number == 0)
    {
        status_ = "Going Home";
    }
    else
    {
        status_ = "Navigating";
    }

    if (order_display_app_)
    {
        order_display_app_->update();
    }
}

void AMRNode::goal_response_callback(
    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future,
    int table_number, bool return_to_initial)
{
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "테이블 %d에 대한 목표가 거부되었습니다.", table_number);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "테이블 %d에 대한 목표가 수락되었습니다. 이동을 시작합니다...", table_number);

    auto result_future = goal_handle->async_result();
    result_future.wait();

    goal_result_callback(result_future.get(), table_number, return_to_initial);
}

void AMRNode::goal_result_callback(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result,
    int table_number, bool return_to_initial)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "테이블 %d에 성공적으로 도달했습니다.", table_number);
        RCLCPP_INFO(this->get_logger(), "테이블 위치: x=%f, y=%f", current_position_.x, current_position_.y);

        status_ = "Arrived";

        // 팝업 메시지 표시
        if (order_display_app_)
        {
            QMessageBox* popup_msg = new QMessageBox();
            popup_msg->setIcon(QMessageBox::Information);
            popup_msg->setWindowTitle("배달 완료");

            if (table_number == 0)
            {
                popup_msg->setText("원위치입니다.");
                popup_msg->setWindowFlags(popup_msg->windowFlags() | Qt::WindowStaysOnTopHint);
                popup_msg->exec();
            }
            else
            {
                popup_msg->setText(QString::number(table_number) + "번 테이블 주문하신 피자 도착하였습니다.");
                popup_msg->setWindowFlags(popup_msg->windowFlags() | Qt::WindowStaysOnTopHint);
                popup_msg->exec();

                // 확인 버튼 클릭 시 원위치로 이동
                connect(popup_msg, &QMessageBox::accepted, [this]() {
                    RCLCPP_INFO(this->get_logger(), "확인 버튼 클릭: 원위치로 돌아갑니다...");
                    this->send_goal(initial_position_.table_number, true);
                });
            }
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "테이블 %d에 도달하지 못했습니다.", table_number);
    }

    if (return_to_initial)
    {
        RCLCPP_INFO(this->get_logger(), "원위치로 돌아왔습니다. 상태 초기화 중...");
        RCLCPP_INFO(this->get_logger(), "원위치 : x=%f, y=%f", current_position_.x, current_position_.y);

        status_ = "Waiting";
        table_number_ = -1;
        order_id_ = -1;

        if (order_display_app_)
        {
            order_display_app_->update();
        }
    }
}

void AMRNode::pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;
    current_position_.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "AMR 위치 업데이트: x=%f, y=%f", current_position_.x, current_position_.y);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    // ROS2 노드 생성
    auto node = std::make_shared<AMRNode>();

    // PyQt5 윈도우 생성
    QWidget window;
    window.setWindowTitle("Service Robot Status Display");
    window.setFixedSize(300, 400);

    // Labels for displaying order information
    QLabel status_label("Status: 대기중", &window);
    QLabel order_label("Order Id: ", &window);
    QLabel table_label("Table Number: ", &window);

    // Create the start delivery button and set it to be initially disabled
    QPushButton start_button("출발", &window);
    start_button.setObjectName("start_button");
    start_button.setEnabled(false);

    // Connect the start button
    QObject::connect(&start_button, &QPushButton::clicked, [&]() {
        if (node->table_number_ != -1)
        {
            node->status_ = "Navigating";
            node->get_logger()->info("Starting delivery to Table %d", node->table_number_);
            node->send_goal(node->table_number_);
        }
        else
        {
            node->get_logger()->error("No table number set. Cannot start delivery.");
        }
    });

    // QLabel to hold the emoji
    QLabel emoji_label(&window);
    emoji_label.setAlignment(Qt::AlignCenter);
    emoji_label.setFixedSize(200, 220);

    // Layout Setup
    QVBoxLayout layout;
    layout.addWidget(&status_label);
    layout.addWidget(&order_label);
    layout.addWidget(&table_label);
    layout.addWidget(&emoji_label);

    // Spacer to push the start button to the bottom
    QSpacerItem spacer(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
    layout.addItem(&spacer);

    // Adding the start button at the bottom and centering it
    layout.addWidget(&start_button, 0, Qt::AlignCenter);

    window.setLayout(&layout);

    // Timer to refresh UI
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        status_label.setText(QString("Status: ") + QString::fromStdString(node->status_));
        order_label.setText(QString("Order Id: ") + QString::number(node->order_id_));
        table_label.setText(QString("Table Number: ") + QString::number(node->table_number_));

        // Change status and emoji based on the robot's status
        if (node->status_ == "Waiting")
        {
            // Set sleeping.svg
            QString svg_path = ":/img/sleeping.svg";  // 리소스 파일로부터 로드
            QSvgRenderer renderer(svg_path);
            QPixmap pixmap(200, 220);
            pixmap.fill(Qt::transparent);
            QPainter painter(&pixmap);
            renderer.render(&painter);
            emoji_label.setPixmap(pixmap);
        }
        else if (node->status_ == "Completed" || node->status_ == "Arrived")
        {
            // Set smiling.svg
            QString svg_path = ":/img/smiling.svg";  // 리소스 파일로부터 로드
            QSvgRenderer renderer(svg_path);
            QPixmap pixmap(200, 220);
            pixmap.fill(Qt::transparent);
            QPainter painter(&pixmap);
            renderer.render(&painter);
            emoji_label.setPixmap(pixmap);
        }
        else if (node->status_ == "Navigating" || node->status_ == "Going Home")
        {
            // Set running.svg
            QString svg_path = ":/img/running.svg";  // 리소스 파일로부터 로드
            QSvgRenderer renderer(svg_path);
            QPixmap pixmap(200, 220);
            pixmap.fill(Qt::transparent);
            QPainter painter(&pixmap);
            renderer.render(&painter);
            emoji_label.setPixmap(pixmap);
        }
        else
        {
            emoji_label.clear();
        }

        emoji_label.update();
    });
    timer.start(500);

    window.show();

    // 노드에 윈도우 포인터 설정
    node->set_order_display_app(&window);

    // QTimer를 사용하여 ROS2 이벤트 주기적으로 실행
    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [node]() {
        rclcpp::spin_some(node);
    });
    ros_timer.start(10);

    int result = app.exec();

    // 종료 처리
    node->destroy_node();
    rclcpp::shutdown();

    return result;
}
