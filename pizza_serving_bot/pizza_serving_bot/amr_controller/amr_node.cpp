#include "amr_node.h"
#include <QMessageBox>
#include <QPushButton>
#include <QLabel>
#include <QSvgRenderer>
#include <QPixmap>
#include <QPainter>
#include <QVBoxLayout>
#include <QSpacerItem>
#include <QSizePolicy>

// 필요한 경우 네임스페이스 사용
using std::placeholders::_1;
using std::placeholders::_2;

AMRNode::AMRNode()
    : Node("amr_node"),
      table_number_(-1),
      order_id_(-1),
      status_("Waiting"),
      order_display_app_(nullptr),
      popup_msg_(nullptr)
{
    // 테이블 좌표 설정
    table_coordinates_ = {
        {0, 0.0, -0.51},
        {1, 2.5, 2.0},   // 오케이
        {2, -1.0379695892333984, -0.5310251712799072},
        {3, 2.3, -1.5},  // 오케이
        {4, 0.0048611038364470005, -0.5726232528686523},
        {5, 0.5, 2.2},   // 오케이
        {6, -0.8, -2.0}
    };

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
    auto it = std::find_if(table_coordinates_.begin(), table_coordinates_.end(),
                           [table_number](const TableCoordinate& coord) {
                               return coord.table_number == table_number;
                           });
    if (it != table_coordinates_.end())
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
        auto it = std::find_if(table_coordinates_.begin(), table_coordinates_.end(),
                               [request](const TableCoordinate& coord) {
                                   return coord.table_number == request->table_number;
                               });

        if (it == table_coordinates_.end())
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

        // UI 갱신
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
    auto it = std::find_if(table_coordinates_.begin(), table_coordinates_.end(),
                           [table_number](const TableCoordinate& coord) {
                               return coord.table_number == table_number;
                           });
    if (it == table_coordinates_.end())
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
            popup_msg_ = new QMessageBox();
            popup_msg_->setIcon(QMessageBox::Information);
            popup_msg_->setWindowTitle("배달 완료");

            if (table_number == 0)
            {
                popup_msg_->setText("원위치입니다.");
                popup_msg_->setWindowFlags(popup_msg_->windowFlags() | Qt::WindowStaysOnTopHint);
                popup_msg_->exec();
            }
            else
            {
                popup_msg_->setText(QString::number(table_number) + "번 테이블 주문하신 피자 도착하였습니다.");
                popup_msg_->setWindowFlags(popup_msg_->windowFlags() | Qt::WindowStaysOnTopHint);
                popup_msg_->exec();

                // 확인 버튼 클릭 시 원위치로 이동
                connect(popup_msg_, &QMessageBox::accepted, [this]() {
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
