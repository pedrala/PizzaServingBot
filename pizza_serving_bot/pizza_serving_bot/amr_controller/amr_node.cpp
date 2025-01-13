#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

using namespace std::placeholders;

class AmrNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    AmrNode() : Node("amr_node") {
        // Initialize action client
        this->action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // GUI Setup
        setupGui();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    void setupGui() {
        QApplication app(argc, argv);

        QMainWindow window;
        QWidget *centralWidget = new QWidget();
        QVBoxLayout *layout = new QVBoxLayout();

        QLabel *label = new QLabel("AMR Controller GUI");
        QPushButton *button = new QPushButton("Send Goal");
        layout->addWidget(label);
        layout->addWidget(button);

        centralWidget->setLayout(layout);
        window.setCentralWidget(centralWidget);

        window.show();
        app.exec();
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped &goal_pose) {
        if (!this->action_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&AmrNode::goalResponseCallback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&AmrNode::feedbackCallback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&AmrNode::resultCallback, this, _1);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalResponseCallback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
    }

    void resultCallback(const GoalHandleNavigateToPose::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AmrNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
