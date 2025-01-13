#ifndef AMR_NODE_H
#define AMR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <pizza_order_msgs/srv/goal_loc.hpp>
#include <QWidget>

#include <vector>
#include <string>

struct TableCoordinate {
    int table_number;
    double x;
    double y;
};

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

    // 테이블 좌표 데이터
    std::vector<TableCoordinate> table_coordinates_;

    // 팝업 메시지 박스
    QMessageBox* popup_msg_;
};

#endif  // AMR_NODE_H
