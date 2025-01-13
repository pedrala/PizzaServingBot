#include "kitchen_service_node.h"
#include <QDebug>

KitchenServiceNode::KitchenServiceNode()
    : Node("kitchen_service_node")
{
    // QoS 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.best_effort();

    // 주문 서비스 서버 설정
    order_service_ = this->create_service<pizza_order_msgs::srv::OrderService>(
        "process_order",
        std::bind(&KitchenServiceNode::process_order_callback, this, std::placeholders::_1, std::placeholders::_2),
        qos_profile);

    // 주문 취소 서비스 클라이언트 설정
    cancel_client_ = this->create_client<pizza_order_msgs::srv::CancelService>(
        "cancel_order", qos_profile);

    // 직원 호출 구독 설정
    call_manager_subscriber_ = this->create_subscription<pizza_order_msgs::msg::CallManager>(
        "call_manager",
        qos_profile,
        std::bind(&KitchenServiceNode::call_manager_callback, this, std::placeholders::_1));

    // 주문 정보와 목표 위치 클라이언트 설정
    goal_loc_client_ = this->create_client<pizza_order_msgs::srv::GoalLoc>(
        "goal_loc", qos_profile);
}

void KitchenServiceNode::process_order_callback(
    const std::shared_ptr<pizza_order_msgs::srv::OrderService::Request> request,
    std::shared_ptr<pizza_order_msgs::srv::OrderService::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Processing order: %s", request->order_id.c_str());
    // 주문 처리 로직 추가
    response->status = "Completed";
}

void KitchenServiceNode::call_manager_callback(
    const pizza_order_msgs::msg::CallManager::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Table number %d requested assistance.", msg->table_number);
    QString popup_message = QString("테이블번호 %1로부터 직원호출이 요청되었습니다.").arg(msg->table_number);
    emit signal_bridge_.table_call_signal(popup_message);
}
