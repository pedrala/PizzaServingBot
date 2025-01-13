#ifndef KITCHEN_SERVICE_NODE_H
#define KITCHEN_SERVICE_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pizza_order_msgs/srv/order_service.hpp>
#include <pizza_order_msgs/srv/cancel_service.hpp>
#include <pizza_order_msgs/msg/call_manager.hpp>
#include <pizza_order_msgs/srv/goal_loc.hpp>
#include <QObject>

class KitchenServiceNode : public rclcpp::Node
{
public:
    KitchenServiceNode();

    // Signal을 위한 QObject 상속 (Qt와 ROS 통신을 위해)
    class SignalBridge : public QObject
    {
        Q_OBJECT
    public:
        SignalBridge() {}
    signals:
        void table_call_signal(const QString &message);
    };

    SignalBridge* getSignalBridge() { return &signal_bridge_; }

private:
    // 콜백 함수
    void process_order_callback(
        const std::shared_ptr<pizza_order_msgs::srv::OrderService::Request> request,
        std::shared_ptr<pizza_order_msgs::srv::OrderService::Response> response);

    void call_manager_callback(
        const pizza_order_msgs::msg::CallManager::SharedPtr msg);

    // 서비스 및 토픽
    rclcpp::Service<pizza_order_msgs::srv::OrderService>::SharedPtr order_service_;
    rclcpp::Client<pizza_order_msgs::srv::CancelService>::SharedPtr cancel_client_;
    rclcpp::Subscription<pizza_order_msgs::msg::CallManager>::SharedPtr call_manager_subscriber_;
    rclcpp::Client<pizza_order_msgs::srv::GoalLoc>::SharedPtr goal_loc_client_;

    SignalBridge signal_bridge_;
};

#endif // KITCHEN_SERVICE_NODE_H
