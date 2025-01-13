#ifndef KITCHEN_NODE_H
#define KITCHEN_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pizza_order_msgs/srv/order_service.hpp>
#include <pizza_order_msgs/srv/cancel_service.hpp>
#include <pizza_order_msgs/msg/call_manager.hpp>
#include <pizza_order_msgs/srv/goal_loc.hpp>
#include <QObject>
#include <QTimer>

class KitchenServiceNode : public rclcpp::Node
{
public:
    KitchenServiceNode(QObject *parent = nullptr);

    // Signal을 위한 QObject 상속
    class SignalBridge : public QObject
    {
        Q_OBJECT
    public:
        SignalBridge() {}
    signals:
        void table_call_signal(const QString &message);
    };

    SignalBridge* getSignalBridge() { return &signal_bridge_; }

    // 주문취소 서비스 클라이언트
    rclcpp::Client<pizza_order_msgs::srv::CancelService>::SharedPtr cancel_client_;

    // 주문정보와 Goal(테이블 위치) 클라이언트
    rclcpp::Client<pizza_order_msgs::srv::GoalLoc>::SharedPtr goal_client_;

private:
    // 콜백 함수
    void process_order_callback(
        const std::shared_ptr<pizza_order_msgs::srv::OrderService::Request> request,
        std::shared_ptr<pizza_order_msgs::srv::OrderService::Response> response);

    void call_manager_callback(
        const pizza_order_msgs::msg::CallManager::SharedPtr msg);

    // 서비스 및 토픽
    rclcpp::Service<pizza_order_msgs::srv::OrderService>::SharedPtr order_service_;
    rclcpp::Subscription<pizza_order_msgs::msg::CallManager>::SharedPtr call_manager_subscriber_;

    SignalBridge signal_bridge_;
};

#endif  // KITCHEN_NODE_H
