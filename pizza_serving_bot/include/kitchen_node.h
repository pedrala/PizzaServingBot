#ifndef KITCHEN_NODE_H
#define KITCHEN_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pizza_order_msgs/srv/order_service.hpp>
#include <pizza_order_msgs/srv/cancel_service.hpp>
#include <pizza_order_msgs/msg/call_manager.hpp>
#include <pizza_order_msgs/srv/goal_loc.hpp>
#include <QObject>
#include <QWidget>
#include <QThread>
#include <QTimer>
#include <QLabel>
#include <QPushButton>
#include <QGridLayout>
#include <QScrollArea>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QList>
#include <QPair>

// REST API 관련 라이브러리
#include <cpprest/http_client.h>
#include <cpprest/json.h>

class SignalBridge : public QObject
{
    Q_OBJECT
public:
    SignalBridge() {}
signals:
    void table_call_signal(const QString &message);
};

class KitchenServiceNode : public rclcpp::Node
{
public:
    KitchenServiceNode(SignalBridge *signal_bridge);

    // 주문취소 서비스 클라이언트
    rclcpp::Client<pizza_order_msgs::srv::CancelService>::SharedPtr cancel_client_;

    // 주문정보와 Goal(테이블 위치) 클라이언트
    rclcpp::Client<pizza_order_msgs::srv::GoalLoc>::SharedPtr goal_client_;

private:
    void process_order_callback(
        const std::shared_ptr<pizza_order_msgs::srv::OrderService::Request> request,
        std::shared_ptr<pizza_order_msgs::srv::OrderService::Response> response);

    void call_manager_callback(
        const pizza_order_msgs::msg::CallManager::SharedPtr msg);

    // 서비스 및 토픽
    rclcpp::Service<pizza_order_msgs::srv::OrderService>::SharedPtr order_service_;
    rclcpp::Subscription<pizza_order_msgs::msg::CallManager>::SharedPtr call_manager_subscriber_;

    SignalBridge *signal_bridge_;
};

class KitchenDisplay : public QWidget
{
    Q_OBJECT
public:
    explicit KitchenDisplay(SignalBridge *signal_bridge, std::shared_ptr<KitchenServiceNode> service_node, QWidget *parent = nullptr);

private slots:
    void show_popup_message(const QString &message);
    void refresh_orders();
    void call_amr(int order_id, int table_number, const QString &status);
    void cancel_order(int order_id, int table_number, const QString &status);

private:
    void load_orders();
    void clear_order_cards();
    void add_order_card(int order_id, const QString &order_time, int table_number, const QList<QString> &menu_items);
    void update_order_grid();
    QGroupBox* create_order_card(int order_id, const QString &order_time, int table_number, const QList<QString> &menu_items);

    QLabel *robot_label_;
    QLabel *title_label_;
    QLabel *call_label_;

    QGridLayout *grid_layout_;
    QWidget *order_widget_;
    QScrollArea *scroll_area_;

    QTimer *timer_;

    QList<QGroupBox*> order_cards_;

    std::shared_ptr<KitchenServiceNode> service_node_;

    // REST API 주소
    QString api_base_url_;

    int rows_;
    int cols_;
};

#endif  // KITCHEN_NODE_H
