#ifndef KIOSK_NODE_H
#define KIOSK_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <pizza_order_msgs/srv/order_service.hpp>
#include <pizza_order_msgs/msg/order_detail.hpp>
#include <pizza_order_msgs/srv/cancel_service.hpp>
#include <pizza_order_msgs/msg/call_manager.hpp>
#include <QObject>
#include <QWidget>
#include <QMainWindow>
#include <QLabel>
#include <QListWidget>
#include <QComboBox>
#include <QTimer>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QScrollArea>
#include <QGridLayout>
#include <QPixmap>
#include <QSpacerItem>
#include <QSizePolicy>

class SignalBridge : public QObject
{
    Q_OBJECT
public:
    SignalBridge() {}
signals:
    void table_call_signal(const QString &message);
};

class KioskNode : public QMainWindow
{
    Q_OBJECT
public:
    explicit KioskNode(SignalBridge *signal_bridge, QWidget *parent = nullptr);
    ~KioskNode();

private slots:
    void show_popup_message(const QString &message);
    void update_time();
    void change_category(const QString &category);
    void load_menu_items();
    void update_cart(const QString &item_name, int quantity_change);
    void update_cart_display();
    void place_order();
    void order_response_callback(const rclcpp::Client<pizza_order_msgs::srv::OrderService>::SharedFuture future);
    void show_total_orders();
    void call_staff();

private:
    void init_data();
    void initUI();
    void fetch_menu_items(const QString &category);
    int get_item_price(const QString &item_name);

    // ROS2 관련 멤버 변수
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<pizza_order_msgs::srv::OrderService>::SharedPtr order_client_;
    rclcpp::Service<pizza_order_msgs::srv::CancelService>::SharedPtr cancel_service_;
    rclcpp::Publisher<pizza_order_msgs::msg::CallManager>::SharedPtr call_manager_publisher_;

    SignalBridge *signal_bridge_;

    // UI 관련 멤버 변수
    QLabel *time_label_;
    QComboBox *table_dropdown_;
    QGridLayout *grid_layout_;
    QListWidget *cart_list_widget_;
    QLabel *total_label_;

    QString current_category_;
    QMap<QString, QList<QMap<QString, QVariant>>> menu_data_;
    QMap<QString, int> cart_;
    QList<QMap<QString, QVariant>> order_history_;

    QTimer *timer_;

    // 기타 멤버 변수
    QString db_path_;
};

#endif  // KIOSK_NODE_H
