#ifndef KIOSK_NODE_H
#define KIOSK_NODE_H

#include <QMainWindow>
#include <QObject>
#include <QLabel>
#include <QComboBox>
#include <QListWidget>
#include <QMap>
#include <QTimer>
#include <QTime>
#include <QSignalMapper>
#include <QGridLayout>
#include <QPushButton>
#include <QFrame>
#include <QScrollArea>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QPixmap>

#include <rclcpp/rclcpp.hpp>
#include <pizza_order_msgs/srv/order_service.hpp>
#include <pizza_order_msgs/msg/order_detail.hpp>
#include <pizza_order_msgs/srv/cancel_service.hpp>
#include <pizza_order_msgs/msg/call_manager.hpp>
#include <std_msgs/msg/string.hpp>

class KioskNode : public QMainWindow, public rclcpp::Node
{
    Q_OBJECT
public:
    KioskNode(QWidget *parent = nullptr);
    ~KioskNode();

private slots:
    void updateTime();
    void changeCategory(const QString &category);
    void updateCart(QString item_name, int quantity_change);
    void updateCartDisplay();
    void placeOrder();
    void callStaff();
    void showTotalOrders();
    void orderResponseCallback(rclcpp::Client<pizza_order_msgs::srv::OrderService>::SharedFuture future);

private:
    void initData();
    void initUI();
    void loadMenuItems();
    int getItemPrice(const QString &item_name);

    // ROS2 통신
    rclcpp::Client<pizza_order_msgs::srv::OrderService>::SharedPtr order_client_;
    rclcpp::Service<pizza_order_msgs::srv::CancelService>::SharedPtr cancel_service_;
    rclcpp::Publisher<pizza_order_msgs::msg::CallManager>::SharedPtr call_manager_publisher_;

    // UI 요소
    QLabel *time_label_;
    QComboBox *table_dropdown_;
    QLabel *total_label_;
    QListWidget *cart_list_widget_;
    QGridLayout *grid_layout_;
    QWidget *menu_grid_;

    // 데이터 관리
    QString db_path_;
    QMap<QString, QList<QMap<QString, QVariant>>> menu_data_;
    QString current_category_;
    QMap<QString, int> cart_;
    QList<QMap<QString, QVariant>> order_history_;
};

#endif // KIOSK_NODE_H
