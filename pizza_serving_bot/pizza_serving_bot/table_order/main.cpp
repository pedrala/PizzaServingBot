#include <QApplication>
#include "kiosk_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    SignalBridge signal_bridge;
    KioskNode window(&signal_bridge);
    window.show();

    int result = app.exec();

    rclcpp::shutdown();

    return result;
}
