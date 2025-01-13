#include <QApplication>
#include <QThread>
#include "kitchen_node.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>

int main(int argc, char *argv[])
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);

    // QApplication 생성
    QApplication app(argc, argv);

    // SignalBridge와 노드 생성
    SignalBridge signal_bridge;
    auto service_node = std::make_shared<KitchenServiceNode>(&signal_bridge);
    KitchenDisplay display_node(&signal_bridge, service_node);

    // 멀티스레드 executor 생성
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(service_node);

    // ROS 2 스핀을 별도의 스레드에서 실행
    std::thread ros_thread([&executor]() {
        executor.spin();
        rclcpp::shutdown();
    });

    // 윈도우 표시
    display_node.show();

    int result = app.exec();

    // ROS 스레드 종료 대기
    if (ros_thread.joinable())
        ros_thread.join();

    return result;
}
