#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <QTimer>
#include <QPushButton>
#include <QSpacerItem>
#include <QSizePolicy>
#include <QMessageBox>
#include <QSvgRenderer>
#include <QPixmap>
#include <QPainter>
#include <rclcpp/rclcpp.hpp>

#include "amr_node.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    // ROS2 노드 생성
    auto node = std::make_shared<AMRNode>();

    // Qt 윈도우 생성
    QWidget window;
    window.setWindowTitle("Service Robot Status Display");
    window.setFixedSize(300, 400);

    // Labels for displaying order information
    QLabel status_label("Status: 대기중", &window);
    QLabel order_label("Order Id: ", &window);
    QLabel table_label("Table Number: ", &window);

    // Create the start delivery button and set it to be initially disabled
    QPushButton start_button("출발", &window);
    start_button.setObjectName("start_button");
    start_button.setEnabled(false);

    // Connect the start button
    QObject::connect(&start_button, &QPushButton::clicked, [&]() {
        if (node->table_number_ != -1)
        {
            node->status_ = "Navigating";
            node->get_logger()->info("Starting delivery to Table %d", node->table_number_);
            node->send_goal(node->table_number_);
        }
        else
        {
            node->get_logger()->error("No table number set. Cannot start delivery.");
        }
    });

    // QLabel to hold the emoji
    QLabel emoji_label(&window);
    emoji_label.setAlignment(Qt::AlignCenter);
    emoji_label.setFixedSize(200, 220);

    // Layout Setup
    QVBoxLayout layout;
    layout.addWidget(&status_label);
    layout.addWidget(&order_label);
    layout.addWidget(&table_label);
    layout.addWidget(&emoji_label);

    // Spacer to push the start button to the bottom
    QSpacerItem spacer(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
    layout.addItem(&spacer);

    // Adding the start button at the bottom and centering it
    layout.addWidget(&start_button, 0, Qt::AlignCenter);

    window.setLayout(&layout);

    // Timer to refresh UI
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        status_label.setText(QString("Status: ") + QString::fromStdString(node->status_));
        order_label.setText(QString("Order Id: ") + QString::number(node->order_id_));
        table_label.setText(QString("Table Number: ") + QString::number(node->table_number_));

        // Change status and emoji based on the robot's status
        QString svg_path;
        if (node->status_ == "Waiting")
        {
            // Set sleeping.svg
            svg_path = ":/img/sleeping.svg";  // 리소스 파일로부터 로드
        }
        else if (node->status_ == "Completed" || node->status_ == "Arrived")
        {
            // Set smiling.svg
            svg_path = ":/img/smiling.svg";  // 리소스 파일로부터 로드
        }
        else if (node->status_ == "Navigating" || node->status_ == "Going Home")
        {
            // Set running.svg
            svg_path = ":/img/running.svg";  // 리소스 파일로부터 로드
        }
        else
        {
            emoji_label.clear();
        }

        if (!svg_path.isEmpty())
        {
            QSvgRenderer renderer(svg_path);
            QPixmap pixmap(200, 220);
            pixmap.fill(Qt::transparent);
            QPainter painter(&pixmap);
            renderer.render(&painter);
            emoji_label.setPixmap(pixmap);
        }

        emoji_label.update();
    });
    timer.start(500);

    window.show();

    // 노드에 윈도우 포인터 설정
    node->set_order_display_app(&window);

    // QTimer를 사용하여 ROS2 이벤트 주기적으로 실행
    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [node]() {
        rclcpp::spin_some(node);
    });
    ros_timer.start(10);

    int result = app.exec();

    // 종료 처리
    node->destroy_node();
    rclcpp::shutdown();

    return result;
}
