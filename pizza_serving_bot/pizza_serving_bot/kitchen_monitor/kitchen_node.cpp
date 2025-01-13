#include "kitchen_node.h"
#include <QDebug>

using namespace web;
using namespace web::http;
using namespace web::http::client;

KitchenServiceNode::KitchenServiceNode(SignalBridge *signal_bridge)
    : Node("kitchen_service_node"), signal_bridge_(signal_bridge)
{
    // QoS 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliable();

    // 주문 서비스 서버 설정
    order_service_ = this->create_service<pizza_order_msgs::srv::OrderService>(
        "process_order",
        std::bind(&KitchenServiceNode::process_order_callback, this, std::placeholders::_1, std::placeholders::_2),
        qos_profile);

    // 주문취소 서비스 클라이언트 설정
    cancel_client_ = this->create_client<pizza_order_msgs::srv::CancelService>(
        "cancel_order", qos_profile);

    // 직원 호출 구독 설정
    call_manager_subscriber_ = this->create_subscription<pizza_order_msgs::msg::CallManager>(
        "call_manager",
        qos_profile,
        std::bind(&KitchenServiceNode::call_manager_callback, this, std::placeholders::_1));

    // 주문 정보와 Goal(테이블 위치) 클라이언트 설정
    goal_client_ = this->create_client<pizza_order_msgs::srv::GoalLoc>(
        "goal_loc", qos_profile);
}

void KitchenServiceNode::process_order_callback(
    const std::shared_ptr<pizza_order_msgs::srv::OrderService::Request> request,
    std::shared_ptr<pizza_order_msgs::srv::OrderService::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "주문 처리 중: %d", request->order_id);
    // 주문 처리 로직 추가
    response->status = "Completed";
}

void KitchenServiceNode::call_manager_callback(
    const pizza_order_msgs::msg::CallManager::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "테이블번호 %d로부터 직원호출이 요청되었습니다.", msg->table_number);
    QString popup_message = QString("테이블번호 %1로부터 직원호출이 요청되었습니다.").arg(msg->table_number);
    emit signal_bridge_->table_call_signal(popup_message);
}

KitchenDisplay::KitchenDisplay(SignalBridge *signal_bridge, std::shared_ptr<KitchenServiceNode> service_node, QWidget *parent)
    : QWidget(parent), service_node_(service_node)
{
    this->setWindowTitle("Kitchen Display System");
    this->resize(1660, 1080);

    rows_ = 2;
    cols_ = 4;
    api_base_url_ = "http://192.168.0.18:5000/api";

    QVBoxLayout *main_layout = new QVBoxLayout(this);
    QHBoxLayout *top_layout = new QHBoxLayout();

    // 상단 레이아웃: 제목 및 좌표 표시
    robot_label_ = new QLabel("ROBOT COORDINATE : X(  ), Y(  ), Z(  )");
    robot_label_->setStyleSheet("font-size: 16px; font-weight: bold;");
    top_layout->addWidget(robot_label_, 0, Qt::AlignLeft);

    title_label_ = new QLabel("KITCHEN DISPLAY");
    title_label_->setAlignment(Qt::AlignCenter);
    title_label_->setStyleSheet("font-size: 30px; font-weight: bold; color: green;");
    top_layout->addWidget(title_label_);

    // 직원 호출 상태 표시
    call_label_ = new QLabel("No calls yet");
    call_label_->setStyleSheet("font-size: 18px; font-weight: bold; color: red;");
    top_layout->addWidget(call_label_, 0, Qt::AlignRight);

    main_layout->addLayout(top_layout);

    scroll_area_ = new QScrollArea();
    scroll_area_->setWidgetResizable(true);

    order_widget_ = new QWidget();
    order_widget_->setStyleSheet("background-color: #F0F0F0;");
    grid_layout_ = new QGridLayout();
    grid_layout_->setSpacing(20);
    order_widget_->setLayout(grid_layout_);

    scroll_area_->setWidget(order_widget_);
    main_layout->addWidget(scroll_area_);

    // Signal-Slot 연결
    connect(signal_bridge, &SignalBridge::table_call_signal,
            this, &KitchenDisplay::show_popup_message);

    // 주문 새로고침 타이머
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &KitchenDisplay::refresh_orders);
    timer_->start(1000);  // 1초마다 새로고침

    load_orders();
}

void KitchenDisplay::show_popup_message(const QString &message)
{
    call_label_->setText(message);
    QMessageBox::information(this, "Call Manager", message);
}

void KitchenDisplay::refresh_orders()
{
    load_orders();
}

void KitchenDisplay::load_orders()
{
    // REST API를 사용하여 주문 정보 로드
    // cpprestsdk를 사용하여 HTTP GET 요청
    http_client client(U(api_base_url_.toStdString()));
    uri_builder builder(U("/orders"));

    client.request(methods::GET, builder.to_string())
        .then([this](http_response response) {
            if (response.status_code() == status_codes::OK) {
                return response.extract_json();
            }
            return pplx::task_from_result(json::value());
        })
        .then([this](json::value json_object) {
            clear_order_cards();
            if (!json_object.is_null()) {
                auto orders = json_object.as_object();
                for (auto const& order_pair : orders) {
                    int order_id = std::stoi(order_pair.first);
                    auto data = order_pair.second.as_object();
                    QString order_time = QString::fromStdString(data["timestamp"].as_string());
                    int table_number = data["table_number"].as_integer();
                    QList<QString> menu_items;
                    for (auto const& item : data["menu_items"].as_array()) {
                        menu_items.append(QString::fromStdString(item.as_string()));
                    }
                    add_order_card(order_id, order_time, table_number, menu_items);
                }
            }
            update_order_grid();
        })
        .wait();
}

void KitchenDisplay::clear_order_cards()
{
    for (auto card : order_cards_) {
        card->setParent(nullptr);
    }
    order_cards_.clear();
}

void KitchenDisplay::add_order_card(int order_id, const QString &order_time, int table_number, const QList<QString> &menu_items)
{
    QGroupBox* card = create_order_card(order_id, order_time, table_number, menu_items);
    order_cards_.append(card);
}

void KitchenDisplay::update_order_grid()
{
    // 기존 위젯 제거
    QLayoutItem* item;
    while ((item = grid_layout_->takeAt(0)) != nullptr) {
        delete item->widget();
        delete item;
    }

    // 주문 카드 추가
    for (int index = 0; index < order_cards_.size(); ++index) {
        int row = index / cols_;
        int col = index % cols_;
        grid_layout_->addWidget(order_cards_.at(index), row, col);
    }

    // 총 슬롯 수 계산
    int total_slots = rows_ * cols_;

    // 부족한 슬롯에 빈 공간 추가
    for (int i = order_cards_.size(); i < total_slots; ++i) {
        int row = i / cols_;
        int col = i % cols_;
        QLabel* placeholder = new QLabel("");
        placeholder->setStyleSheet("background-color: #E0E0E0; border: 1px dashed gray;");
        grid_layout_->addWidget(placeholder, row, col);
    }
}

QGroupBox* KitchenDisplay::create_order_card(int order_id, const QString &order_time, int table_number, const QList<QString> &menu_items)
{
    QGroupBox* card = new QGroupBox();
    QVBoxLayout* card_layout = new QVBoxLayout();
    card_layout->setAlignment(Qt::AlignTop);

    QLabel* order_id_label = new QLabel(QString("Order #%1").arg(order_id));
    order_id_label->setFixedHeight(50);
    order_id_label->setAlignment(Qt::AlignCenter);
    order_id_label->setStyleSheet("font-size: 28px; font-weight: bold; color: green;");
    card_layout->addWidget(order_id_label);

    QHBoxLayout* order_info_layout = new QHBoxLayout();
    QLabel* order_time_label = new QLabel(QString("TIME : %1").arg(order_time));
    order_time_label->setFixedHeight(50);
    order_time_label->setAlignment(Qt::AlignCenter);
    QLabel* table_number_label = new QLabel(QString("TABLE : %1").arg(table_number));
    table_number_label->setFixedHeight(50);
    table_number_label->setAlignment(Qt::AlignCenter);
    order_time_label->setStyleSheet("font-size: 20px;");
    table_number_label->setStyleSheet("font-size: 20px;");
    order_info_layout->addWidget(order_time_label);
    order_info_layout->addWidget(table_number_label);
    card_layout->addLayout(order_info_layout);

    for (const auto& item : menu_items) {
        QLabel* menu_label = new QLabel(QString("  • %1").arg(item));
        menu_label->setStyleSheet("font-size: 20px;");
        menu_label->setFixedHeight(50);
        card_layout->addWidget(menu_label);
    }

    QPushButton* cancel_button = new QPushButton("Cancel Order");
    cancel_button->setStyleSheet(R"(
        background-color: red;
        color: white;
        font-size: 20px;
        padding: 5px 10px;
        border-radius: 5px;
    )");
    connect(cancel_button, &QPushButton::clicked, [=]() {
        cancel_order(order_id, table_number, "Cancelled");
    });
    card_layout->addWidget(cancel_button);

    QPushButton* complete_button = new QPushButton("Complete Order");
    complete_button->setStyleSheet(R"(
        background-color: blue;
        color: white;
        font-size: 20px;
        padding: 5px 10px;
        border-radius: 5px;
    )");
    connect(complete_button, &QPushButton::clicked, [=]() {
        call_amr(order_id, table_number, "Completed");
    });
    card_layout->addWidget(complete_button);

    card->setLayout(card_layout);
    card->setStyleSheet(R"(
        QGroupBox {
            background-color: white;
            border: 2px solid green;
            border-radius: 10px;
            padding: 15px;
        }
    )");

    return card;
}

void KitchenDisplay::call_amr(int order_id, int table_number, const QString &status)
{
    // GoalLoc 서비스 호출
    auto request = std::make_shared<pizza_order_msgs::srv::GoalLoc::Request>();
    request->order_id = order_id;
    request->table_number = table_number;
    request->status = status.toStdString();

    auto result_future = service_node_->goal_client_->async_send_request(request);

    // 결과 처리
    if (rclcpp::spin_until_future_complete(service_node_, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        qDebug() << "Navigation Result:" << result->send_result
                 << "x:" << result->x << "y:" << result->y;
    }
    else
    {
        qDebug() << "Navigation failed";
    }

    // 주문 상태를 'Completed'로 업데이트하기 위해 API 호출
    QString api_url = QString("%1/order/%2/Completed").arg(api_base_url_).arg(order_id);

    http_client client(U(api_url.toStdString()));
    client.request(methods::PUT)
        .then([](http_response response) {
            if (response.status_code() == status_codes::OK) {
                qDebug() << "Order status updated to 'Completed'";
            } else {
                qDebug() << "Failed to update order status";
            }
        })
        .wait();
}

void KitchenDisplay::cancel_order(int order_id, int table_number, const QString &status)
{
    // CancelService 서비스 호출
    auto request = std::make_shared<pizza_order_msgs::srv::CancelService::Request>();
    request->order_id = order_id;
    request->table_number = table_number;

    auto result_future = service_node_->cancel_client_->async_send_request(request);

    // 결과 처리
    if (rclcpp::spin_until_future_complete(service_node_, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result_future.get();
        if (response->status == "Cancelled")
        {
            QMessageBox::information(this, "주문 취소", QString("주문 번호: %1번\n주문이 취소되었습니다.\n테이블 번호: %2").arg(order_id).arg(table_number));
        }
        else
        {
            QMessageBox::warning(this, "주문 취소 실패", "주문 취소 요청이 실패했습니다.");
        }
    }
    else
    {
        QMessageBox::critical(this, "오류 발생", "주문 취소 처리 중 오류가 발생했습니다.");
    }

    // 주문 상태를 'Cancelled'로 업데이트하기 위해 API 호출
    QString api_url = QString("%1/order/%2/Cancelled").arg(api_base_url_).arg(order_id);

    http_client client(U(api_url.toStdString()));
    client.request(methods::PUT)
        .then([](http_response response) {
            if (response.status_code() == status_codes::OK) {
                qDebug() << "Order status updated to 'Cancelled'";
            } else {
                qDebug() << "Failed to update order status";
            }
        })
        .wait();
}
