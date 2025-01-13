#include "kitchen_node.h"
#include <QDebug>

using namespace web;
using namespace web::http;
using namespace web::http::client;

// REST API 엔드포인트를 api_server.py에 맞게 수정합니다.
KitchenDisplay::KitchenDisplay(SignalBridge *signal_bridge, std::shared_ptr<KitchenServiceNode> service_node, QWidget *parent)
    : QWidget(parent), service_node_(service_node)
{
    this->setWindowTitle("Kitchen Display System");
    this->resize(1660, 1080);

    rows_ = 2;
    cols_ = 4;
    api_base_url_ = "http://192.168.0.18:5000/api";  // api_server.py에서 설정한 포트와 호스트를 사용합니다.

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

void KitchenDisplay::load_orders()
{
    // REST API를 사용하여 주문 정보 로드
    // cpprestsdk를 사용하여 HTTP GET 요청
    http_client client(U(api_base_url_.toStdString()));
    uri_builder builder(U("/orders"));  // 엔드포인트 수정

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
            auto orders_object = json_object.as_object();
            for (auto const& order_pair : orders_object) {
                int order_id = std::stoi(order_pair.first);
                auto data = order_pair.second.as_object();

                QString timestamp = QString::fromStdString(data["timestamp"].as_string());
                int table_number = data["table_number"].as_integer();
                QList<QString> menu_items;

                for (auto const& item : data["menu_items"].as_array()) {
                    menu_items.append(QString::fromStdString(item.as_string()));
                }

                add_order_card(order_id, timestamp, table_number, menu_items);
            }
        }
        update_order_grid();
    })
    .wait();
}

// 주문 상태를 업데이트하는 함수 수정
void KitchenDisplay::update_order_status(int order_id, const QString& status)
{
    try {
        http_client client(U(api_base_url_.toStdString()));

        // 상태에 따라 URL 수정 ('Cancelled' 또는 'Completed')
        uri_builder builder(U("/order/" + std::to_string(order_id) + "/" + status.toStdString()));

        client.request(methods::PUT, builder.to_string())
        .then([order_id, status](http_response response) {
            if (response.status_code() == status_codes::OK) {
                qDebug() << QString("Order %1 status updated to %2").arg(order_id).arg(status);
            } else {
                qDebug() << QString("Failed to update order status, status code: %1").arg(response.status_code());
            }
        })
        .wait();

        // 주문 새로고침
        refresh_orders();

    } catch (const std::exception& e) {
        qDebug() << "Error updating order status:" << e.what();
    }
}

// 주문 취소 함수 수정
void KitchenDisplay::cancel_order(int order_id, int table_number, const QString& status)
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

            // 주문 상태 업데이트
            update_order_status(order_id, "Cancelled");
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
}

// 주문 완료 함수 수정
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

        // 주문 상태 업데이트
        update_order_status(order_id, "Completed");
    }
    else
    {
        qDebug() << "Navigation failed";
    }
}

// 매출 정보를 로드하는 함수 추가 (SalesWindow 클래스 내부)
void SalesWindow::create_graph(const QString &sales_type)
{
    qDebug() << "sales_type:" << sales_type;

    try {
        http_client client(U(api_base_url_.toStdString()));

        // sales_type 값을 소문자로 변환
        QString sales_type_lower = sales_type.toLower();

        // 매출액 데이터 요청
        uri_builder builder(U("/sales/" + sales_type_lower.toStdString()));
        auto sales_response = client.request(methods::GET, builder.to_string()).get();

        // 메뉴별 판매량 데이터 요청
        uri_builder menu_builder(U("/menu_sales/" + sales_type_lower.toStdString()));
        auto menu_sales_response = client.request(methods::GET, menu_builder.to_string()).get();

        if (sales_response.status_code() != status_codes::OK) {
            qDebug() << "[ERROR] Failed to fetch sales data:" << sales_response.status_code();
            return;
        }
        if (menu_sales_response.status_code() != status_codes::OK) {
            qDebug() << "[ERROR] Failed to fetch menu sales data:" << menu_sales_response.status_code();
            return;
        }

        // 응답 데이터 파싱
        auto sales_json = sales_response.extract_json().get();
        auto menu_sales_json = menu_sales_response.extract_json().get();

        // 매출액 데이터
        std::vector<QString> labels;
        std::vector<double> sales_values;

        auto labels_array = sales_json.at(U("labels")).as_array();
        auto sales_array = sales_json.at(U("sales")).as_array();

        for (auto& label : labels_array) {
            labels.push_back(QString::fromStdString(label.as_string()));
        }
        for (auto& sales : sales_array) {
            sales_values.push_back(sales.as_double());
        }

        // 메뉴별 판매량 데이터
        std::vector<QString> menu_names;
        std::vector<double> menu_sales_values;

        auto menu_names_array = menu_sales_json.at(U("menu_names")).as_array();
        auto menu_sales_array = menu_sales_json.at(U("menu_sales")).as_array();

        for (auto& name : menu_names_array) {
            menu_names.push_back(QString::fromStdString(name.as_string()));
        }
        for (auto& menu_sale : menu_sales_array) {
            menu_sales_values.push_back(menu_sale.as_double());
        }

        qDebug() << "[DEBUG] Parsed Sales Data:" << labels << sales_values;
        qDebug() << "[DEBUG] Parsed Menu Sales Data:" << menu_names << menu_sales_values;

        // 그래프 그리기
        this->figure.clear();

        // 상단: 매출액 그래프
        auto ax1 = this->figure.add_subplot(211);
        ax1->bar(labels.begin(), labels.end(), sales_values.begin(), sales_values.end(), "green");
        ax1->set_title("매출액", "fontsize=18", "fontweight='bold'");
        ax1->set_ylabel("금액(원)", "fontsize=18", "fontweight='bold'", "rotation=0", "labelpad=45");
        ax1->grid(true);

        // 하단: 메뉴별 판매량 그래프
        auto ax2 = this->figure.add_subplot(212);
        ax2->bar(menu_names.begin(), menu_names.end(), menu_sales_values.begin(), menu_sales_values.end(), "orange");
        ax2->set_title("판매량", "fontsize=18", "fontweight='bold'");
        ax2->set_ylabel("수량(개)", "fontsize=18", "fontweight='bold'", "rotation=0", "labelpad=70");
        ax2->set_xticklabels(menu_names);
        ax2->grid(true);

        // 그래프 업데이트
        this->canvas.draw();

    } catch (const std::exception& e) {
        qDebug() << "Error:" << e.what();
        return;
    }
}
