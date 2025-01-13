#include "kiosk_node.h"
#include <QDebug>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QDateTime>
#include <functional>

KioskNode::KioskNode(SignalBridge *signal_bridge, QWidget *parent)
    : QMainWindow(parent), signal_bridge_(signal_bridge)
{
    // ROS2 노드 초기화
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    node_ = rclcpp::Node::make_shared("kiosk_node");

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // 주문요청 클라이언트 설정
    order_client_ = node_->create_client<pizza_order_msgs::srv::OrderService>(
        "process_order", qos_profile);

    // 주문취소 서버 설정
    cancel_service_ = node_->create_service<pizza_order_msgs::srv::CancelService>(
        "cancel_order",
        [this](const std::shared_ptr<pizza_order_msgs::srv::CancelService::Request> request,
               std::shared_ptr<pizza_order_msgs::srv::CancelService::Response> response) {
            this->node_->get_logger().info("주문 취소 요청: order_id=%d, table_number=%d",
                                           request->order_id, request->table_number);
            response->status = "Cancelled";

            QString popup_message = QString("테이블 번호 %1의 주문이 취소되었습니다.").arg(request->table_number);
            emit signal_bridge_->table_call_signal(popup_message);

            this->node_->get_logger().info(popup_message.toStdString());

            return response;
        });

    // 직원호출 토픽 퍼블리셔 설정
    call_manager_publisher_ = node_->create_publisher<pizza_order_msgs::msg::CallManager>(
        "call_manager", qos_profile);

    db_path_ = "/home/viator/ws/b3p1_ws/pizza.db";

    setWindowTitle("키오스크 주문 시스템");
    setGeometry(100, 100, 1024, 768);

    init_data();
    initUI();

    // Signal-Slot 연결
    connect(signal_bridge_, &SignalBridge::table_call_signal, this, &KioskNode::show_popup_message);
}

KioskNode::~KioskNode()
{
    node_->~Node();
    rclcpp::shutdown();
}

void KioskNode::show_popup_message(const QString &message)
{
    QMessageBox::information(this, "Call Manager", message);
}

void KioskNode::init_data()
{
    menu_data_["피자"] = QList<QMap<QString, QVariant>>();
    menu_data_["음료"] = QList<QMap<QString, QVariant>>();
    menu_data_["기타"] = QList<QMap<QString, QVariant>>();

    fetch_menu_items("피자");
    fetch_menu_items("음료");
    fetch_menu_items("기타");

    current_category_ = "피자";
    cart_.clear();
    order_history_.clear();
}

void KioskNode::fetch_menu_items(const QString &category)
{
    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", "menu_connection");
    db.setDatabaseName(db_path_);

    if (!db.open()) {
        qDebug() << "데이터베이스 열기 실패:" << db.lastError().text();
        return;
    }

    QSqlQuery query(db);
    QString sql_query;

    if (category == "피자") {
        sql_query = "SELECT item_name, price, image FROM Menu WHERE item_id BETWEEN 1 AND 6 ORDER BY item_id;";
    } else if (category == "음료") {
        sql_query = "SELECT item_name, price, image FROM Menu WHERE item_id BETWEEN 7 AND 8 ORDER BY item_id;";
    } else if (category == "기타") {
        sql_query = "SELECT item_name, price, image FROM Menu WHERE item_id BETWEEN 9 AND 12 ORDER BY item_id;";
    } else {
        sql_query = "SELECT item_name, price, image FROM Menu ORDER BY item_id;";
    }

    if (!query.exec(sql_query)) {
        qDebug() << "쿼리 실행 실패:" << query.lastError().text();
        db.close();
        return;
    }

    while (query.next()) {
        QMap<QString, QVariant> item;
        item["name"] = query.value(0).toString();
        item["price"] = query.value(1).toInt();
        item["image"] = query.value(2).toString();
        menu_data_[category].append(item);
    }

    db.close();
}

void KioskNode::initUI()
{
    QWidget *main_widget = new QWidget();
    setCentralWidget(main_widget);

    QVBoxLayout *main_layout = new QVBoxLayout();
    QHBoxLayout *top_layout = new QHBoxLayout();

    // 테이블 번호 선택
    QLabel *table_label = new QLabel("테이블 번호:");
    table_label->setStyleSheet("font-size: 16px; font-weight: bold; color: #424242;");
    top_layout->addWidget(table_label);

    table_dropdown_ = new QComboBox();
    for (int i = 1; i <= 6; ++i) {
        table_dropdown_->addItem(QString::number(i));
    }
    table_dropdown_->setStyleSheet(
        "font-size: 16px; background-color: #FFFFFF; border: 1px solid #DDDDDD; border-radius: 10px; padding: 5px;");
    top_layout->addWidget(table_dropdown_);

    top_layout->addStretch();

    // 현재 시간 표시
    time_label_ = new QLabel();
    time_label_->setStyleSheet("font-size: 16px; color: #424242;");
    update_time();
    top_layout->addWidget(time_label_);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &KioskNode::update_time);
    timer_->start(1000);

    main_layout->addLayout(top_layout);

    // 중간 레이아웃
    QHBoxLayout *middle_layout = new QHBoxLayout();

    // 카테고리 버튼
    QVBoxLayout *category_layout = new QVBoxLayout();
    QLabel *category_label = new QLabel("카테고리");
    category_label->setStyleSheet("font-size: 20px; font-weight: bold; color: #424242;");
    category_layout->addWidget(category_label);

    for (const QString &category : menu_data_.keys()) {
        QPushButton *category_button = new QPushButton(category);
        category_button->setStyleSheet(
            "font-size: 16px; background-color: #FFB74D; color: white; border-radius: 10px; margin-bottom: 10px;");
        connect(category_button, &QPushButton::clicked, [this, category]() {
            this->change_category(category);
        });
        category_layout->addWidget(category_button);
    }

    category_layout->addStretch();
    middle_layout->addLayout(category_layout, 1);

    // 메뉴 레이아웃
    QVBoxLayout *menu_layout = new QVBoxLayout();
    QLabel *menu_header = new QLabel("메뉴 목록");
    menu_header->setStyleSheet("font-size: 20px; font-weight: bold; margin-bottom: 10px; color: #424242;");
    menu_layout->addWidget(menu_header);

    QScrollArea *scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);

    QWidget *menu_grid_widget = new QWidget();
    grid_layout_ = new QGridLayout(menu_grid_widget);
    grid_layout_->setSpacing(10);
    scroll_area->setWidget(menu_grid_widget);
    menu_layout->addWidget(scroll_area);
    middle_layout->addLayout(menu_layout, 3);

    // 장바구니 레이아웃
    QVBoxLayout *cart_layout = new QVBoxLayout();
    QLabel *cart_label = new QLabel("장바구니");
    cart_label->setStyleSheet("font-size: 20px; font-weight: bold; color: #424242;");
    cart_layout->addWidget(cart_label);

    cart_list_widget_ = new QListWidget();
    cart_list_widget_->setStyleSheet(
        "background-color: #FFF8E1; border: 1px solid #DDDDDD; border-radius: 10px; padding: 5px;");
    cart_layout->addWidget(cart_list_widget_);

    total_label_ = new QLabel("현재 가격: 0원");
    total_label_->setStyleSheet("font-size: 18px; font-weight: bold; color: #388E3C;");
    cart_layout->addWidget(total_label_);

    QPushButton *order_button = new QPushButton("주문하기");
    order_button->setStyleSheet(
        "font-size: 18px; background-color: #4CAF50; color: white; border-radius: 10px; padding: 10px;");
    connect(order_button, &QPushButton::clicked, this, &KioskNode::place_order);
    cart_layout->addWidget(order_button);

    QPushButton *call_staff_button = new QPushButton("직원 호출");
    call_staff_button->setStyleSheet(
        "font-size: 18px; background-color: #FF9800; color: white; border-radius: 10px; padding: 10px;");
    connect(call_staff_button, &QPushButton::clicked, this, &KioskNode::call_staff);
    cart_layout->addWidget(call_staff_button);

    QPushButton *total_button = new QPushButton("Total");
    total_button->setStyleSheet(
        "font-size: 18px; background-color: #64B5F6; color: white; border-radius: 10px; padding: 10px;");
    connect(total_button, &QPushButton::clicked, this, &KioskNode::show_total_orders);
    cart_layout->addWidget(total_button);

    middle_layout->addLayout(cart_layout, 2);
    main_layout->addLayout(middle_layout);
    main_widget->setLayout(main_layout);

    // 초기 메뉴 로드
    load_menu_items();
}

void KioskNode::update_time()
{
    QString current_time = QDateTime::currentDateTime().toString("hh:mm:ss");
    time_label_->setText(QString("현재 시간: %1").arg(current_time));
}

void KioskNode::change_category(const QString &category)
{
    current_category_ = category;
    load_menu_items();
}

void KioskNode::load_menu_items()
{
    // 기존 위젯 삭제
    QLayoutItem *child;
    while ((child = grid_layout_->takeAt(0)) != nullptr) {
        delete child->widget();
        delete child;
    }

    QList<QMap<QString, QVariant>> items = menu_data_[current_category_];
    for (int index = 0; index < items.size(); ++index) {
        QMap<QString, QVariant> item = items[index];

        QFrame *frame = new QFrame();
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setStyleSheet("background-color: #FFF9C4; border: 1px solid #FFEB3B; border-radius: 15px;");
        frame->setFixedSize(200, 280);

        QVBoxLayout *frame_layout = new QVBoxLayout();

        // 이미지 추가
        QLabel *image_label = new QLabel();
        QString image_path = item["image"].toString();
        if (!image_path.isEmpty()) {
            QPixmap pixmap(image_path);
            pixmap = pixmap.scaled(200, 180, Qt::KeepAspectRatio);
            image_label->setPixmap(pixmap);
        } else {
            image_label->setText("이미지 없음");
        }
        frame_layout->addWidget(image_label);

        // 음식 이름과 가격
        QLabel *label = new QLabel(QString("%1\n%2원").arg(item["name"].toString()).arg(item["price"].toInt()));
        label->setStyleSheet("font-size: 14px; font-weight: bold; color: #424242;");
        label->setAlignment(Qt::AlignCenter);

        // 버튼 레이아웃
        QHBoxLayout *button_layout = new QHBoxLayout();
        QPushButton *minus_button = new QPushButton("-");
        minus_button->setStyleSheet("font-size: 18px; background-color: #F44336; color: white; border-radius: 10px;");
        connect(minus_button, &QPushButton::clicked, [this, item]() {
            this->update_cart(item["name"].toString(), -1);
        });
        button_layout->addWidget(minus_button);

        QPushButton *plus_button = new QPushButton("+");
        plus_button->setStyleSheet("font-size: 18px; background-color: #4CAF50; color: white; border-radius: 10px;");
        connect(plus_button, &QPushButton::clicked, [this, item]() {
            this->update_cart(item["name"].toString(), 1);
        });
        button_layout->addWidget(plus_button);

        frame_layout->addWidget(label);
        frame_layout->addLayout(button_layout);

        frame->setLayout(frame_layout);
        grid_layout_->addWidget(frame, index / 3, index % 3);
    }
}

void KioskNode::update_cart(const QString &item_name, int quantity_change)
{
    if (!cart_.contains(item_name)) {
        cart_[item_name] = 0;
    }
    cart_[item_name] += quantity_change;

    if (cart_[item_name] <= 0) {
        cart_.remove(item_name);
    }

    update_cart_display();
}

void KioskNode::update_cart_display()
{
    cart_list_widget_->clear();
    int total_price = 0;
    QMapIterator<QString, int> i(cart_);
    while (i.hasNext()) {
        i.next();
        int item_total = get_item_price(i.key()) * i.value();
        cart_list_widget_->addItem(QString("%1 x%2 = %3원").arg(i.key()).arg(i.value()).arg(item_total));
        total_price += item_total;
    }
    total_label_->setText(QString("현재 가격: %1원").arg(total_price));
}

int KioskNode::get_item_price(const QString &item_name)
{
    for (const auto &item : menu_data_["피자"]) {
        if (item["name"].toString() == item_name) {
            return item["price"].toInt();
        }
    }
    for (const auto &item : menu_data_["음료"]) {
        if (item["name"].toString() == item_name) {
            return item["price"].toInt();
        }
    }
    for (const auto &item : menu_data_["기타"]) {
        if (item["name"].toString() == item_name) {
            return item["price"].toInt();
        }
    }
    return 0;
}

void KioskNode::place_order()
{
    int table_number = table_dropdown_->currentText().toInt();

    if (table_number <= 0) {
        QMessageBox::warning(this, "경고", "유효한 테이블 번호를 선택하세요.");
        return;
    }

    if (cart_.isEmpty()) {
        QMessageBox::warning(this, "경고", "장바구니에 항목이 없습니다.");
        return;
    }

    // 데이터베이스에 주문 저장
    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", "order_connection");
    db.setDatabaseName(db_path_);

    if (!db.open()) {
        qDebug() << "데이터베이스 열기 실패:" << db.lastError().text();
        return;
    }

    QSqlQuery query(db);

    // 주문 상태를 'Pending'으로 설정
    query.prepare("INSERT INTO Orders (table_number, status) VALUES (?, ?)");
    query.addBindValue(table_number);
    query.addBindValue("Pending");
    if (!query.exec()) {
        qDebug() << "주문 삽입 실패:" << query.lastError().text();
        db.close();
        return;
    }

    // 방금 추가된 주문의 ID를 가져옴
    int order_id = query.lastInsertId().toInt();

    // OrderDetails 테이블에 주문 항목 추가
    QList<QString> order_details_for_show;
    QList<pizza_order_msgs::msg::OrderDetail> order_details;
    int total_price = 0;
    int detail_id = 0;

    QMapIterator<QString, int> i(cart_);
    while (i.hasNext()) {
        i.next();
        QString item_name = i.key();
        int quantity = i.value();
        int item_price = get_item_price(item_name);
        int item_total_price = item_price * quantity;
        total_price += item_total_price;
        order_details_for_show.append(QString("%1 x%2 (%3원)").arg(item_name).arg(quantity).arg(item_total_price));

        pizza_order_msgs::msg::OrderDetail order_detail;
        order_detail.detail_id = ++detail_id;
        order_detail.item_name = item_name.toStdString();
        order_detail.quantity = quantity;
        order_detail.price = item_price;
        order_details.append(order_detail);

        // OrderDetails 테이블에 삽입
        QSqlQuery detail_query(db);
        detail_query.prepare("INSERT INTO OrderDetails (detail_id, order_id, item_name, quantity, total_price) VALUES (?, ?, ?, ?, ?)");
        detail_query.addBindValue(detail_id);
        detail_query.addBindValue(order_id);
        detail_query.addBindValue(item_name);
        detail_query.addBindValue(quantity);
        detail_query.addBindValue(item_total_price);
        if (!detail_query.exec()) {
            qDebug() << "주문 세부사항 삽입 실패:" << detail_query.lastError().text();
        }
    }

    db.close();

    // 주문 처리 후 장바구니 초기화 및 화면 업데이트
    cart_.clear();
    update_cart_display();

    // 전체 주문 내역에 추가
    QMap<QString, QVariant> order_info;
    order_info["table_number"] = table_number;
    order_info["order_details"] = order_details_for_show.join(", ");
    order_info["total"] = total_price;
    order_history_.append(order_info);

    auto request = std::make_shared<pizza_order_msgs::srv::OrderService::Request>();
    request->order_id = order_id;
    request->table_number = table_number;
    request->detail_id = detail_id;
    request->order_details = order_details;

    // ROS2 서비스 호출
    auto future = order_client_->async_send_request(request);
    future.wait();

    // 주문 완료 메시지 표시
    QString order_details_text = order_details_for_show.join("\n");
    QMessageBox::information(this, "주문 완료",
                             QString("주문 번호 : %1번\n주문이 완료되었습니다.\n주문 내용: \n%2\n총 금액: %3원")
                                 .arg(order_id)
                                 .arg(order_details_text)
                                 .arg(total_price));
}

void KioskNode::show_total_orders()
{
    QString selected_table = table_dropdown_->currentText();

    if (order_history_.isEmpty()) {
        QMessageBox::information(this, "주문 내역", "현재까지의 주문 내역이 없습니다.");
        return;
    }

    QList<QMap<QString, QVariant>> orders_for_table;
    for (const auto &order : order_history_) {
        if (order["table_number"].toInt() == selected_table.toInt()) {
            orders_for_table.append(order);
        }
    }

    if (orders_for_table.isEmpty()) {
        QMessageBox::information(this, QString("테이블 %1의 주문 내역").arg(selected_table), "주문 내역이 없습니다.");
        return;
    }

    // 동일한 메뉴 항목을 하나로 묶기
    QMap<QString, int> combined_orders;
    for (const auto &order : orders_for_table) {
        QStringList items = order["order_details"].toString().split(", ");
        for (const QString &item : items) {
            QStringList parts = item.split(" x");
            QString item_name = parts[0];
            int quantity = parts[1].split(" ").first().toInt();

            combined_orders[item_name] += quantity;
        }
    }

    // 계산서 텍스트 생성
    QString bill_details;
    int total_price = 0;
    QMapIterator<QString, int> i(combined_orders);
    while (i.hasNext()) {
        i.next();
        int item_price = get_item_price(i.key());
        int item_total_price = item_price * i.value();
        total_price += item_total_price;
        bill_details += QString("%1 x%2 = %3원\n").arg(i.key()).arg(i.value()).arg(item_total_price);
    }

    QString bill_summary = QString("테이블 %1의 계산서:\n\n%2\n총 금액: %3원").arg(selected_table).arg(bill_details).arg(total_price);
    QMessageBox::information(this, QString("테이블 %1 계산서").arg(selected_table), bill_summary);
}

void KioskNode::call_staff()
{
    QString selected_table = table_dropdown_->currentText();

    auto msg = pizza_order_msgs::msg::CallManager();
    msg.table_number = selected_table.toInt();

    call_manager_publisher_->publish(msg);

    QMessageBox::information(this, "직원 호출", QString("테이블 %1에서 직원을 호출하였습니다.").arg(selected_table));
}
