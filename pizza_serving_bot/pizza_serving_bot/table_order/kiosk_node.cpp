// kiosk_node.cpp

#include "kiosk_node.h"
#include <QDebug>
#include <thread>
#include <chrono>

KioskNode::KioskNode()
    : QMainWindow(), Node("kiosk_node") 
{
    // 데이터베이스 경로 설정 (실제 경로로 변경 필요)
    db_path_ = "/home/viator/ws/b3p1_ws/pizza.db";

    // SignalBridge 초기화
    signal_bridge_ = new SignalBridge(this);

    // ROS 2 클라이언트 및 서비스 초기화
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    order_client_ = this->create_client<pizza_order_msgs::srv::OrderService>("process_order", qos_profile);
    cancel_service_ = this->create_service<pizza_order_msgs::srv::CancelService>(
        "cancel_order",
        std::bind(&KioskNode::cancel_order_callback, this, std::placeholders::_1, std::placeholders::_2),
        qos_profile);
    call_manager_publisher_ = this->create_publisher<pizza_order_msgs::msg::CallManager>("call_manager", qos_profile);

    // UI 초기화
    this->setWindowTitle("키오스크 주문 시스템");
    this->resize(1024, 768);

    init_data();
    initUI();

    // Signal-Slot 연결
    connect(signal_bridge_, &SignalBridge::table_call_signal, this, &KioskNode::show_popup_message);
}

KioskNode::~KioskNode() {
    delete signal_bridge_;
}

void KioskNode::show_popup_message(const QString &message) {
    QMessageBox::information(this, "알림", message);
}

void KioskNode::init_data() {
    // 카테고리별로 메뉴 항목을 가져옵니다.
    menu_data_["피자"] = fetch_menu_items("피자");
    menu_data_["음료"] = fetch_menu_items("음료");
    menu_data_["기타"] = fetch_menu_items("기타");
    current_category_ = "피자";  // 초기 카테고리는 "피자"로 설정
}

std::vector<std::map<QString, QString>> KioskNode::fetch_menu_items(const QString &category) {
    std::vector<std::map<QString, QString>> menu_items;

    sqlite3 *db;
    if (sqlite3_open(db_path_.toStdString().c_str(), &db) != SQLITE_OK) {
        qDebug() << "Cannot open database:" << sqlite3_errmsg(db);
        return menu_items;
    }

    QString query;
    if (category == "피자") {
        query = "SELECT item_name, price, image FROM Menu WHERE item_id BETWEEN 1 AND 6 ORDER BY item_id;";
    } else if (category == "음료") {
        query = "SELECT item_name, price, image FROM Menu WHERE item_id BETWEEN 7 AND 8 ORDER BY item_id;";
    } else if (category == "기타") {
        query = "SELECT item_name, price, image FROM Menu WHERE item_id BETWEEN 9 AND 12 ORDER BY item_id;";
    } else {
        query = "SELECT item_name, price, image FROM Menu ORDER BY item_id;";
    }

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query.toStdString().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        qDebug() << "Failed to prepare statement:" << sqlite3_errmsg(db);
        sqlite3_close(db);
        return menu_items;
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        std::map<QString, QString> item;
        item["name"] = QString::fromUtf8(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
        item["price"] = QString::fromUtf8(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1)));
        item["image"] = QString::fromUtf8(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
        menu_items.push_back(item);
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);

    return menu_items;
}

void KioskNode::initUI() {
    QWidget *main_widget = new QWidget();
    this->setCentralWidget(main_widget);

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
    table_dropdown_->setStyleSheet(R"(
        font-size: 16px;
        background-color: #FFFFFF;
        border: 1px solid #DDDDDD;
        border-radius: 10px;
        padding: 5px;
    )");
    top_layout->addWidget(table_dropdown_);

    top_layout->addStretch();

    // 현재 시간 표시
    time_label_ = new QLabel();
    time_label_->setStyleSheet("font-size: 16px; color: #424242;");
    update_time();
    top_layout->addWidget(time_label_);

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &KioskNode::update_time);
    timer->start(1000);

    main_layout->addLayout(top_layout);

    // 중간 레이아웃
    QHBoxLayout *middle_layout = new QHBoxLayout();

    // 카테고리 버튼
    QVBoxLayout *category_layout = new QVBoxLayout();
    QLabel *category_label = new QLabel("카테고리");
    category_label->setStyleSheet("font-size: 20px; font-weight: bold; color: #424242;");
    category_layout->addWidget(category_label);

    QStringList categories = {"피자", "음료", "기타"};
    for (const auto &category : categories) {
        QPushButton *category_button = new QPushButton(category);
        category_button->setStyleSheet(R"(
            font-size: 16px;
            background-color: #FFB74D;
            color: white;
            border-radius: 10px;
            margin-bottom: 10px;
        )");
        connect(category_button, &QPushButton::clicked, [this, category]() {
            change_category(category);
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

    menu_grid_ = new QWidget();
    grid_layout_ = new QGridLayout(menu_grid_);
    grid_layout_->setSpacing(10);
    scroll_area->setWidget(menu_grid_);
    menu_layout->addWidget(scroll_area);
    middle_layout->addLayout(menu_layout, 3);

    // 장바구니 레이아웃
    QVBoxLayout *cart_layout = new QVBoxLayout();
    QLabel *cart_label = new QLabel("장바구니");
    cart_label->setStyleSheet("font-size: 20px; font-weight: bold; color: #424242;");
    cart_layout->addWidget(cart_label);

    cart_list_widget_ = new QListWidget();
    cart_list_widget_->setStyleSheet(R"(
        background-color: #FFF8E1;
        border: 1px solid #DDDDDD;
        border-radius: 10px;
        padding: 5px;
    )");
    cart_layout->addWidget(cart_list_widget_);

    total_label_ = new QLabel("현재 가격: 0원");
    total_label_->setStyleSheet("font-size: 18px; font-weight: bold; color: #388E3C;");
    cart_layout->addWidget(total_label_);

    QPushButton *order_button = new QPushButton("주문하기");
    order_button->setStyleSheet(R"(
        font-size: 18px;
        background-color: #4CAF50;
        color: white;
        border-radius: 10px;
        padding: 10px;
    )");
    connect(order_button, &QPushButton::clicked, this, &KioskNode::place_order);
    cart_layout->addWidget(order_button);

    QPushButton *call_staff_button = new QPushButton("직원 호출");
    call_staff_button->setStyleSheet(R"(
        font-size: 18px;
        background-color: #FF9800;
        color: white;
        border-radius: 10px;
        padding: 10px;
    )");
    connect(call_staff_button, &QPushButton::clicked, this, &KioskNode::call_staff);
    cart_layout->addWidget(call_staff_button);

    QPushButton *total_button = new QPushButton("Total");
    total_button->setStyleSheet(R"(
        font-size: 18px;
        background-color: #64B5F6;
        color: white;
        border-radius: 10px;
        padding: 10px;
    )");
    connect(total_button, &QPushButton::clicked, this, &KioskNode::show_total_orders);
    cart_layout->addWidget(total_button);

    middle_layout->addLayout(cart_layout, 2);
    main_layout->addLayout(middle_layout);
    main_widget->setLayout(main_layout);

    // 초기 메뉴 로드
    load_menu_items();
}

void KioskNode::update_time() {
    QString current_time = QTime::currentTime().toString("hh:mm:ss");
    time_label_->setText("현재 시간: " + current_time);
}

void KioskNode::change_category(const QString &category) {
    current_category_ = category;
    load_menu_items();
}

void KioskNode::load_menu_items() {
    // 기존 메뉴 위젯 삭제
    QLayoutItem *child;
    while ((child = grid_layout_->takeAt(0)) != nullptr) {
        delete child->widget();
        delete child;
    }

    auto items = menu_data_[current_category_];
    int index = 0;
    for (const auto &item : items) {
        QFrame *frame = new QFrame();
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setStyleSheet("background-color: #FFF9C4; border: 1px solid #FFEB3B; border-radius: 15px;");
        frame->setFixedSize(200, 280);

        QVBoxLayout *frame_layout = new QVBoxLayout();

        // 이미지 추가
        QLabel *image_label = new QLabel();
        QString image_path = item.at("image");
        if (!image_path.isEmpty()) {
            QPixmap pixmap(image_path);
            pixmap = pixmap.scaled(200, 180, Qt::KeepAspectRatio);
            image_label->setPixmap(pixmap);
        } else {
            image_label->setText("이미지 없음");
        }
        frame_layout->addWidget(image_label);

        // 음식 이름과 가격
        QLabel *label = new QLabel(item.at("name") + "\n" + item.at("price") + "원");
        label->setStyleSheet("font-size: 14px; font-weight: bold; color: #424242;");
        label->setAlignment(Qt::AlignCenter);

        // 버튼 레이아웃
        QHBoxLayout *button_layout = new QHBoxLayout();
        QPushButton *minus_button = new QPushButton("-");
        minus_button->setStyleSheet(R"(
            font-size: 18px;
            background-color: #F44336;
            color: white;
            border-radius: 10px;
        )");
        connect(minus_button, &QPushButton::clicked, [this, item]() {
            update_cart(item.at("name"), -1);
        });
        button_layout->addWidget(minus_button);

        QPushButton *plus_button = new QPushButton("+");
        plus_button->setStyleSheet(R"(
            font-size: 18px;
            background-color: #4CAF50;
            color: white;
            border-radius: 10px;
        )");
        connect(plus_button, &QPushButton::clicked, [this, item]() {
            update_cart(item.at("name"), 1);
        });
        button_layout->addWidget(plus_button);

        frame_layout->addWidget(label);
        frame_layout->addLayout(button_layout);

        frame->setLayout(frame_layout);
        grid_layout_->addWidget(frame, index / 3, index % 3);
        index++;
    }
}

void KioskNode::update_cart(const QString &item_name, int quantity_change) {
    if (cart_.find(item_name) == cart_.end()) {
        cart_[item_name] = 0;
    }
    cart_[item_name] += quantity_change;

    if (cart_[item_name] <= 0) {
        cart_.erase(item_name);
    }

    update_cart_display();
}

void KioskNode::update_cart_display() {
    cart_list_widget_->clear();
    double total_price = 0;
    for (const auto &entry : cart_) {
        QString item_name = entry.first;
        int quantity = entry.second;
        double item_price = get_item_price(item_name);
        cart_list_widget_->addItem(item_name + " x" + QString::number(quantity) + " = " + QString::number(item_price * quantity) + "원");
        total_price += item_price * quantity;
    }
    total_label_->setText("현재 가격: " + QString::number(total_price) + "원");
}

double KioskNode::get_item_price(const QString &item_name) {
    for (const auto &item : menu_data_["피자"]) {
        if (item.at("name") == item_name) {
            return item.at("price").toDouble();
        }
    }
    for (const auto &item : menu_data_["음료"]) {
        if (item.at("name") == item_name) {
            return item.at("price").toDouble();
        }
    }
    for (const auto &item : menu_data_["기타"]) {
        if (item.at("name") == item_name) {
            return item.at("price").toDouble();
        }
    }
    return 0;
}

void KioskNode::place_order() {
    int table_number = table_dropdown_->currentText().toInt();
    if (cart_.empty()) {
        QMessageBox::warning(this, "경고", "장바구니에 항목이 없습니다.");
        return;
    }

    // 데이터베이스 연결
    sqlite3 *db;
    if (sqlite3_open(db_path_.toStdString().c_str(), &db) != SQLITE_OK) {
        qDebug() << "Cannot open database:" << sqlite3_errmsg(db);
        return;
    }

    // 주문 추가
    QString insert_order_query = "INSERT INTO Orders (table_number, status) VALUES (?, 'Pending');";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, insert_order_query.toStdString().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        qDebug() << "Failed to prepare statement:" << sqlite3_errmsg(db);
        sqlite3_close(db);
        return;
    }
    sqlite3_bind_int(stmt, 1, table_number);
    if (sqlite3_step(stmt) != SQLITE_DONE) {
        qDebug() << "Failed to execute statement:" << sqlite3_errmsg(db);
        sqlite3_finalize(stmt);
        sqlite3_close(db);
        return;
    }
    sqlite3_finalize(stmt);

    // 방금 추가된 주문의 ID 가져오기
    int order_id = sqlite3_last_insert_rowid(db);

    // 주문 상세 추가 및 주문 메시지 구성
    QString order_details_text;
    double total_price = 0;
    int detail_id = 0;

    for (const auto &entry : cart_) {
        QString item_name = entry.first;
        int quantity = entry.second;
        double item_price = get_item_price(item_name);
        double item_total_price = item_price * quantity;
        total_price += item_total_price;
        order_details_text += item_name + " x" + QString::number(quantity) + " (" + QString::number(item_total_price) + "원)\n";

        // OrderDetails 테이블에 추가
        QString insert_detail_query = "INSERT INTO OrderDetails (order_id, item_name, quantity, total_price) VALUES (?, ?, ?, ?);";
        if (sqlite3_prepare_v2(db, insert_detail_query.toStdString().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
            qDebug() << "Failed to prepare statement:" << sqlite3_errmsg(db);
            sqlite3_close(db);
            return;
        }
        sqlite3_bind_int(stmt, 1, order_id);
        sqlite3
