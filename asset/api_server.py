from flask import Flask, jsonify, request
import sqlite3
from datetime import datetime

app = Flask(__name__)

# SQLite 데이터베이스 연결 함수
def get_db_connection():
    conn = sqlite3.connect('/home/viator/ws/b3p1_ws/asset/pizza.db')  # 실제 DB 경로로 수정 필요
    conn.row_factory = sqlite3.Row
    return conn

# 기본 페이지: '/'
@app.route('/')
def index():
    return "Welcome to the Pizza Order API!"

# 주문 상태를 'canceled'로 업데이트
@app.route('/api/order/<int:order_id>/Cancelled', methods=['PUT'])
def cancel_order(order_id):
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # 주문 상태를 'canceled'로 업데이트
    cursor.execute("UPDATE Orders SET status = ? WHERE order_id = ?", ('Cancelled', order_id))
    conn.commit()
    conn.close()
    
    return jsonify({"message": f"Order with ID {order_id} has been Canceled."}), 200

# 주문 상태를 'completed'로 업데이트
@app.route('/api/order/<int:order_id>/Completed', methods=['PUT'])
def complete_order(order_id):
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # 주문 상태를 'completed'로 업데이트
    cursor.execute("UPDATE Orders SET status = ? WHERE order_id = ?", ('Completed', order_id))
    conn.commit()
    conn.close()
    
    return jsonify({"message": f"Order with ID {order_id} has been Completed."}), 200

# 주문 목록을 가져오기 (Pending 상태의 주문만)
@app.route('/api/orders', methods=['GET'])
def get_orders():
    conn = get_db_connection()
    cursor = conn.cursor()
    
    query = """
    SELECT o.order_id, o.timestamp, o.table_number, od.item_name, od.quantity
    FROM orders o
    JOIN orderdetails od ON o.order_id = od.order_id
    WHERE o.status = 'Pending'
    """
    cursor.execute(query)
    rows = cursor.fetchall()
    
    orders = {}
    for order_id, timestamp, table_number, item_name, quantity in rows:
        if order_id not in orders:
            orders[order_id] = {
                "timestamp": timestamp[11:16],  # HH:MM 형식으로 변환
                "table_number": table_number,
                "menu_items": []
            }
        orders[order_id]["menu_items"].append(f"{item_name} x {quantity}")
    
    conn.close()
    
    return jsonify(orders)

@app.route('/api/sales/<sales_type>', methods=['GET'])
def get_sales(sales_type):
    db_path = "pizza.db"
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    now = datetime.now()
    current_year = now.year
    current_month = now.month
    current_day = now.strftime('%Y-%m-%d')

    sales_data = {}

    if sales_type == "DAY":
        labels = [str(day) for day in range(1, 32)]
        query = f"""
            SELECT strftime('%d', o.timestamp) AS day, SUM(od.total_price)
            FROM orders o
            JOIN orderdetails od ON o.order_id = od.order_id
            WHERE o.status = 'Completed' AND strftime('%Y-%m', o.timestamp) = '{current_year:04d}-{current_month:02d}'
            GROUP BY day
        """
        cursor.execute(query)
        data = cursor.fetchall()
        sales_data = {int(row[0]): row[1] for row in data}
        sales = [sales_data.get(day, 0) for day in range(1, 32)]
        return jsonify(labels=labels, sales=sales)

    elif sales_type == "MONTH":
        labels = [str(month) for month in range(1, 13)]
        query = f"""
            SELECT strftime('%m', o.timestamp) AS month, SUM(od.total_price)
            FROM orders o
            JOIN orderdetails od ON o.order_id = od.order_id
            WHERE o.status = 'Completed' AND strftime('%Y', o.timestamp) = '{current_year:04d}'
            GROUP BY month
        """
        cursor.execute(query)
        data = cursor.fetchall()
        sales_data = {int(row[0]): row[1] for row in data}
        sales = [sales_data.get(month, 0) for month in range(1, 13)]
        return jsonify(labels=labels, sales=sales)

    elif sales_type == "YEAR":
        labels = [str(year) for year in range(current_year - 10, current_year + 1)]
        query = f"""
            SELECT strftime('%Y', o.timestamp) AS year, SUM(od.total_price)
            FROM orders o
            JOIN orderdetails od ON o.order_id = od.order_id
            WHERE o.status = 'Completed' AND strftime('%Y', o.timestamp) BETWEEN '{current_year - 10:04d}' AND '{current_year:04d}'
            GROUP BY year
        """
        cursor.execute(query)
        data = cursor.fetchall()
        sales_data = {int(row[0]): row[1] for row in data}
        sales = [sales_data.get(year, 0) for year in range(current_year - 10, current_year + 1)]
        return jsonify(labels=labels, sales=sales)

    else:
        return jsonify({"error": "Invalid sales type"}), 400

@app.route('/api/menu_sales/<sales_type>', methods=['GET'])
def get_menu_sales(sales_type):
    db_path = "pizza.db"
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    now = datetime.now()
    current_year = now.year
    current_month = now.month
    current_day = now.strftime('%Y-%m-%d')

    query_menu = ""
    if sales_type == "DAY":
        query_menu = f"""
            SELECT m.item_name, COALESCE(SUM(od.total_price), 0)
            FROM Menu m
            LEFT JOIN OrderDetails od ON m.item_name = od.item_name
            LEFT JOIN orders o ON od.order_id = o.order_id AND o.status = 'Completed'
            WHERE strftime('%Y-%m-%d', o.timestamp) = '{current_day}'
            GROUP BY m.item_id, m.item_name
        """
    elif sales_type == "MONTH":
        query_menu = f"""
            SELECT m.item_name, COALESCE(SUM(od.total_price), 0)
            FROM Menu m
            LEFT JOIN OrderDetails od ON m.item_name = od.item_name
            LEFT JOIN orders o ON od.order_id = o.order_id AND o.status = 'Completed'
            WHERE strftime('%Y-%m', o.timestamp) = '{current_year:04d}-{current_month:02d}'
            GROUP BY m.item_id, m.item_name
        """
    elif sales_type == "YEAR":
        query_menu = f"""
            SELECT m.item_name, COALESCE(SUM(od.total_price), 0)
            FROM Menu m
            LEFT JOIN OrderDetails od ON m.item_name = od.item_name
            LEFT JOIN orders o ON od.order_id = o.order_id AND o.status = 'Completed'
            WHERE strftime('%Y', o.timestamp) = '{current_year:04d}'
            GROUP BY m.item_id, m.item_name
        """
    else:
        return jsonify({"error": "Invalid sales type"}), 400

    cursor.execute(query_menu)
    sold_menu_data = dict(cursor.fetchall())
    menu_sales = list(sold_menu_data.values())
    menu_names = list(sold_menu_data.keys())

    return jsonify(menu_names=menu_names, menu_sales=menu_sales)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
