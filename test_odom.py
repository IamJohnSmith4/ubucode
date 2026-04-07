from flask import Flask, request, jsonify
import time

app = Flask(__name__)

print("=========================================")
print("  Robot Mock API Server (โหมดทดสอบเครือข่าย)  ")
print("=========================================")

@app.route('/status', methods=['GET'])
def get_status():
    # จำลองการส่งพิกัด Odometry กลับไปให้ไฟล์ทดสอบ
    return jsonify({
        "is_navigating": False,
        "current_location": 1,
        "current_progress": 0,
        "position": {"x": 1.25, "y": 0.50},
        "yaw_deg": 90.0
    })

@app.route('/command', methods=['POST'])
def handle_command():
    data = request.json
    start = data.get('start', 1)
    target = data.get('target', 2)
    
    # แสดงข้อความบนจอหุ่นยนต์ว่าได้รับคำสั่งแล้ว
    print(f"[API] ได้รับคำสั่งให้วิ่งจาก Node {start} -> Node {target}")
    
    # ตอบกลับทันทีเพื่อให้ฝั่งคอมพิวเตอร์จับเวลา Latency ได้
    return jsonify({"status": "starting"}), 200

@app.route('/stop', methods=['POST', 'GET'])
def stop_robot():
    print("[API] ได้รับคำสั่ง: STOP (หยุดฉุกเฉิน)")
    return jsonify({"status": "success", "message": "Stopped"}), 200

if __name__ == "__main__":
    # รันบน IP ของหุ่นยนต์ พอร์ต 5000
    app.run(host='0.0.0.0', port=5000, threaded=True)
