from flask import Flask, jsonify

app = Flask(__name__)

print("=========================================")
print("  Robot API Server (รองรับ /move, /rotate)  ")
print("=========================================")

# 1. รับคำสั่ง เดิน (Move)
@app.route('/move', methods=['GET', 'POST'])
def move_robot():
    print("[API] ได้รับคำสั่ง: เดิน (Move)")
    return jsonify({"status": "success", "action": "move"}), 200

# 2. รับคำสั่ง หมุน (Rotate)
@app.route('/rotate', methods=['GET', 'POST'])
def rotate_robot():
    print("[API] ได้รับคำสั่ง: หมุนตัว (Rotate)")
    return jsonify({"status": "success", "action": "rotate"}), 200

if __name__ == "__main__":
    # รันเซิร์ฟเวอร์ที่พอร์ต 5000
    app.run(host='0.0.0.0', port=5000, threaded=True)
