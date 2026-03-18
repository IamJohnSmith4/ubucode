# robot_server.py — เพิ่มตรงนี้
import json, os

POSITION_FILE = "/tmp/robot_last_position.json"

def save_position(node):
    """บันทึกตำแหน่งล่าสุดลงไฟล์"""
    with open(POSITION_FILE, 'w') as f:
        json.dump({"current_location": node}, f)

def load_position():
    """โหลดตำแหน่งล่าสุดตอนเปิดโปรแกรม"""
    if os.path.exists(POSITION_FILE):
        try:
            with open(POSITION_FILE, 'r') as f:
                data = json.load(f)
                return data.get("current_location", 1)
        except:
            pass
    return 1  # default = Home

# โหลดตำแหน่งตอนเริ่ม
current_location = load_position()
