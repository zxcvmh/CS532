import re

bridge_path = "/home/jetbot/mhieu/jetbot_bridge.py"
with open(bridge_path, "r") as f:
    code = f.read()

# Fix serial read trap
old_read = "chunk = ser.read(ser.in_waiting or 47)"
new_read = """n = ser.in_waiting
                    if n > 0:
                        chunk = ser.read(n)
                    else:
                        time.sleep(0.005)
                        continue"""

if old_read in code:
    code = code.replace(old_read, new_read)
    with open(bridge_path, "w") as f:
        f.write(code)
    print("[SUCCESS] Đã vá jetbot_bridge.py sang Non-blocking Serial read thành công!")
else:
    print("[INFO] Đã được vá từ trước hoặc không tìm thấy chuỗi cũ.")
