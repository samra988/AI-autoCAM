"""
=================================================================
AUTONOMOUS CAMERA CAR — DISTANCE-HOLD VERSION (1.5 FT TARGET)
=================================================================
PURE WIFI ARCHITECTURE — NO SERIAL, NO USB, NO TX/RX
=================================================================
"""

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

import cv2
import socket
import time
import numpy as np

from collections import deque
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# ===============================================================
# CONFIGURATION
# ===============================================================

ESP32_CAM_URL  = "http://10.205.197.165:81/stream"

ESP32_UDP_IP   = "10.205.197.12"
ESP32_UDP_PORT = 4210

# ===============================================================
# SETTINGS
# ===============================================================

FRAME_W, FRAME_H = 160, 120
FRAME_CX, FRAME_CY = FRAME_W // 2, FRAME_H // 2

PAN_CENTER = 90
TILT_CENTER = 90

PAN_MIN, PAN_MAX = 20, 160
TILT_MIN, TILT_MAX = 40, 140

SERVO_SMOOTH = 0.2

LOCK_STABLE_SECONDS = 0.5

# --- STABILIZED POWER DYNAMICS TO PREVENT TRACK DRIFT ---
BASE_SPEED = 140   # Dropped slightly to prevent high-momentum overshoots
MIN_SPEED  = 90    # Smooth crawl speed to guarantee precision tracking
MAX_SPEED  = 200

# ── HORIZONTAL TRACKING ──
H_DEADZONE = 30
V_DEADZONE = 25

# ── DISTANCE HOLD (1.5 FT TARGET) ──
TARGET_BBOX_W = 110

DIST_DEADZONE_OUT = 15   
DIST_DEADZONE_IN  = 6    

BBOX_SMOOTH = 0.3
SEND_INTERVAL = 0.20

# ===============================================================
# SENSOR SAFETY
# ===============================================================

sensor_data = {
    "F": 999,
    "L": 999,
    "R": 999
}

OBSTACLE_FRONT = 25
OBSTACLE_SIDE  = 15

# ===============================================================
# PID CONTROLLER
# ===============================================================

class PID:
    def __init__(self, Kp, Ki, Kd, limit=255):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limit = limit

        self._prev_error = 0
        self._integral = 0
        self._last_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = max(now - self._last_time, 0.01)

        self._integral += error * dt
        self._integral = max(-self.limit, min(self.limit, self._integral))

        derivative = (error - self._prev_error) / dt

        output = (
            self.Kp * error +
            self.Ki * self._integral +
            self.Kd * derivative
        )

        output = max(-self.limit, min(self.limit, output))

        self._prev_error = error
        self._last_time = now

        return output

    def reset(self):
        self._prev_error = 0
        self._integral = 0

pid_turn     = PID(Kp=0.45, Ki=0.01, Kd=0.08, limit=50)
pid_distance = PID(Kp=0.6,  Ki=0.0,  Kd=0.15, limit=(MAX_SPEED - BASE_SPEED))

# ===============================================================
# UDP COMMUNICATION
# ===============================================================

class Comms:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.001)
        print(f"[UDP] Connected → {ESP32_UDP_IP}:{ESP32_UDP_PORT}")

    def send(self, cmd):
        try:
            self.sock.sendto(
                (cmd + "\n").encode(),
                (ESP32_UDP_IP, ESP32_UDP_PORT)
            )
        except Exception as e:
            print("[UDP SEND ERROR]", e)

    def receive_sensor_data(self):
        global sensor_data
        try:
            data, _ = self.sock.recvfrom(128)
            line = data.decode().strip()
            if line.startswith("DIST:"):
                parts = line[5:].split(",")
                for p in parts:
                    k, v = p.split("=")
                    sensor_data[k.strip()] = int(v.strip())
        except:
            pass

    def close(self):
        self.sock.close()

# ===============================================================
# SERVO CONTROL
# ===============================================================

pan_angle  = PAN_CENTER
tilt_angle = TILT_CENTER

def update_servos(person_cx, person_cy):
    global pan_angle, tilt_angle

    error_x = person_cx - FRAME_CX
    error_y = person_cy - FRAME_CY

    if abs(error_x) < H_DEADZONE:
        target_pan = pan_angle
    else:
        target_pan = PAN_CENTER + error_x * 0.15

    if abs(error_y) < V_DEADZONE:
        target_tilt = tilt_angle
    else:
        target_tilt = TILT_CENTER - error_y * 0.15

    target_pan = max(PAN_MIN, min(PAN_MAX, target_pan))
    target_tilt = max(TILT_MIN, min(TILT_MAX, target_tilt))

    pan_angle  = (SERVO_SMOOTH * target_pan)  + ((1.0 - SERVO_SMOOTH) * pan_angle)
    tilt_angle = (SERVO_SMOOTH * target_tilt) + ((1.0 - SERVO_SMOOTH) * tilt_angle)

    return int(pan_angle), int(tilt_angle)

# ===============================================================
# MOVEMENT LOGIC — ANTI-TILT CONTINUOUS STEERING WORKING PARADIGM
# ===============================================================

dist_state = "OK"

def compute_command(person_cx, smoothed_bbox_w):
    global dist_state

    dist_f = sensor_data["F"]
    dist_l = sensor_data["L"]
    dist_r = sensor_data["R"]

    error_x = person_cx - FRAME_CX            # + = right, - = left
    error_d = smoothed_bbox_w - TARGET_BBOX_W  # + = too close, - = too far

    # ---------- STATE TRANSITIONS (hysteresis) ----------
    if dist_state == "OK":
        if error_d > DIST_DEADZONE_OUT:
            dist_state = "TOO_CLOSE"
        elif error_d < -DIST_DEADZONE_OUT:
            dist_state = "TOO_FAR"
    elif dist_state == "TOO_CLOSE":
        if error_d <= DIST_DEADZONE_IN:
            dist_state = "OK"
    elif dist_state == "TOO_FAR":
        if error_d >= -DIST_DEADZONE_IN:
            dist_state = "OK"

    # ---------- TOO CLOSE -> SMOOTH CRAWL BACKUP ----------
    if dist_state == "TOO_CLOSE":
        pid_turn.reset()
        out = pid_distance.compute(error_d)
        speed = int(BASE_SPEED + abs(out))
        speed = max(MIN_SPEED, min(MAX_SPEED, speed))
        return f"CMD:BACKWARD,{speed},{speed}"

    # ---------- TOO FAR -> MOVE FORWARD (+ blended turning) ----------
    if dist_state == "TOO_FAR":
        if dist_f < OBSTACLE_FRONT:
            pid_distance.reset()
            pid_turn.reset()
            return "CMD:STOP,0,0"

        out = pid_distance.compute(error_d)
        speed = int(BASE_SPEED + abs(out))
        speed = max(MIN_SPEED, min(MAX_SPEED, speed))

        if abs(error_x) <= H_DEADZONE:
            pid_turn.reset()
            return f"CMD:FORWARD,{speed},{speed}"

        turn_out = pid_turn.compute(error_x)
        if turn_out > 0 and dist_r < OBSTACLE_SIDE: turn_out = 0
        if turn_out < 0 and dist_l < OBSTACLE_SIDE: turn_out = 0

        left  = int(max(MIN_SPEED, min(MAX_SPEED, speed + turn_out)))
        right = int(max(MIN_SPEED, min(MAX_SPEED, speed - turn_out)))
        return f"CMD:FORWARD,{left},{right}"

    # ---------- DISTANCE OK -> LINEAR DIFFERENTIAL STEERING ----------
    # [FIX] Replaced high jhatka pivot (CMD:LEFT/RIGHT) with smooth forward differential curving
    pid_distance.reset()

    if abs(error_x) <= H_DEADZONE:
        pid_turn.reset()
        return "CMD:STOP,0,0"

    turn_out = pid_turn.compute(error_x)
    
    # Smooth curvature mapping using differential drive instead of hard spot turns
    left_pwm  = int(BASE_SPEED + turn_out)
    right_pwm = int(BASE_SPEED - turn_out)
    
    left_pwm  = max(MIN_SPEED, min(MAX_SPEED, left_pwm))
    right_pwm = max(MIN_SPEED, min(MAX_SPEED, right_pwm))

    if error_x > 0 and dist_r < OBSTACLE_SIDE: return "CMD:STOP,0,0"
    if error_x < 0 and dist_l < OBSTACLE_SIDE: return "CMD:STOP,0,0"

    return f"CMD:FORWARD,{left_pwm},{right_pwm}"

# ===============================================================
# MAIN
# ===============================================================

def main():
    global dist_state, TARGET_BBOX_W

    print("=" * 60)
    print("AUTONOMOUS CAMERA CAR — DISTANCE HOLD (1.5 FT)")
    print(f"TARGET_BBOX_W = {TARGET_BBOX_W}  (press 'c' to calibrate)")
    print("=" * 60)

    model = YOLO("yolov8n.pt", verbose=False)

    tracker = DeepSort(
        max_age=30,
        n_init=2,
        max_cosine_distance=0.3,
        nn_budget=100
    )

    comms = Comms()
    cap = cv2.VideoCapture(ESP32_CAM_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("[ERROR] Cannot connect to ESP32-CAM")
        return

    locked_id = None
    lock_candidate = None
    lock_start_time = 0

    fps_deque = deque(maxlen=30)
    prev_time = time.time()
    last_send_time = 0

    smoothed_bbox_w = float(TARGET_BBOX_W)

    while True:
        now = time.time()
        comms.receive_sensor_data()

        fps_deque.append(1.0 / max(now - prev_time, 0.001))
        fps = np.mean(fps_deque)
        prev_time = now

        ret, frame = cap.read()

        if not ret:
            print("[CAMERA] Stream lost")
            comms.send(f"CMD:STOP,0,0,PAN:{PAN_CENTER},TILT:{TILT_CENTER}")
            cap.release()
            reconnect_ok = False

            for _ in range(20):
                cap = cv2.VideoCapture(ESP32_CAM_URL)
                if cap.isOpened():
                    reconnect_ok = True
                    print("[CAMERA] Reconnected")
                    break
                cv2.waitKey(1)
                time.sleep(0.2)

            if not reconnect_ok:
                frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
                cv2.putText(frame, "CAMERA DISCONNECTED", (20, FRAME_H // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("AI Camera Car", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            continue

        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        results = model(frame, classes=[0], verbose=False)[0]
        detections = []

        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < 0.45:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append(([x1, y1, x2 - x1, y2 - y1], conf, "person"))

        tracks = tracker.update_tracks(detections, frame=frame)
        confirmed_tracks = [t for t in tracks if t.is_confirmed()]

        if locked_id is not None:
            exists = any(t.track_id == locked_id for t in confirmed_tracks)
            if not exists:
                locked_id = None
                pid_turn.reset()
                pid_distance.reset()

        if locked_id is None:
            best = None
            best_area = 0
            for t in confirmed_tracks:
                x1, y1, x2, y2 = t.to_ltrb()
                area = (x2 - x1) * (y2 - y1)
                if area > best_area:
                    best_area = area
                    best = t

            if best:
                if lock_candidate != best.track_id:
                    lock_candidate = best.track_id
                    lock_start_time = now

                if now - lock_start_time >= LOCK_STABLE_SECONDS:
                    locked_id = lock_candidate

        # [FIX] Default command loops keep continuous central stabilization signals 
        command = f"CMD:STOP,0,0,PAN:{PAN_CENTER},TILT:{TILT_CENTER}"

        locked_track = next((t for t in confirmed_tracks if t.track_id == locked_id), None)
        if locked_track:
            x1, y1, x2, y2 = locked_track.to_ltrb()
            person_cx = int((x1 + x2) / 2)
            person_cy = int((y1 + y2) / 2)
            bbox_w = int(x2 - x1)

            smoothed_bbox_w = (BBOX_SMOOTH * bbox_w) + ((1 - BBOX_SMOOTH) * smoothed_bbox_w)

            pan, tilt = update_servos(person_cx, person_cy)
            movement = compute_command(person_cx, smoothed_bbox_w)
            command = f"{movement},PAN:{pan},TILT:{tilt}"

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
            cv2.putText(frame, f"LOCKED ID:{locked_id}", (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            cv2.rectangle(frame, (FRAME_CX - H_DEADZONE, FRAME_CY - V_DEADZONE),
                          (FRAME_CX + H_DEADZONE, FRAME_CY + V_DEADZONE), (0, 255, 0), 1)

            cv2.putText(frame, f"bbox_w:{int(smoothed_bbox_w)} target:{int(TARGET_BBOX_W)} [{dist_state}]",
                        (10, FRAME_H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 200, 0), 1)
        else:
            pid_turn.reset()
            pid_distance.reset()

        if now - last_send_time >= SEND_INTERVAL:
            comms.send(command)
            last_send_time = now

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, command, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
        cv2.imshow("AI Camera Car", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            locked_id = None
            lock_candidate = None
            pid_turn.reset()
            pid_distance.reset()
            dist_state = "OK"
        elif key == ord('c'):
            TARGET_BBOX_W = int(smoothed_bbox_w)
            dist_state = "OK"
            pid_distance.reset()
            print(f"[CALIBRATE] TARGET_BBOX_W set to {TARGET_BBOX_W} (1.5ft reference)")

    comms.send(f"CMD:STOP,0,0,PAN:{PAN_CENTER},TILT:{TILT_CENTER}")
    cap.release()
    cv2.destroyAllWindows()
    comms.close()

if __name__ == "__main__":
    main()
