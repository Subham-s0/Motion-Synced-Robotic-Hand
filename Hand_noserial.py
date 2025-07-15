#! Python3.9_version\Scripts\python.exe
import cv2
import mediapipe as mp
import math

# === Initialize Mediapipe Hands ===
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1, 
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_drawing = mp.solutions.drawing_utils

# === Tracking state - Default OFF ===
tracking_enabled = False

# === Function to calculate angle between three points ===
def calculate_angle(a, b, c):
    ba = (a[0] - b[0], a[1] - b[1])
    bc = (c[0] - b[0], c[1] - b[1])
    dot_product = ba[0]*bc[0] + ba[1]*bc[1]
    mag_ba = math.hypot(ba[0], ba[1])
    mag_bc = math.hypot(bc[0], bc[1])
    if mag_ba * mag_bc == 0:
        return 0
    angle = math.acos(dot_product / (mag_ba * mag_bc))
    return math.degrees(angle)

# === Function to determine if thumb is bent ===
def get_thumb_status(landmarks, angle_threshold=150):
    cmc = landmarks[mp_hands.HandLandmark.THUMB_CMC]
    mcp = landmarks[mp_hands.HandLandmark.THUMB_MCP]
    tip = landmarks[mp_hands.HandLandmark.THUMB_TIP]
    angle = calculate_angle(cmc, mcp, tip)
    return 1 if angle < angle_threshold else 0

# === Function to determine if a finger is bent ===
def is_finger_bent(joint1, joint2):
    return joint2[1] > joint1[1]  # TIP below the previous joint in y-axis

# === Start capturing video ===
cap = cv2.VideoCapture(0)

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # === Display control instructions ===
        status_text = "ON" if tracking_enabled else "OFF"
        status_color = (0, 255, 0) if tracking_enabled else (0, 0, 255)
        cv2.putText(frame, f"Press Y to start, N to stop | Status: {status_text}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        # === Handle key presses ===
        key = cv2.waitKey(1) & 0xFF
        if key == ord('y') or key == ord('Y'):
            tracking_enabled = True
            print("Tracking started")
        elif key == ord('n') or key == ord('N'):
            tracking_enabled = False
            print("Tracking stopped")
        elif key == ord('q'):
            break

        if tracking_enabled:
            results = hands.process(frame_rgb)
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    landmarks = [(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark]

                    fingers = {
                        "thumb": [mp_hands.HandLandmark.THUMB_IP, mp_hands.HandLandmark.THUMB_TIP],
                        "index": [mp_hands.HandLandmark.INDEX_FINGER_PIP, mp_hands.HandLandmark.INDEX_FINGER_TIP],
                        "middle": [mp_hands.HandLandmark.MIDDLE_FINGER_PIP, mp_hands.HandLandmark.MIDDLE_FINGER_TIP],
                        "ring": [mp_hands.HandLandmark.RING_FINGER_PIP, mp_hands.HandLandmark.RING_FINGER_TIP],
                        "pinky": [mp_hands.HandLandmark.PINKY_PIP, mp_hands.HandLandmark.PINKY_TIP],
                    }

                    finger_status = {}
                    for finger_name, indices in fingers.items():
                        if finger_name == 'thumb':
                            status = get_thumb_status(landmarks)
                        else:
                            joint1 = landmarks[indices[0]]
                            joint2 = landmarks[indices[1]]
                            status = 1 if is_finger_bent(joint1, joint2) else 0
                        finger_status[finger_name] = status

                    data_to_print = "$" + ''.join(str(finger_status[f]) for f in ["thumb", "index", "middle", "ring", "pinky"])
                    print("Finger data:", data_to_print)

                    # === Display on frame ===
                    y_offset = 70
                    for finger_name, status in finger_status.items():
                        cv2.putText(frame, f"{finger_name.capitalize()}: {status}",
                                    (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        y_offset += 30
        else:
            cv2.putText(frame, "Tracking disabled - Press Y to start",
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow('Gesture Tracking', frame)

finally:
    cap.release()
    cv2.destroyAllWindows()
