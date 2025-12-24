import cv2
import mediapipe as mp
import numpy as np
import serial
import time

ARDUINO_PORT = 'COM3'
BAUD_RATE = 9600
SERIAL_TIMEOUT = 1
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
MIN_DETECTION_CONFIDENCE = 0.7
MIN_TRACKING_CONFIDENCE = 0.5
SMOOTHING_FACTOR = 0.6

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=MIN_DETECTION_CONFIDENCE,
    min_tracking_confidence=MIN_TRACKING_CONFIDENCE
)
mp_drawing = mp.solutions.drawing_utils

arduino = None
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
    time.sleep(2)
    print(f"âœ“ Arduino connected on {ARDUINO_PORT}")
except Exception as e:
    print(f"âœ— Failed to connect to Arduino: {e}")
    arduino = None

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

prev_angles = {
    'thumb': 90,
    'index': 90,
    'middle': 90,
    'ring': 90,
    'pinky': 90
}

def calculate_finger_angles(landmarks):
    angles = {}
    finger_tips = {
        'thumb': (4, 3, 2),      
        'index': (8, 6, 5),
        'middle': (12, 10, 9),
        'ring': (16, 14, 13),
        'pinky': (20, 18, 17)
    }
    
    for finger_name, (tip, mid, base) in finger_tips.items():
        try:
            p1 = np.array([landmarks[base].x, landmarks[base].y, landmarks[base].z])
            p2 = np.array([landmarks[mid].x, landmarks[mid].y, landmarks[mid].z])
            p3 = np.array([landmarks[tip].x, landmarks[tip].y, landmarks[tip].z])
            
            v1 = p1 - p2
            v2 = p3 - p2
            
            dot_product = np.dot(v1, v2)
            magnitude_v1 = np.linalg.norm(v1)
            magnitude_v2 = np.linalg.norm(v2)
            
            if magnitude_v1 == 0 or magnitude_v2 == 0:
                servo_angle = 90
            else:
                cos_angle = dot_product / (magnitude_v1 * magnitude_v2)
                cos_angle = np.clip(cos_angle, -1, 1)
                angle_rad = np.arccos(cos_angle)
                angle_deg = np.degrees(angle_rad)
                servo_angle = int(np.interp(angle_deg, [0, 180], [180, 0]))
                servo_angle = max(0, min(180, servo_angle))
            
            angles[finger_name] = servo_angle
        
        except Exception as e:
            print(f"Error calculating {finger_name} angle: {e}")
            angles[finger_name] = prev_angles[finger_name]
    
    return angles

def apply_smoothing(current_angles, prev_angles, factor=SMOOTHING_FACTOR):
    smoothed = {}
    for finger in current_angles:
        smoothed[finger] = int(
            factor * prev_angles[finger] + (1 - factor) * current_angles[finger]
        )
    return smoothed

def send_to_arduino(angles):
    if arduino is None:
        return
    
    try:
        command = (
            f"T:{angles['thumb']},"
            f"I:{angles['index']},"
            f"M:{angles['middle']},"
            f"R:{angles['ring']},"
            f"P:{angles['pinky']}\n"
        )
        arduino.write(command.encode())
    except Exception as e:
        print(f"Error sending data to Arduino: {e}")

def draw_hand_info(frame, angles, hand_detected):
    if not hand_detected:
        cv2.putText(frame, "No hand detected", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return frame
    
    y_offset = 30
    cv2.putText(frame, "Servo Angles:", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    y_offset += 25
    
    for finger, angle in angles.items():
        cv2.putText(frame, f"{finger.capitalize()}: {angle}Â°", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y_offset += 25
    
    cv2.putText(frame, "Connected: Arduino Ready", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    cv2.putText(frame, "Press 'q' to quit", (10, frame.shape[0] - 20),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
    
    return frame

def main():
    global prev_angles
    
    print("\n" + "="*60)
    print("PROSTHETIC HAND GESTURE RECOGNITION SYSTEM")
    print("="*60)
    print(f"Camera: Camera {CAMERA_INDEX}")
    print(f"Arduino: {ARDUINO_PORT if arduino else 'NOT CONNECTED'}")
    print("\nPress 'q' to quit")
    print("="*60 + "\n")
    
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to read from camera")
                break
            
            frame_count += 1
            frame = cv2.flip(frame, 1)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            results = hands.process(rgb_frame)
            hand_detected = results.multi_hand_landmarks is not None
            
            if hand_detected:
                landmarks = results.multi_hand_landmarks[0].landmark
                raw_angles = calculate_finger_angles(landmarks)
                smooth_angles = apply_smoothing(raw_angles, prev_angles, SMOOTHING_FACTOR)
                send_to_arduino(smooth_angles)
                prev_angles = smooth_angles.copy()
                
                mp_drawing.draw_landmarks(
                    frame,
                    results.multi_hand_landmarks[0],
                    mp_hands.HAND_CONNECTIONS
                )
            
            frame = draw_hand_info(frame, prev_angles, hand_detected)
            cv2.imshow("Prosthetic Hand Control - Gesture Detection", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nShutting down...")
                break
            
            if frame_count % 30 == 0:
                status = "Hand detected âœ“" if hand_detected else "No hand detected"
                print(f"Frame {frame_count}: {status}")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    except Exception as e:
        print(f"\nError in main loop: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\nCleaning up...")
        cap.release()
        cv2.destroyAllWindows()
        
        if arduino:
            try:
                arduino.close()
                print("âœ“ Arduino connection closed")
            except:
                pass
        
        hands.close()
        print("âœ“ All resources released\n")

if __name__ == "__main__":
    main()
