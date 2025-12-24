import cv2
import mediapipe as mp
import numpy as np
from datetime import datetime

CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
MIN_DETECTION_CONFIDENCE = 0.7
MIN_TRACKING_CONFIDENCE = 0.5

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=MIN_DETECTION_CONFIDENCE,
    min_tracking_confidence=MIN_TRACKING_CONFIDENCE
)
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

open_hand_angles = None
closed_hand_angles = None
current_angles = {}

def calculate_finger_angles(landmarks):
    angles = {}
    
    finger_tips = {
        'Thumb': (4, 3, 2),      
        'Index': (8, 6, 5),
        'Middle': (12, 10, 9),
        'Ring': (16, 14, 13),
        'Pinky': (20, 18, 17)
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
                angle_deg = 90.0
            else:
                cos_angle = dot_product / (magnitude_v1 * magnitude_v2)
                cos_angle = np.clip(cos_angle, -1, 1)
                angle_rad = np.arccos(cos_angle)
                angle_deg = np.degrees(angle_rad)
            
            angles[finger_name] = angle_deg
        
        except Exception as e:
            print(f"Error calculating {finger_name} angle: {e}")
            angles[finger_name] = 0.0
    
    return angles

def draw_calibration_ui(frame, calibration_state):
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (500, 300), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    
    y_offset = 30
    instructions = [
        "CALIBRATION TOOL",
        "",
        "Instructions:",
        "1. Show FULLY OPEN hand (all fingers straight)",
        "2. Press 's' to save open hand position",
        "3. Show FULLY CLOSED hand (fist)",
        "4. Press 's' to save closed hand position",
        "5. Press 'q' to exit and generate report"
    ]
    
    for instruction in instructions:
        if instruction.startswith("CALIBRATION"):
            cv2.putText(frame, instruction, (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(frame, instruction, (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 25
    
    cv2.rectangle(frame, (10, 320), (500, 380), (0, 0, 0), -1)
    
    status_color = (0, 255, 0)
    if calibration_state == 0:
        status_text = "STATE: Waiting for open hand..."
        status_color = (0, 165, 255)
    elif calibration_state == 1:
        status_text = "STATE: Open hand saved âœ“"
        status_color = (0, 255, 0)
    elif calibration_state == 2:
        status_text = "STATE: Waiting for closed hand..."
        status_color = (0, 165, 255)
    else:
        status_text = "STATE: Calibration complete âœ“"
        status_color = (0, 255, 0)
    
    cv2.putText(frame, status_text, (20, 350),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
    
    return frame

def draw_finger_angles(frame, angles):
    overlay = frame.copy()
    cv2.rectangle(overlay, (frame.shape[1] - 250, 10), 
                 (frame.shape[1] - 10, 150), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    
    y_offset = 30
    cv2.putText(frame, "Current Angles:", (frame.shape[1] - 240, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    y_offset += 25
    
    for finger, angle in angles.items():
        cv2.putText(frame, f"{finger}: {angle:.1f}Â°", 
                   (frame.shape[1] - 240, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 20
    
    return frame

def print_calibration_report():
    
    print("\n" + "="*70)
    print("CALIBRATION REPORT")
    print("="*70)
    print(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    
    if open_hand_angles is None or closed_hand_angles is None:
        print("ERROR: Incomplete calibration data")
        return
    
    print("OPEN HAND POSITION (Fully Extended Fingers)")
    print("-" * 70)
    for finger, angle in open_hand_angles.items():
        print(f"  {finger:8s}: {angle:6.2f}Â°")
    
    print("\nCLOSED HAND POSITION (Fully Closed Fist)")
    print("-" * 70)
    for finger, angle in closed_hand_angles.items():
        print(f"  {finger:8s}: {angle:6.2f}Â°")
    
    print("\nANGLE RANGES (for servo mapping)")
    print("-" * 70)
    for finger in open_hand_angles:
        open_angle = open_hand_angles[finger]
        closed_angle = closed_hand_angles[finger]
        angle_range = abs(open_angle - closed_angle)
        print(f"  {finger:8s}: {min(open_angle, closed_angle):.2f}Â° to {max(open_angle, closed_angle):.2f}Â° (range: {angle_range:.2f}Â°)")
    
    print("\n" + "="*70 + "\n")

def main():
    global open_hand_angles, closed_hand_angles, current_angles
    
    calibration_state = 0
    
    print("\n" + "="*70)
    print("PROSTHETIC HAND CALIBRATION TOOL")
    print("="*70)
    print("\nInstructions:")
    print("1. Show your hand fully OPEN (all fingers extended)")
    print("2. Press 's' to save the open hand position")
    print("3. Show your hand as a FIST (fully closed)")
    print("4. Press 's' to save the closed hand position")
    print("5. Press 'q' to exit and generate calibration report")
    print("\n" + "="*70 + "\n")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to read from camera")
                break
            
            frame = cv2.flip(frame, 1)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb_frame)
            
            if results.multi_hand_landmarks:
                landmarks = results.multi_hand_landmarks[0].landmark
                current_angles = calculate_finger_angles(landmarks)
                mp_drawing.draw_landmarks(
                    frame,
                    results.multi_hand_landmarks[0],
                    mp_hands.HAND_CONNECTIONS
                )
            
            frame = draw_calibration_ui(frame, calibration_state)
            frame = draw_finger_angles(frame, current_angles)
            cv2.imshow("Prosthetic Hand Calibration", frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s'):
                if not current_angles:
                    print("No hand detected. Please show your hand.")
                else:
                    if calibration_state == 0:
                        open_hand_angles = current_angles.copy()
                        print("\nâœ“ Open hand position saved!")
                        print("  Angles:", {k: f"{v:.2f}Â°" for k, v in open_hand_angles.items()})
                        calibration_state = 1
                    elif calibration_state == 1 or calibration_state == 2:
                        closed_hand_angles = current_angles.copy()
                        print("\nâœ“ Closed hand position saved!")
                        print("  Angles:", {k: f"{v:.2f}Â°" for k, v in closed_hand_angles.items()})
                        calibration_state = 3
            
            elif key == ord('q'):
                print("\nExiting calibration...")
                break
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        hands.close()
        print_calibration_report()

if __name__ == "__main__":
    main()
