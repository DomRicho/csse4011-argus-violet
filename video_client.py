import socket
import time
import threading
import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import queue
import serial 
import mediapipe as mp
import cv2
import csv
import tensorflow as tf
from tensorflow import keras
 # <-- NEW

WIDTH = 240
HEIGHT = 240
BYTES_PER_FRAME = WIDTH * HEIGHT * 2

SERIAL_PORT = '/dev/ttyUSB0'  # <-- Change this to your actual port, e.g., '/dev/ttyUSB0'
BAUD_RATE = 921600

def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            raise ConnectionError("Socket closed")
        data += packet
    return data

class VideoClientGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("ESP32 Video Client")

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Serial connected on {SERIAL_PORT}")
        except serial.SerialException as e:
            self.serial_conn = None
            print("Serial connection failed:", e)

        # Top frame
        top_frame = tk.Frame(master)
        top_frame.pack(pady=5)

        self.connect_btn = tk.Button(top_frame, text="Connect", command=self.connect)
        self.connect_btn.pack()

        self.canvas = tk.Label(top_frame)
        self.canvas.pack()

        # Button frame
        self.button_frame = tk.Frame(master)
        self.button_frame.pack(pady=10)

        self.up_btn = tk.Button(self.button_frame, text="Up", width=10, command=lambda: self.send_command("UP"))
        self.up_btn.grid(row=0, column=1)

        self.left_btn = tk.Button(self.button_frame, text="Left", width=10, command=lambda: self.send_command("LEFT"))
        self.left_btn.grid(row=1, column=0)

        self.down_btn = tk.Button(self.button_frame, text="Down", width=10, command=lambda: self.send_command("DOWN"))
        self.down_btn.grid(row=1, column=1)

        self.right_btn = tk.Button(self.button_frame, text="Right", width=10, command=lambda: self.send_command("RIGHT"))
        self.right_btn.grid(row=1, column=2)

        self.running = False
        self.sock = None
        self.frame_queue = queue.Queue(maxsize=5)
        self.model_queue = queue.Queue(maxsize=5)
        
    def send_command(self, direction):
        print(f"Sending command: {direction}")
        # Send via Serial
        if self.serial_conn and self.serial_conn.is_open:
            try:
                if (direction == "UP"): 
                    self.serial_conn.write(b"U1")
                elif (direction == "DOWN"): 
                    self.serial_conn.write(b"D1")
                elif (direction == "RIGHT"): 
                    self.serial_conn.write(b"R1")
                elif (direction == "LEFT"): 
                    self.serial_conn.write(b"L1")
                elif (direction == "CENTRE"):
                    self.serial_conn.write(b"C0")
                else:
                    print("invalid direction")
            except Exception as e:
                print("Serial send failed:", e)

    def connect(self):
        if self.running:
            return
        self.running = True
        threading.Thread(target=self.network_thread, daemon=True).start()
        threading.Thread(target=self.model_thread, daemon=True).start()
        self.master.after(10, self.display_loop)

    def bgr565_to_rgb888(self, frame):
        b = ((frame & 0xF800) >> 11).astype(np.uint8)
        g = ((frame & 0x07E0) >> 5).astype(np.uint8)
        r = (frame & 0x001F).astype(np.uint8)
        r = (r << 3) | (r >> 2)
        g = (g << 2) | (g >> 4)
        b = (b << 3) | (b >> 2)
        rgb = np.dstack((b, g, r))
        return rgb

    def network_thread(self):
        try:
            frame = bytearray(115200)
            old_id = -1  # start with an invalid frame_id
            print("network_thread running")
            while self.running:
                
                data = self.serial_conn.read_until(expected=b'F0000', size=528)  # or read_until if there's a reliable marker
                if len(data) != 528:
                    continue  # ignore incomplete chunks

                try:
                    frame_info = data[:11].decode("utf-8")  # Assuming header is 16 bytes
                    frame_chunk = int(frame_info[-3:])
                    frame_id = int(frame_info[:-3])
                except Exception as e:
                    print(f"Parse error: {e}")
                    continue

                if frame_id != old_id:
                    # New frame: send last one to display
                    if old_id != -1 and not self.frame_queue.full():
                        self.frame_queue.put_nowait(frame)
                        print("Frame", old_id)
                    frame = bytearray(115200)
                    old_id = frame_id

                idx = frame_chunk * 512
                if idx + 512 <= len(frame):
                    frame[idx:idx + 512] = data[12:-4]
                else:
                    print("Warning: Chunk index out of range")
        except Exception as e:
            print("Network error:", e)
        finally:
            self.running = False

    def display_loop(self):
        if not self.running:
            return
        try:
            data = self.frame_queue.get_nowait()
            frame = np.frombuffer(data, dtype='<u2').reshape((HEIGHT, WIDTH))
            # frame = frame.byteswap()
            rgb = self.bgr565_to_rgb888(frame)
            rgb = cv2.flip(rgb, 0)
            rgb = cv2.flip(rgb, 1)
        
            self.model_queue.put(rgb)

            img = Image.fromarray(rgb, 'RGB')
            imgtk = ImageTk.PhotoImage(image=img)
            self.canvas.imgtk = imgtk
            self.canvas.configure(image=imgtk)
        except queue.Empty:
            pass
        self.master.after(1, self.display_loop)
    
    def on_close(self):
            self.running = False
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.master.destroy()

    def model_thread(self):
        # Initialize MediaPipe Hands and Drawing utilities
        mp_drawing = mp.solutions.drawing_utils
        mp_hands = mp.solutions.hands

        #capture = cv2.VideoCapture(0)

        recording = False
        gesture_number = 0

        gestures = ['Open hand', 'Closed hand', 'Down', 'Up', 'Left', 'Right']

        model = self.load_model()
        pause = 0
        pause_count = 0
        last_check = time.time()

        with mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.8, min_tracking_confidence=0.5, static_image_mode='store_true') as hands:
            while self.running:
                #print("Model thread running...")
                # Check for key presses
                key = cv2.waitKey(1) & 0xFF
                gesture_number = -1  # Reset gesture number if no valid key is pressed
                predicted_gesture_num = -1
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    # Start recording data
                    recording = not recording
                    if recording:
                        print("Recording data...")
                    else:
                        print("Stopped recording data.")
                elif ord('0') <= key <= ord('1'):
                    gesture_number = key - ord('0')
                    print(f"Gesture number set to: {gesture_number}")

                # Capture frame-by-frame
                #ret, frame = capture.read()
                #GET FRAME
                try:
                    frame = self.model_queue.get_nowait()
                except queue.Empty:
                    continue

                #frame = cv2.flip(frame, 1)

                # Resize the frame to 240x240
                #frame = cv2.resize(frame, (240, 240))
                
                #image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                #image = cv2.normalize(image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                # Process the image and detect hands
                detected_image = hands.process(frame)

                image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                if pause:
                    pause_count = pause_count + 1
                    print("skipping..")
                    if pause_count >= 2:
                        print("ready")
                        pause_count = 0
                        pause = 0
                    continue    

                # Draw hand landmarks and connections if hands are detected
                if detected_image.multi_hand_landmarks:

                    absolute_landmarks = self.normalize_landmarks(image, detected_image.multi_hand_landmarks)

                    # Predict gesture 
                    predicted_gesture_num = self.predict_gesture(model, absolute_landmarks)
                    predicted_gesture = gestures[predicted_gesture_num]
                    print(f"Predicted Gesture: {predicted_gesture}")

                    # if closed hand, get positon and direction of centre frame
                    pixel_landmarks = self.get_landmarks_pixels(image, detected_image.multi_hand_landmarks)
                    hand_position = self.get_hand_position(pixel_landmarks)

                    cv2.circle(image, hand_position, 5, (0, 255, 0), -1)
                    
                    centre = self.get_centre(image)
                    cv2.circle(image, centre, 5, (255, 0, 0), -1)

                    cv2.line(image, centre, hand_position, (0, 255, 0), 2)
                    
                    distance_from_centre = self.get_distance_from_centre(hand_position, image)

                    print(f"Distance from center: {distance_from_centre}")
                    
                    
                    if (predicted_gesture_num):
                        direction = self.get_directions(image, hand_position)
                        
                        servo_cmd = self.get_servo_cmd(hand_position, predicted_gesture_num)

                        if servo_cmd[0] is not None:
                            #for i in range(servo_cmd_nums[0]):
                            self.send_command(servo_cmd[0])
                            # pause = 1
                        if servo_cmd[1] is not None:
                            #for i in range(servo_cmd_nums[1]):
                            self.send_command(servo_cmd[1])
                            # pause = 1
                    else:
                        direction = None

                        
                    # Display the predicted gesture on the image
                    cv2.putText(image, f"Gesture: {predicted_gesture}", 
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    if direction is not None:
                        cv2.putText(image, f"Direction: {direction}",
                                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    for hand_lms in detected_image.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(image, hand_lms,
                                                    mp_hands.HAND_CONNECTIONS,
                                                    landmark_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(
                                                        color=(255, 0, 255), thickness=2, circle_radius=1),
                                                    connection_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(
                                                        color=(20, 180, 90), thickness=1, circle_radius=1)
                                                    )
                    # Record data if recording is enabled
                    self.record_data(recording, gesture_number, absolute_landmarks)
                cv2.imshow('Webcam', image)   
        #capture.release()
        cv2.destroyAllWindows()
        self.running = False

    def get_landmarks_pixels(self, image, landmarks):
        """
        Extracts the pixel coordinates of the landmarks from the detected hand landmarks.
        """
        height, width, _ = image.shape
        pixel_landmarks = []
        for hand_landmarks in landmarks:
            for landmark in hand_landmarks.landmark:
                pixel_x = int(landmark.x * width)
                pixel_y = int(landmark.y * height)
                pixel_landmarks.append((pixel_x, pixel_y))
        return pixel_landmarks

    def normalize_landmarks(self, image, landmarks):
        """
        Get relative landmarks
        """
        landmarks_pos_list = []
        base_x, base_y = 0, 0
        if not landmarks:
            return []

        landmarks_pos_list = self.get_landmarks_pixels(image, landmarks)
        relative_landmarks = []

        for landmark in landmarks_pos_list:
            if landmark == landmarks_pos_list[0]:
                base_x = landmark[0]
                base_y = landmark[1]
            
            relative_x = abs(landmark[0] - base_x)
            relative_y = abs(landmark[1] - base_y)

            relative_landmarks.append(relative_x)
            relative_landmarks.append(relative_y)

        # Normalize the landmarks to a range of 0 to 1
        max_val = max(relative_landmarks)

        normalized_landmarks = []

        for landmark in relative_landmarks:
            normalized_landmark = landmark / max_val
            normalized_landmarks.append(normalized_landmark)

        return normalized_landmarks

    def record_data(self, recording, gesture_number, landmarks):
        """
        Record the normalized landmarks to a file.
        """
        csv_file = 'data/landmarks.csv'
        if recording and gesture_number != -1:
            with open(csv_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([gesture_number, *landmarks])
        return

    def load_model(self):
        """
        Load the pre-trained model for gesture recognition.
        """
        model = keras.models.load_model('pc_gesture_model/model/model.keras')

        return model

    def predict_gesture(self, model, landmarks):
        """
        Predict the gesture based on the normalized landmarks.
        """
        if not landmarks:
            return None

        # Reshape landmarks for model input
        input_data = np.array([landmarks])
        
        # Predict gesture
        prediction = model.predict(input_data, verbose=0)
        predicted_gesture = np.argmax(np.squeeze(prediction))

        return predicted_gesture
        
    def get_hand_position(self, landmarks):
        """
        Get the position of the hand based on the landmarks.
        """
        if not landmarks:
            return None

        # Calculate the center of the hand
        x_coords = [landmark[0] for landmark in landmarks]
        y_coords = [landmark[1] for landmark in landmarks]

        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))

        return (center_x, center_y)

    def get_distance_from_centre(self, hand_position, image):
        """
        Calculate the distance of the hand position from the center of the image.
        """
        height, width, _ = image.shape
        center_x = width // 2
        center_y = height // 2

        if hand_position is None:
            return None

        distance = np.sqrt((hand_position[0] - center_x) ** 2 + (hand_position[1] - center_y) ** 2)
        
        return distance

    def get_centre(self, image):
        """
        Get the center of the image.
        """
        height, width, _ = image.shape
        center_x = width // 2
        center_y = height // 2

        return (center_x, center_y)

    def get_directions(self, image, hand_position):
        """
        Get the direction the needs to be moved to get to centre of the image.
        first moves to the centre line, then up or down

        centre is defined as a 30 x 30 pixel squre in the middle of the image

        """

        height, width, _ = image.shape
        center_x = width // 2
        center_y = height // 2

        hand_x, hand_y = hand_position

        centre_vertical_bounds = (center_x - 15, center_x + 15)

        cv2.line(image, (centre_vertical_bounds[0], 0), (centre_vertical_bounds[0], height), (255, 0, 0), 2)
        cv2.line(image, (centre_vertical_bounds[1], 0), (centre_vertical_bounds[1], height), (255, 0, 0), 2)
        

        centre_horizontal_bounds = (center_y - 15, center_y + 15)
        cv2.line(image, (0, centre_horizontal_bounds[0]), (width, centre_horizontal_bounds[0]), (255, 0, 0), 2)
        cv2.line(image, (0, centre_horizontal_bounds[1]), (width, centre_horizontal_bounds[1]), (255, 0, 0), 2)

        if centre_vertical_bounds[0] <= hand_x <= centre_vertical_bounds[1]:
            if centre_horizontal_bounds[0] <= hand_y <= centre_horizontal_bounds[1]:
                return "Centre"
            elif hand_y < centre_horizontal_bounds[0]:
                return "Down"
            else:
                return "Up"
        else:
            if hand_x < centre_vertical_bounds[0]:
                return "Right"
            else:
                return "Left"        
    def send_servo_cmd(self, ser, cmd):
        """
        Send the servo command.
        """
        if ser is not None:
            try:
                ser.write(cmd.encode())
                print(f"Sent command: {cmd}")
            except serial.SerialException as e:
                print(f"Error sending command: {e}")
        else:
            print("Serial port is not open.")
    
    def get_servo_cmd(self, postion, gesture_num):
        """
        Get the servo command based on the distance from the center.
        """
        x,y = postion
        x_cmd = None
        y_cmd = None

        if gesture_num == 1:
            if 145 < x <= 240:
                x_cmd = "LEFT"
            elif 95 < x <= 145:
                x_cmd = None
            elif 95 >= x >= 0:
                x_cmd = "RIGHT"

            if 145 < y <= 240:
                y_cmd = "DOWN"
            elif 95 < y <= 145:
                y_cmd = None
            elif 95 >= y >= 0:
                y_cmd = "UP"
        elif gesture_num == 2:
            y_cmd = "DOWN"
        elif gesture_num == 3:
            y_cmd = "UP"
        elif gesture_num == 4:
            x_cmd = "LEFT"
        elif gesture_num == 5:
            x_cmd = "RIGHT"
        elif gesture_num == 0:
            x_cmd = "CENTRE"

        return (x_cmd, y_cmd)

if __name__ == "__main__":
    root = tk.Tk()
    app = VideoClientGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
