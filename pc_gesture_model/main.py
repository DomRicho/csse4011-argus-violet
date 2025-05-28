import mediapipe as mp
import cv2
import numpy as np
import csv

import tensorflow as tf
from tensorflow import keras

def main():

    # Initialize MediaPipe Hands and Drawing utilities
    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands

    capture = cv2.VideoCapture(0)

    recording = False
    gesture_number = 0

    gestures = ['Open hand', 'Closed hand']

    model = load_model()

    with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5, static_image_mode='store_true') as hands:
        while capture.isOpened():
            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            gesture_number = -1  # Reset gesture number if no valid key is pressed
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
            ret, frame = capture.read()
            frame = cv2.flip(frame, 1)

            # Resize the frame to 240x240
            #frame = cv2.resize(frame, (240, 240))

            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Process the image and detect hands
            detected_image = hands.process(image)

            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            

            # Draw hand landmarks and connections if hands are detected
            if detected_image.multi_hand_landmarks:

                absolute_landmarks = normalize_landmarks(image, detected_image.multi_hand_landmarks)

                # Predict gesture 
                predicted_gesture_num = predict_gesture(model, absolute_landmarks)
                predicted_gesture = gestures[predicted_gesture_num]
                print(f"Predicted Gesture: {predicted_gesture}")

                # if closed hand, get positon and direction of centre frame
                pixel_landmarks = get_landmarks_pixels(image, detected_image.multi_hand_landmarks)
                hand_position = get_hand_position(pixel_landmarks)

                cv2.circle(image, hand_position, 5, (0, 255, 0), -1)
                
                centre = get_centre(image)
                cv2.circle(image, centre, 5, (255, 0, 0), -1)

                cv2.line(image, centre, hand_position, (0, 255, 0), 2)
                
                distance_from_centre = get_distance_from_centre(hand_position, image)

                print(f"Distance from center: {distance_from_centre}")

                if (predicted_gesture_num == 1):
                    direction = get_directions(image, hand_position)
                    print(f"Direction to move: {direction}")
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
                record_data(recording, gesture_number, absolute_landmarks)
        
            cv2.imshow('Webcam', image)


            
            

    capture.release()
    cv2.destroyAllWindows()

def get_landmarks_pixels(image, landmarks):
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

def normalize_landmarks(image, landmarks):
    """
    Get relative landmarks
    """
    landmarks_pos_list = []
    base_x, base_y = 0, 0
    if not landmarks:
        return []

    landmarks_pos_list = get_landmarks_pixels(image, landmarks)
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

def record_data(recording, gesture_number, landmarks):
    """
    Record the normalized landmarks to a file.
    """
    csv_file = 'data/landmarks.csv'
    if recording and gesture_number != -1:
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([gesture_number, *landmarks])
    return

def load_model():
    """
    Load the pre-trained model for gesture recognition.
    """
    model = keras.models.load_model('model/model.h5')

    return model

def predict_gesture(model, landmarks):
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
    
def get_hand_position(landmarks):
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

def get_distance_from_centre(hand_position, image):
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

def get_centre(image):
    """
    Get the center of the image.
    """
    height, width, _ = image.shape
    center_x = width // 2
    center_y = height // 2

    return (center_x, center_y)

def get_directions(image, hand_position):
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



if __name__ == "__main__":
    main()