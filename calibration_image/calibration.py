import cv2
import numpy as np
import json

IMAGE_PATH = "corn.jpg"
OUTPUT_JSON = "colorrange_photo.json"
CAMERA_INDEX = "/dev/v4l/by-id/usb-XZC-260109-A_Streaming_Webcam_Audio_01.00.00-video-index0"


# -----------------------------
# HSV Slider Window
# -----------------------------
def create_hsv_slider_window(window_name, initial_lower, initial_upper):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    cv2.createTrackbar("H_low", window_name, initial_lower[0], 179, lambda x: None)
    cv2.createTrackbar("S_low", window_name, initial_lower[1], 255, lambda x: None)
    cv2.createTrackbar("V_low", window_name, initial_lower[2], 255, lambda x: None)

    cv2.createTrackbar("H_high", window_name, initial_upper[0], 179, lambda x: None)
    cv2.createTrackbar("S_high", window_name, initial_upper[1], 255, lambda x: None)
    cv2.createTrackbar("V_high", window_name, initial_upper[2], 255, lambda x: None)


def read_hsv_sliders(window_name):
    h_low = cv2.getTrackbarPos("H_low", window_name)
    s_low = cv2.getTrackbarPos("S_low", window_name)
    v_low = cv2.getTrackbarPos("V_low", window_name)

    h_high = cv2.getTrackbarPos("H_high", window_name)
    s_high = cv2.getTrackbarPos("S_high", window_name)
    v_high = cv2.getTrackbarPos("V_high", window_name)

    return np.array([h_low, s_low, v_low], dtype=np.uint8), \
           np.array([h_high, s_high, v_high], dtype=np.uint8)


# -----------------------------
# Main
# -----------------------------
def main():
    # Open camera and ask user to capture image or use existing file
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Failed to open camera.")
        return

    image = None
    print("Camera opened. Press 'c' to capture from camera, 'x' to use corn.jpg, or 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from camera.")
            break

        preview = frame.copy()
        cv2.putText(preview, "Press 'c' capture from camera | 'x' use corn.jpg | 'q' quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Camera", preview)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            image = frame.copy()
            print("Captured image from camera.")
            break
        elif key == ord('x'):
            image = cv2.imread(IMAGE_PATH)
            if image is None:
                print(f"Failed to load {IMAGE_PATH}")
                continue
            print(f"Using image file {IMAGE_PATH}.")
            break
        elif key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return

    cap.release()
    cv2.destroyWindow("Camera")

    if image is None:
        return

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Initial ranges
    green_lower = np.array([35, 80, 80], dtype=np.uint8)
    green_upper = np.array([85, 255, 255], dtype=np.uint8)

    yellow_lower = np.array([15, 80, 80], dtype=np.uint8)
    yellow_upper = np.array([35, 255, 255], dtype=np.uint8)

    # Create slider windows
    create_hsv_slider_window("Green HSV", green_lower, green_upper)
    create_hsv_slider_window("Yellow HSV", yellow_lower, yellow_upper)

    print("Adjust sliders. Press 'c' to confirm and export JSON. Press 'q' to quit.")

    while True:
        # Read slider values
        green_lower[:], green_upper[:] = read_hsv_sliders("Green HSV")
        yellow_lower[:], yellow_upper[:] = read_hsv_sliders("Yellow HSV")

        # Generate masks
        green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
        yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)

        # Display everything
        cv2.imshow("Image", image)
        cv2.imshow("Green Mask", green_mask)
        cv2.imshow("Yellow Mask", yellow_mask)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            # Export JSON
            data = {
                "green_lower": green_lower.tolist(),
                "green_upper": green_upper.tolist(),
                "yellow_lower": yellow_lower.tolist(),
                "yellow_upper": yellow_upper.tolist()
            }
            with open(OUTPUT_JSON, "w") as f:
                json.dump(data, f, indent=4)

            print(f"Exported HSV ranges to {OUTPUT_JSON}")
            break

        if key == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()