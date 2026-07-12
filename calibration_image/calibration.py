import cv2
import numpy as np
import json

IMAGE_PATH = "/home/corn/ros2_mr26/src/calibration_image/corn.jpg"
OUTPUT_JSON = "/home/corn/ros2_mr26/src/calibration_image/colorrange_photo.json"


def create_hsv_slider_window(window_name, initial_lower, initial_upper):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    cv2.createTrackbar("H_low", window_name, initial_lower[0], 179, lambda x: None)
    cv2.createTrackbar("S_low", window_name, initial_lower[1], 255, lambda x: None)
    cv2.createTrackbar("V_low", window_name, initial_lower[2], 255, lambda x: None)

    cv2.createTrackbar("H_high", window_name, initial_upper[0], 179, lambda x: None)
    cv2.createTrackbar("S_high", window_name, initial_upper[1], 255, lambda x: None)
    cv2.createTrackbar("V_high", window_name, initial_upper[2], 255, lambda x: None)


def read_hsv_sliders(window_name):
    lower = np.array([
        cv2.getTrackbarPos("H_low", window_name),
        cv2.getTrackbarPos("S_low", window_name),
        cv2.getTrackbarPos("V_low", window_name)
    ], dtype=np.uint8)

    upper = np.array([
        cv2.getTrackbarPos("H_high", window_name),
        cv2.getTrackbarPos("S_high", window_name),
        cv2.getTrackbarPos("V_high", window_name)
    ], dtype=np.uint8)

    return lower, upper


def main():

    image = cv2.imread(IMAGE_PATH)

    if image is None:
        print(f"Failed to load: {IMAGE_PATH}")
        return

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    green_lower = np.array([35, 80, 80], dtype=np.uint8)
    green_upper = np.array([85, 255, 255], dtype=np.uint8)

    yellow_lower = np.array([15, 100, 80], dtype=np.uint8)
    yellow_upper = np.array([35, 255, 255], dtype=np.uint8)

    create_hsv_slider_window(
        "Green HSV",
        green_lower,
        green_upper
    )

    create_hsv_slider_window(
        "Yellow HSV",
        yellow_lower,
        yellow_upper
    )

    print("\nControls:")
    print("Adjust sliders")
    print("Press 'c' to save JSON")
    print("Press 'q' to quit\n")

    while True:

        green_lower, green_upper = read_hsv_sliders("Green HSV")
        yellow_lower, yellow_upper = read_hsv_sliders("Yellow HSV")

        green_mask = cv2.inRange(
            hsv_image,
            green_lower,
            green_upper
        )

        yellow_mask = cv2.inRange(
            hsv_image,
            yellow_lower,
            yellow_upper
        )

        green_preview = cv2.bitwise_and(
            image,
            image,
            mask=green_mask
        )

        yellow_preview = cv2.bitwise_and(
            image,
            image,
            mask=yellow_mask
        )

        # cv2.imshow("Original Image", image)
        cv2.imshow("Green Mask", green_mask)
        cv2.imshow("Yellow Mask", yellow_mask)

        # cv2.imshow(
        #     "Green Detection Preview",
        #     green_preview
        # )

        # cv2.imshow(
        #     "Yellow Detection Preview",
        #     yellow_preview
        # )

        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):

            data = {
                "yellow_lower": yellow_lower.tolist(),
                "yellow_upper": yellow_upper.tolist(),
                "green_lower": green_lower.tolist(),
                "green_upper": green_upper.tolist()
            }

            with open(OUTPUT_JSON, "w") as f:
                json.dump(data, f, indent=4)

            print(f"Saved: {OUTPUT_JSON}")
            break

        elif key == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()