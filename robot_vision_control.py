import cv2
import numpy as np
from pydexarm1 import Dexarm
import pyrealsense2 as rs

def apply_settings_to_device(device, json_text):
    # Get the active profile and load the JSON file which contains settings readable by the RealSense
    advanced_mode = rs.rs400_advanced_mode(device)
    advanced_mode.load_json(json_text)

def save_bounding_boxes_to_text(bounding_boxes, output_file):
    with open(output_file, 'w') as file:
        for box in bounding_boxes:
            file.write(f"{box[0]},{box[1]},{box[2]},{box[3]}\n")

def capture_screenshot_with_box(image, box, output_filename):
    x, y, w, h = box
    screenshot = image.copy()
    cv2.rectangle(screenshot, (x, y), (x + w, y + h), (0, 255, 0), 3)
    cv2.imwrite(output_filename, screenshot)

# Load the JSON file
json_file_path = './Desktop/Code/highdensity.json'
with open(json_file_path, 'r') as file:
    json_text = file.read().strip()

# Initialize the Dexarm robot
dexarm = Dexarm(port="/dev/ttyACM0")
dexarm.go_home()

# Initialize the RealSense pipeline
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(config)
active_device = profile.get_device()

# Apply the settings from the JSON file to the active device
apply_settings_to_device(active_device, json_text)

# List of arbitrary red color points [(x1, y1, z1), ... (xk, yk, zk)]
red_color_points = [(100, 200, 1000), (300, 400, 1200), (500, 600, 800)]

frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
color_init = np.asanyarray(color_frame.get_data())

font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 500)
fontScale = 1
fontColor = (255, 255, 255)
lineType = 2

def map_camera_to_robot(camera_x, camera_z, depth_mm):
    x_scale = 310 / 640 # Maximum of robo coordinate system/ maximum of camera coordinate system
    z_scale = 180 / 480
    y_scale = 410 / 280

    robot_x = ((camera_x - 320) * x_scale)
    robot_z = ((240 - camera_z) * z_scale) - 100 
    robot_y = depth_mm * y_scale

    return robot_x, robot_y, robot_z

# Flag to check if red color is detected
red_detected = False

# List to store bounding box coordinates [(x1, y1, w1, h1), ... (xk, yk, wk, hk)]
bounding_boxes = []

while True:
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()

    color = np.asanyarray(color_frame.get_data())

    res = color.copy()
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

    l_b = np.array([136, 87, 111], np.uint8)
    u_b = np.array([180, 255, 255], np.uint8)

    mask = cv2.inRange(hsv, l_b, u_b)
    color = cv2.bitwise_and(color, color, mask=mask)

    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)

    aligned_depth_frame = depth_frame
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

    d = cv2.absdiff(color_init, color)
    gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
    (c, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    color_init = color

    depth = np.asanyarray(aligned_depth_frame.get_data())

    for contour in c:
        if cv2.contourArea(contour) > 800:
            continue

        (x, y, w, h) = cv2.boundingRect(contour)
        bottomLeftCornerOfText = (x, y)

        depth_crop = depth[y:y + h, x:x + w].copy()

        if depth_crop.size == 0:
            continue
        depth_res = depth_crop[depth_crop != 0]

        depth_scale = active_device.first_depth_sensor().get_depth_scale()
        depth_res = depth_res * depth_scale

        if depth_res.size == 0:
            continue

        dist = min(depth_res)
        depth_mm = dist * 1000
        print(depth_mm)
        if(depth_mm< 280 and depth_mm>0):
             red_detected=True
        robot_x, robot_y, robot_z = map_camera_to_robot(x, y, depth_mm)

        print(robot_x, robot_y, robot_z)
        cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 3)
        text = "Depth: " + str("{0:.2f}").format(dist)
        cv2.putText(res,
                    text,
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    lineType)
        

        # Move the robot only if red color is detected
        if red_detected:
            dexarm.move_to(robot_x, robot_y, robot_z)
             # Capture live screenshot with the bounding box
            screenshot_frame = res.copy()
            cv2.rectangle(screenshot_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            # Display x, y, w, h values on the captured image
            text = f"x: {x}, y: {y}, w: {w}, h: {h}"
            cv2.putText(screenshot_frame, text, (10, 30), font, fontScale, fontColor, lineType)
            screenshot_filename = f'./screenshot_{len(bounding_boxes) - 1}.png'
            cv2.imwrite(screenshot_filename, screenshot_frame)

            red_detected=False

        # Save bounding box coordinates to the list
        bounding_boxes.append((x, y, w, h))

        

    cv2.namedWindow('RBG', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RBG', res)
    cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Depth', colorized_depth)
    cv2.namedWindow('mask', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('mask', mask)

    # Check if 'q' is pressed to exit the loop
    key = cv2.waitKey(10)
    if key & 0xFF == ord('q'):
        break

# Save bounding box coordinates to a text file
output_file_path = './bounding_boxes.txt'
save_bounding_boxes_to_text(bounding_boxes, output_file_path)


# Release resources and close windows
pipe.stop()
cv2.destroyAllWindows()

# Close the Dexarm robot connection
dexarm.close()
