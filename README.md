# Dexarm-RealSense-Robot-Vision
Controlling a robotic arm based on visual input from a RealSense camera.

## Overview

The **Robot Vision Control** script is a Python program designed for controlling a robotic arm using computer vision with the integration of the Dexarm robot and the RealSense camera. The script captures real-time visual data, processes it, and makes decisions based on detected objects, specifically focusing on the recognition of red-colored objects.

## Features

- **RealSense Integration:** Utilizes the RealSense camera to capture both color and depth data for enhanced visual perception.
  
- **Dexarm Robot Control:** Controls the Dexarm robot based on the visual input, allowing the robot to respond to detected objects.

- **Object Detection:** Identifies and tracks red-colored objects within the camera's field of view.

- **Bounding Box Visualization:** Displays bounding boxes around detected objects on the live camera feed.

- **Depth Sensing:** Utilizes depth information to calculate the distance to detected objects.

- **Data Logging:** Records bounding box coordinates and depth information to a text file for further analysis.

## Prerequisites

Before running the script, make sure you have the following dependencies installed:

- Python 3.x
- OpenCV
- NumPy
- PyDexarm
- PyRealSense2

## Usage

1. Ensure all dependencies are installed by running:
   ```
   pip install opencv-python numpy pydexarm pyrealsense2
   ```

2. Connect the Dexarm robot to the specified port (`/dev/ttyACM0` in the example).

3. Load the JSON file (`highdensity.json`) containing camera settings.

4. Run the script:
   ```
   python robot_vision_control.py
   ```

5. Press 'q' to exit the program.
## Input/Output

  Input:
      The script requires the Dexarm robot to be connected to the specified port (/dev/ttyACM0 in the example).
      The script loads camera settings from a JSON file (highdensity.json).

  Output:
      The script outputs real-time visualizations of the camera feed with bounding boxes around detected objects.
      Bounding box coordinates and depth information are saved to a text file (bounding_boxes.txt).

## Configuration

Adjust the script parameters as needed, such as file paths, color thresholds, or robot movement scales, to suit your specific setup and requirements.


## Acknowledgments

- [OpenCV](https://opencv.org/) for computer vision functionality.
- [Dexarm](https://www.dexarm.com/) for the Dexarm robot control library.
- [Intel RealSense](https://www.intelrealsense.com/) for the RealSense camera integration.

Feel free to contribute, report issues, or make suggestions!


