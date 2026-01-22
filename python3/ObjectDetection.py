import jetson.inference
import jetson.utils
import argparse
import cv2
import numpy as np
import pickle

# Parse command line arguments
parser = argparse.ArgumentParser(
    description="Locate objects in a live camera stream using an object detection DNN.",
    formatter_class=argparse.RawTextHelpFormatter,
    epilog=jetson.inference.detectNet.Usage() + jetson.utils.videoSource.Usage() +
            jetson.utils.videoOutput.Usage()
)

parser.add_argument("input", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output", type=str, default="display://0", nargs='?', help="URI of the output stream")
parser.add_argument(
    "--network",
    type=str,
    default="/home/hiru/pytorch-ssd/models/canbopl/ssd-mobilenet.onnx",
    help="pre-trained model to load (see below for options)"
)
parser.add_argument(
    "--labels",
    type=str,
    default="/home/hiru/pytorch-ssd/models/canbopl/labels.txt",
    help="path to labels file"
)
parser.add_argument(
    "--overlay",
    type=str,
    default="labels",
    help="detection overlay flags (e.g. --overlay=box,labels,conf)\n"
         "valid combinations are: 'box', 'labels', 'conf', 'none'"
)
parser.add_argument(
    "--threshold",
    type=float,
    default=0.1,
    help="minimum detection threshold to use"
)

args = parser.parse_args()

# Initialize DetectNet object
net = jetson.inference.detectNet(
    argv=[
        "--model=" + args.network,
        "--labels=" + args.labels,
        "--input-blob=input_0",
        "--output-cvg=scores",
        "--output-bbox=boxes"
    ],
    threshold=args.threshold
)

# Create OpenCV video capture object using GStreamer pipeline
pipeline_front = (
    "v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480, "
    "framerate=30/1 ! videoconvert ! appsink"
)
cap_front = cv2.VideoCapture(pipeline_front, cv2.CAP_GSTREAMER)

# Load the camera calibration data for the front camera
with open('/home/hiru/Desktop/Camera/CameraCalibration/front_camera_calibration.pkl', 'rb') as f:
    camera_matrix_1, dist_coeffs_1 = pickle.load(f)

while True:
    # Capture image
    ret_front, frame_front = cap_front.read()

    if not ret_front:
        print("Cannot read image from the camera.")
        break

    # Convert OpenCV image to CUDA image
    img_front = jetson.utils.cudaFromNumpy(frame_front)

    # Detect objects in the image
    detections_front = net.Detect(img_front)

    if detections_front:
        # Assume the first detection is the object of interest
        detection_front = detections_front[0]

        # Get the center of the bounding box
        center_front = (
            (detection_front.Left + detection_front.Right) / 2,
            (detection_front.Top + detection_front.Bottom) / 2
        )

        # Convert to numpy array
        point_2d_front = np.array(
            [[center_front[0], center_front[1]]],
            dtype=np.float32
        ).reshape(-1, 1)

        # Print 2D coordinates
        print("2D coordinates:", point_2d_front.T)

        # Draw bounding box
        cv2.rectangle(
            frame_front,
            (int(detection_front.Left), int(detection_front.Top)),
            (int(detection_front.Right), int(detection_front.Bottom)),
            (0, 255, 0),
            2
        )

    # Display image
    cv2.imshow("Front Camera", frame_front)

    if cv2.waitKey(1) == ord('q'):
        break

cap_front.release()
cv2.destroyAllWindows()
