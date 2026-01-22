#!/usr/bin/env python3

import sys
import argparse
import time
import cv2
import numpy as np
import jetson.inference
import jetson.utils
from pyorbbecsdk import Config, OBSensorType, Pipeline
import serial

ESC_KEY = 27
MIN_DEPTH = 20     # 20mm를 미터로 설정
MAX_DEPTH = 10000  # 10000mm를 미터로 설정


class TemporalFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.previous_frame = None

    def process(self, frame):
        if self.previous_frame is None:
            result = frame
        else:
            result = cv2.addWeighted(frame, self.alpha, self.previous_frame, 1 - self.alpha, 0)
        self.previous_frame = result
        return result


def map_value(center_x, old_min=150, old_max=450, new_min=-9, new_max=9):
    a = (new_max - new_min) / (old_max - old_min)
    b = new_min - a * old_min
    new_x = a * center_x + b
    return new_x


# Serial ports
ser = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=None)
ser1 = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=None)


def send_command(command):
    command_str = " ".join(map(str, command)) + "\n"
    ser1.write(command_str.encode())
    print(f"Sent command: {command_str}")


def main():
    parser = argparse.ArgumentParser(
        description="Locate objects in a live camera stream using an object detection DNN.",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=jetson.inference.detectNet.Usage() + jetson.utils.videoSource.Usage() +
               jetson.utils.videoOutput.Usage()
    )

    parser.add_argument("input", type=str, default="", nargs='?', help="URI of the input stream for depth camera")
    parser.add_argument("output", type=str, default="display://0", nargs='?', help="URI of the output stream for object detection")
    parser.add_argument("--network", type=str,
                        default="/home/hiru/pytorch-ssd/models/canbopl/ssd-mobilenet.onnx",
                        help="pre-trained model to load")
    parser.add_argument("--labels", type=str,
                        default="/home/hiru/pytorch-ssd/models/canbopl/labels.txt",
                        help="path to labels file")
    parser.add_argument("--overlay", type=str, default="labels",
                        help="detection overlay flags")
    parser.add_argument("--threshold", type=float, default=0.1,
                        help="minimum detection threshold to use")

    args = parser.parse_known_args()[0]

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

    # Orbbec Depth Camera setup
    config = Config()
    pipeline = Pipeline()
    temporal_filter = TemporalFilter(alpha=0.5)

    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        assert profile_list is not None
        depth_profile = profile_list.get_default_video_stream_profile()
        assert depth_profile is not None
        config.enable_stream(depth_profile)
    except Exception as e:
        print(e)
        return

    pipeline.start()

    input_video = jetson.utils.videoSource("/dev/video0", argv=sys.argv + ["--width=640", "--height=480"])
    output_video = jetson.utils.videoOutput(args.output, argv=sys.argv + ["--width=640", "--height=480"])

    last_object_center_y = 0
    last_object_center_x = 0
    detection_count = 0  # 탐지 횟수

    while True:
        frames = pipeline.wait_for_frames(100)
        if frames is None:
            continue

        depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            continue

        width = depth_frame.get_width()
        height = depth_frame.get_height()
        scale = depth_frame.get_depth_scale()

        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
        depth_data = depth_data.reshape((height, width)).astype(np.float32) * scale
        depth_data = np.where((depth_data > MIN_DEPTH) & (depth_data < MAX_DEPTH), depth_data, 0)

        img = input_video.Capture()
        if img is None:
            continue

        img_np = jetson.utils.cudaToNumpy(img)
        detections = net.Detect(img, overlay=args.overlay)

        if detections:
            detection_count += 1
            print(f"Detections count: {detection_count}")

            for detection in detections:
                left = int(detection.Left)
                top = int(detection.Top)
                right = int(detection.Right)
                bottom = int(detection.Bottom)

                cv2.rectangle(img_np, (left, top), (right, bottom), (0, 255, 0), 2)

                center_x = (left + right) // 2
                center_y = (top + bottom) // 2

                cv2.circle(img_np, (center_x, center_y), 5, (0, 0, 255), -1)

                last_object_center_x = center_x
                last_object_center_y = center_y
                last_object_label = net.GetClassDesc(detection.ClassID)

                # 좌표 출력
                print(f"Current Center Coordinates: ({center_x}, {center_y})")

                # 거리 측정
                if 0 <= center_x < width and 0 <= center_y < height:
                    depth_data = temporal_filter.process(depth_data)

                    center_y = int(height / 2)
                    center_x = int(width / 2)

                    center_distance = depth_data[center_y, center_x]
                    mapped_center_x = map_value(center_x)

                    print(
                        f"Object: {last_object_label}, Center: ({mapped_center_x}, {last_object_center_y}), "
                        f"Distance: {center_distance:.2f} mm"
                    )

                    # 필요한 명령어 전송
                    if detection_count >= 150:
                        send_command(['s'])
                        time.sleep(5)

                        # 'yes' 또는 'move' 명령 처리
                        while True:
                            if ser.in_waiting > 0:
                                received_data = ser.readline().decode().strip()

                                if received_data == "yes":
                                    print("Received 'yes' command.")
                                    time.sleep(3)

                                    if last_object_label == "plastic":
                                        ser.write("pet\n".encode())
                                        print("Sent label: pet")
                                    elif last_object_label in ["can", "bottle"]:
                                        ser.write(f"{last_object_label}\n".encode())
                                        print(f"Sent label: {last_object_label}")
                                    break

                                elif received_data == "move":
                                    print("Received 'move' command.")

                                    ser.write(f"{center_distance:.2f}\n".encode())
                                    time.sleep(0.1)

                                    ser.write(f"{mapped_center_x:.2f}\n".encode())
                                    time.sleep(1)

                                    print(f"Sent mapped data: {center_distance:.2f}, {mapped_center_x:.2f}")
                                    break

                        detection_count = 0

        img_cuda = jetson.utils.cudaFromNumpy(img_np)
        output_video.Render(img_cuda)
        output_video.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

        depth_image = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
        cv2.imshow("Depth Viewer", depth_image)

        key = cv2.waitKey(1)
        if key == ord('q') or key == ESC_KEY:
            break

    pipeline.stop()
    ser1.close()
    ser.close()


if __name__ == "__main__":
    main()
