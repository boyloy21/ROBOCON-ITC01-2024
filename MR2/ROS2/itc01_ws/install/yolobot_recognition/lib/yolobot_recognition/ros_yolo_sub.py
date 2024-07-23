#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolov8_msgs.msg import Yolov8Inference
from yolov8_msgs.msg import InferenceResult
import cv2

import numpy as np
import time
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        weights='/home/cobra/ai_ws/yolov5/runs/train/exp18/weights/best.pt'  # model.pt path(s)
        self.imgsz=(640, 480)  # inference size (pixels)
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        self.classes=None  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False  # class-agnostic NMS
        self.augment=False  # augmented inference
        self.visualize=False  # visualize features
        self.line_thickness=3  # bounding box thickness (pixels)
        self.hide_labels=False  # hide labels
        self.hide_conf=False  # hide confidences
        self.half=False  # use FP16 half-precision inference
        self.stride = 32
        device_num=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.dnn = False
        self.data= 'data/coco128.yaml'  # dataset.yaml path
        self.half=False  # use FP16 half-precision inference
        self.augment=False  # augmented inferenc

        # Initialize
        self.device = select_device(device_num)

        # Load model
        self.model = DetectMultiBackend(weights, device=self.device, dnn=self.dnn, data=self.data, fp16=self.half)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        imgsz = check_img_size(self.imgsz, s=stride)  # check image size

        # Run inference
        bs = 1  # batch_size
        self.model.warmup(imgsz=(1 if pt or self.model.triton else bs, 3, *imgsz))  # warmup

        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 30)
        self.img_pub = self.create_publisher(Image, "/inference_result", 30)

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data)

        # Letterbox
        img0 = img.copy()
        img = img[np.newaxis, :, :, :]        

        # Stack
        img = np.stack(img, 0)

        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.model.device)
        img = img.half() if self.model.fp16 else img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim

        # Inference
        visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if self.visualize else False
        pred = self.model(img, augment=self.augment, visualize=visualize)

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        print(pred)
        # Process detections
        for r in pred:
            self.inference_result = InferenceResult()
            b = r.tolist()
            # print(b)
            self.inference_result.class_name = self.model.names[int(b[5])] # string of the ball
            # print(c)
            self.inference_result.top = int(b[1])
            self.inference_result.left = int(b[0])
            self.inference_result.bottom = int(b[3])
            self.inference_result.right = int(b[2])
            # self.yolov8_inference.append(self.inference_result)
            self.yolov8_inference.yolov8_inference.append(self.inference_result)
            
            print(self.yolov8_inference)
            # self.yolov8_pub.publish(self.yolov8_inference)

        self.yolov8_pub.publish(self.yolov8_inference)
        annotated_frame = results.render()[0]
        cvt_2 = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        img_msg = bridge.cv2_to_imgmsg(cvt_2)


        cv2.imshow("IMAGE", img0)
        cv2.waitKey(4)    

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

