#!/usr/bin/env python3
import rospy
from yolo.msg import yolo
import math


rospy.init_node('yolo_talker', anonymous=True)
pub = rospy.Publisher('/yolo', yolo, queue_size=10)
import cv2
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import struct
import serial
import numpy as np

# from rknnpool import rknnPoolExecutor
#图像处理函数，实际应用过程中需要自行修改
# from func import myFunc
 
#cap = cv2.VideoCapture('./video/islandBenchmark.mp4')



import numpy as np
from rknnlite.api import RKNNLite
 
QUANTIZE_ON = True
 
OBJ_THRESH, NMS_THRESH, IMG_SIZE = 0.70, 0.01, 640
 
CLASSES = ('1')

#elephant 
 
modelPath = "/home/orangepi/fly_ros1_ws/ros1_basic_ws/src/yolo/rknn/best.rknn"
 
# def sigmoid(x):
#     # Sigmoid激活函数，将输入值映射到0-1之间
#     return 1 / (1 + np.exp(-x))
 
 
def xywh2xyxy(x):
    # 将边界框格式从 [中心点x, 中心点y, 宽度, 高度] 转换为 [左上角x, 左上角y, 右下角x, 右下角y]
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y
 
 
# def process(input, mask, anchors):
#     # 处理YOLOv5模型的输出，生成边界框、置信度和类别概率
#     anchors = [anchors[i] for i in mask]
#     grid_h, grid_w = map(int, input.shape[0:2])
 
#     box_confidence = sigmoid(input[..., 4])
#     box_confidence = np.expand_dims(box_confidence, axis=-1)
 
#     box_class_probs = sigmoid(input[..., 5:])
 
#     box_xy = sigmoid(input[..., :2])*2 - 0.5
 
#     col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
#     row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
#     col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
#     row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
#     grid = np.concatenate((col, row), axis=-1)
#     box_xy += grid
#     box_xy *= int(IMG_SIZE/grid_h)
 
#     box_wh = pow(sigmoid(input[..., 2:4])*2, 2)
#     box_wh = box_wh * anchors
 
#     box = np.concatenate((box_xy, box_wh), axis=-1)
 
#     return box, box_confidence, box_class_probs
 
def process(input, mask, anchors):

    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])

    box_confidence = input[..., 4]
    box_confidence = np.expand_dims(box_confidence, axis=-1)

    box_class_probs = input[..., 5:]

    box_xy = input[..., :2]*2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(IMG_SIZE/grid_h)

    box_wh = pow(input[..., 2:4]*2, 2)
    box_wh = box_wh * anchors

    box = np.concatenate((box_xy, box_wh), axis=-1)

    return box, box_confidence, box_class_probs
def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with box threshold. It's a bit different with origin yolov5 post process!
    # Arguments
        boxes: ndarray, boxes of objects.
        box_confidences: ndarray, confidences of objects.
        box_class_probs: ndarray, class_probs of objects.
    # Returns
        boxes: ndarray, filtered boxes.
        classes: ndarray, classes for boxes.
        scores: ndarray, scores for boxes.
    """
    # 根据置信度阈值过滤边界框，只保留置信度高于阈值的检测结果
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])
 
    _box_pos = np.where(box_confidences >= OBJ_THRESH)
    boxes = boxes[_box_pos]
    box_confidences = box_confidences[_box_pos]
    box_class_probs = box_class_probs[_box_pos]
 
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    _class_pos = np.where(class_max_score >= OBJ_THRESH)
 
    boxes = boxes[_class_pos]
    classes = classes[_class_pos]
    scores = (class_max_score * box_confidences)[_class_pos]
 
    return boxes, classes, scores
 
 
def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.
    # Arguments
        boxes: ndarray, boxes of objects.
        scores: ndarray, scores of objects.
    # Returns
        keep: ndarray, index of effective boxes.
    """
    # 非极大值抑制(NMS)，去除重叠的边界框，只保留置信度最高的框
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]
 
    areas = w * h
    order = scores.argsort()[::-1]
 
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
 
        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
 
        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1
 
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep
 
 
def yolov5_post_process(input_data):
    # YOLOv5后处理函数，整合所有处理步骤生成最终的检测结果
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],
               [59, 119], [116, 90], [156, 198], [373, 326]]
 
    boxes, classes, scores = [], [], []
    for input, mask in zip(input_data, masks):
        b, c, s = process(input, mask, anchors)
        b, c, s = filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)
 
    boxes = np.concatenate(boxes)
    boxes = xywh2xyxy(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)
 
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
 
        keep = nms_boxes(b, s)
 
        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])
 
    if not nclasses and not nscores:
        return None, None, None
 
    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)
 
    return boxes, classes, scores
 
def draw(image, boxes, scores, classes):
    """Draw the boxes on the image.

    # Argument:
        image: original image.
        boxes: ndarray, boxes of objects.
        classes: ndarray, classes of objects.
        scores: ndarray, scores of objects.
        all_classes: all classes name.
    """
    # print("{:^12} {:^12}  {}".format('class', 'score', 'xmin, ymin, xmax, ymax'))
    # print('-' * 50)
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = box
        top = int(top)
        left = int(left)
        right = int(right)
        bottom = int(bottom)

        # 计算检测框中心点坐标
        center_x = int((top + right) / 2)
        center_y = int((left + bottom) / 2)
        
        # 创建并发布yolo消息
        yolo_msg = yolo()
        yolo_msg.leixing = str(CLASSES[cl])  # 类别名称
        yolo_msg.x = center_x                 # 中心点x坐标
        yolo_msg.y = center_y                 # 中心点y坐标
        yolo_msg.confidence = float(score)    # 置信度
        pub.publish(yolo_msg)     

        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
        cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                    (top, left - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)

        # print("{:^12} {:^12.3f} [{:>4}, {:>4}, {:>4}, {:>4}]".format(CLASSES[cl], score, top, left, right, bottom))




 
 
def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    # 将图像调整为指定尺寸，保持宽高比，多余部分用指定颜色填充
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
 
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
 
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - \
        new_unpad[1]  # wh padding
 
    dw /= 2  # divide padding into 2 sides
    dh /= 2
 
    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right,
                            cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)
 
def myFunc(rknn_lite, IMG):
    try:
        # 使用RKNN模型进行推理的主要函数
        img = cv2.cvtColor(IMG, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
        # # 添加批次维度，将3D图像扩展为4D输入
        img2 = np.expand_dims(img, 0)
        outputs = rknn_lite.inference(inputs=[img2], data_format=['nhwc'])
        
        # 检查推理输出是否为空
        if outputs is None:
            return IMG
        
        # 增加对输出数据的检查
        if any(output is None for output in outputs[:3]):
            return IMG
        # print("推理输出:", outputs)
        input0_data = outputs[0]
        input1_data = outputs[1]
        input2_data = outputs[2]
        # print("输入数据形状1:", input0_data)
        # print("输入数据形状2:", input1_data)
        # print("输入数据形状3:", input2_data)
        # 增加形状检查
        if input0_data is None or input1_data is None or input2_data is None:
            return IMG

        input0_data = input0_data.reshape([3, -1]+list(input0_data.shape[-2:]))
        input1_data = input1_data.reshape([3, -1]+list(input1_data.shape[-2:]))
        input2_data = input2_data.reshape([3, -1]+list(input2_data.shape[-2:]))
    
        input_data = list()
        input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))
        
        boxes, classes, scores = yolov5_post_process(input_data)
        # print("检测到的边界框:", boxes)
        # print("检测到的类别:", classes)
        # print("检测到的置信度:", scores)
        img_1 = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if boxes is not None:
            draw(img_1, boxes, scores, classes)

            return img_1
        else:
            # 当未检测到目标时也返回处理后的图像
            return img_1
    except Exception as e:
        print(f"Error in myFunc: {e}")
        return IMG
    
    
    
    
from queue import Queue
from rknnlite.api import RKNNLite
from concurrent.futures import ThreadPoolExecutor, as_completed
 
 
def initRKNN(rknnModel= modelPath, id=0):
    try:
        # 初始化单个RKNN模型实例
        rknn_lite = RKNNLite()
        ret = rknn_lite.load_rknn(rknnModel)
        if ret != 0:
            print("Load RKNN rknnModel failed")
            exit(ret)
        if id == 0:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        elif id == 1:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_1)
        elif id == 2:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_2)
        elif id == -1:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        else:
            ret = rknn_lite.init_runtime()
        if ret != 0:
            print("Init runtime environment failed")
            exit(ret)
        print(rknnModel, "\t\tdone")
        return rknn_lite
    except Exception as e:
        print(f"Error initializing RKNN: {e}")
        raise e
 
 
def initRKNNs(rknnModel= modelPath, TPEs=1):
    # 初始化多个RKNN模型实例，用于多线程处理
    rknn_list = []
    for i in range(TPEs):
        rknn_list.append(initRKNN(rknnModel, i % 3))
    return rknn_list
 
 
class rknnPoolExecutor():
    # RKNN模型池执行器，用于管理多个RKNN模型实例的并发执行
    def __init__(self, rknnModel, TPEs, func):
        self.TPEs = TPEs
        self.queue = Queue()
        self.rknnPool = initRKNNs(rknnModel, TPEs)
        self.pool = ThreadPoolExecutor(max_workers=TPEs)
        self.func = func
        self.num = 0
 
    def put(self, frame):
        # 将图像帧提交给线程池进行处理
        self.queue.put(self.pool.submit(
            self.func, self.rknnPool[self.num % self.TPEs], frame))
        self.num += 1
 
    def get(self):
        # 获取处理完成的结果
        if self.queue.empty():
            return None, False
        temp = []
        temp.append(self.queue.get())
        for frame in as_completed(temp):
            return frame.result(), True
 
    def release(self):
        # 释放资源，关闭线程池和RKNN模型实例
        self.pool.shutdown()
        for rknn_lite in self.rknnPool:
            try:
                rknn_lite.release()
            except:
                pass
            
            
# 主程序循环增加异常处理
try:
    cap = cv2.VideoCapture("/dev/video0")
    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头")
        exit(-1)
        
    # 线程数
    TPEs = 6
    # 初始化rknn池
    pool = rknnPoolExecutor(
        rknnModel=modelPath,
        TPEs=TPEs,
        func=myFunc)
     
    # 初始化异步所需要的帧
    if (cap.isOpened()):
        for i in range(TPEs + 1):
            ret, frame = cap.read()
            if not ret:
                print("无法读取视频帧")
                cap.release()
                del pool
                exit(-1)
            pool.put(frame)
     
    frames, loopTime, initTime = 0, time.time(), time.time()
    while (cap.isOpened()):
        frames += 1
        ret, frame = cap.read()
        if not ret:
            print("视频流结束或读取错误")
            break
        pool.put(frame)
        frame, flag = pool.get()
        if flag == False:
            break
        if frame is not None and frame.shape[0] > 0 and frame.shape[1] > 0:
            cv2.imshow('test', frame)
        # cv2.imshow('test', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if frames % 30 == 0:
            print("30帧平均帧率:\t", 30 / (time.time() - loopTime), "帧")
            loopTime = time.time()
     
    print("总平均帧率\t", frames / (time.time() - initTime))
except KeyboardInterrupt:
    print("程序被用户中断")
finally:
    # 确保资源得到释放
    try:
        cap.release()
        cv2.destroyAllWindows()
        pool.release()
    except:
        pass