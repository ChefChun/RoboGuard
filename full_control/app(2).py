from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import cv2
# 设置 OpenCV 使用单线程模式，避免多线程解码问题
cv2.setNumThreads(1)
# 禁用 OpenCV 优化
cv2.ocl.setUseOpenCL(False)

import numpy as np
import base64
import json
import threading
import time
import os
import logging
import gc
import asyncio
from datetime import datetime
from ultralytics import YOLO
import torch
import uvicorn
import socketio
from typing import Dict, List, Optional, Any, Tuple
import math
import collections
import sys
import paho.mqtt.client as mqtt
from paho.mqtt import publish # <-- 确保导入了 publish 模块
import socket
import struct
import queue
from torchreid import utils as treid_utils
from PIL import Image
import clip
from decision_maker import DecisionMaker
from robot_tracker import RobotTracker

from config import (
    TEST_MODE,
    CAMERA_MODE,
    TEST_VIDEO_PATH,
    LOG_LEVEL,
    TEST_ROBOT_CONFIG,
    API_CALL, # 这个在当前逻辑中可能不再需要，但保留以防万一
    CLIP_SIMILARITY_THRESHOLD
)

# 配置日志
current_dir = os.path.dirname(os.path.abspath(__file__))
log_file_path = os.path.join(current_dir, 'app.log')
log_dir = os.path.dirname(log_file_path)
os.makedirs(log_dir, exist_ok=True)  # 确保日志目录存在

logging.basicConfig(
    level=LOG_LEVEL,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file_path),
        logging.StreamHandler()  # 同时输出到控制台
    ]
)
logger = logging.getLogger(__name__)

# MQTT配置
MQTT_BROKER = "192.168.137.38"
MQTT_PORT = 1883
MQTT_CONTROL_TOPIC = "car/control"
MQTT_STATUS_TOPIC = "car/status"
MQTT_USER = "pi"
MQTT_PASS = "yjh123"
MQTT_AUTH = {'username': MQTT_USER, 'password': MQTT_PASS}

# 初始化MQTT客户端
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
try:
    # 修复：只启动循环，连接在on_connect回调中处理或手动调用
    mqtt_client.loop_start()
    logger.info("MQTT client loop started.")
except Exception as e:
    logger.error(f"Failed to start MQTT client loop: {e}")
    mqtt_client = None  # 如果失败，设置为None

# 创建必要的目录
os.makedirs(os.path.join(current_dir, 'ai_models'), exist_ok=True)
os.makedirs(os.path.join(current_dir, 'logs'), exist_ok=True)

# 初始化队列和锁
frame_queue = queue.Queue(maxsize=10) # 用于树莓派传来的帧
frame_lock = threading.Lock()
feature_lock = threading.Lock()
latest_frame = None # 用于存储SocketStreamServer接收的最新帧

# 发送小车控制命令
def send_car_command(command, speed=None):
    if mqtt_client is None:
        logger.warning("MQTT client is not initialized, cannot send command.")
        return

    data = {"command": command}
    if speed is not None:
        data["speed"] = speed

    try:
        mqtt_client.publish(
            MQTT_CONTROL_TOPIC,
            payload=json.dumps(data)
        )
        logger.info(f"Sent MQTT command: {data}")
    except Exception as e:
        logger.error(f"Failed to send MQTT command: {e}")


# 订阅小车状态并推送到前端
def on_status(client, userdata, msg):
    status = json.loads(msg.payload.decode())
    # 假设你用Socket.IO推送到前端
    try:
        # 使用 run_coroutine_threadsafe 在事件循环中安全地emit
        asyncio.run_coroutine_threadsafe(sio.emit('car_status', status), sio.get_loop())
    except Exception as e:
        logger.error(f"推送小车状态到前端失败: {e}")

if mqtt_client:
    try:
        mqtt_client.subscribe(MQTT_STATUS_TOPIC)
        mqtt_client.on_message = on_status
        logger.info(f"Subscribed to MQTT topic: {MQTT_STATUS_TOPIC}")
        # 在这里连接MQTT Broker
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        logger.info(f"MQTT client connected to {MQTT_BROKER}:{MQTT_PORT}")
    except Exception as e:
        logger.error(f"Failed to connect or subscribe MQTT client: {e}")
        mqtt_client = None


# 初始化TorchReID特征提取器
extractor = treid_utils.FeatureExtractor(
    model_name='osnet_x1_0',
    model_path='',  # 使用预训练模型
    device='cuda' if torch.cuda.is_available() else 'cpu'
)

# 人物特征库
person_features = {}  # 格式: {person_id: features}

class SocketStreamServer:
    def __init__(self, host, port):
        self.server_socket = socket.socket()
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # 允许地址复用
        self.server_socket.bind((host, port))
        self.server_socket.listen(0)
        self.running = True
        self.target_fps = 10  # 目标帧率 (树莓派可能更高，但处理速度有限)
        self.last_frame_time = time.time()
        self.frame_interval = 1.0 / self.target_fps
        # 树莓派传输过来的原始分辨率，用于显示到本地窗口，这里假设为640x480
        self.rpi_stream_res = (640, 480)
        self.jpeg_quality = 80  # JPEG压缩质量
        self.connection = None # 初始化连接为None
        self.client_address = None

    async def start(self):
        self.server_socket.setblocking(False) # For asyncio.sock_accept
        logger.info("Socket服务器已启动，等待树莓派连接...")

        # 启动接收线程
        threading.Thread(target=self.receive_stream, daemon=True).start()

        # 在后台异步接受连接
        asyncio.create_task(self.accept_connection())

    def receive_stream(self):
        global latest_frame
        stream_bytes = b''
        image = None # 每次循环开始时重置 image

        while self.running:
            try:
                # 控制帧率
                current_time = time.time()
                elapsed = current_time - self.last_frame_time
                if elapsed < self.frame_interval:
                    time.sleep(max(0, self.frame_interval - elapsed))

                len_buf = None
                if not self.connection: # 没有连接，等待
                    time.sleep(0.1)
                    continue

                try:
                    len_buf = self.connection.read(4)
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    logger.error(f"Socket connection lost while reading length: {e}. Attempting to reconnect.")
                    try:
                        self.connection.close()
                        self.connection = None
                    except: pass
                    time.sleep(0.1) # 短暂等待后重试接受连接
                    continue # 跳过当前循环，等待新连接

                if not len_buf: # 连接可能刚刚断开或读取到空数据
                    time.sleep(0.01)
                    continue

                try:
                    image_len = struct.unpack('<L', len_buf)[0]
                    if image_len == 0:
                        time.sleep(0.01)
                        continue

                    # 读取图像数据
                    image_data = b''
                    while len(image_data) < image_len:
                        remaining = image_len - len(image_data)
                        chunk = self.connection.read(remaining)
                        if not chunk: # 连接中断或数据不完整
                            logger.error("Socket connection unexpectedly closed while reading image data or received incomplete data.")
                            try:
                                self.connection.close()
                                self.connection = None
                            except: pass
                            image = None # 标记为无效图像
                            break # 退出内部循环
                        image_data += chunk

                    if image_data: # 确保读取到了数据才尝试解码
                        image = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if image is None:
                            logger.warning("Failed to decode image from Raspberry Pi stream.")
                            time.sleep(0.01)
                            continue
                    else: # 没有读取到数据
                        image = None
                        time.sleep(0.01)
                        continue
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    logger.error(f"解码图像或连接错误: {e}")
                    try:
                        self.connection.close()
                        self.connection = None
                    except: pass
                    time.sleep(0.1)
                    continue
                except Exception as e: # 捕获其他异常
                    logger.error(f"处理图像数据流错误: {e}", exc_info=True)
                    time.sleep(0.1)
                    continue

                if image is not None:
                    try:
                        # 降采样到目标分辨率（这里是SocketStreamServer接收后进行的处理，保持稳定尺寸）
                        image = cv2.resize(image, self.rpi_stream_res, interpolation=cv2.INTER_AREA)

                        # 压缩图像 (可选，如果树莓派已经压缩则不需要，但可以确保尺寸稳定)
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                        _, compressed_image = cv2.imencode('.jpg', image, encode_param)
                        image = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)

                        # 更新最新帧
                        with frame_lock:
                            if image is not None:
                                latest_frame = image.copy()  # 创建副本

                        # 将图像放入处理队列（如果队列为空时才添加新帧）
                        if frame_queue.empty() and image is not None:
                            frame_queue.put(image.copy())  # 放入副本
                            self.last_frame_time = current_time
                    except Exception as e:
                        logger.error(f"处理图像错误: {e}", exc_info=True)
                        continue
                else: # 如果image为None，短暂等待
                    time.sleep(0.01)

            except Exception as e: # 捕获主循环中的所有其他未预期错误
                logger.error(f"接收图像流主循环错误: {e}", exc_info=True)
                time.sleep(0.1)
                continue

        # 清理资源
        if self.connection:
            try:
                self.connection.close()
            except: pass
        try:
            self.server_socket.close()
        except: pass
        logger.info("图像接收服务器已关闭")

    async def accept_connection(self):
        """异步接受连接"""
        while self.running:
            if self.connection: # 如果已经有连接，则不再接受新连接
                await asyncio.sleep(1) # 短暂等待，避免CPU空转
                continue
            try:
                # 尝试接受连接
                client_socket, client_address = await asyncio.get_event_loop().sock_accept(self.server_socket)
                client_socket.setblocking(True) # 设为阻塞模式，因为makefile需要阻塞操作
                self.connection = client_socket.makefile('rb')
                self.client_address = client_address
                logger.info(f"树莓派已连接: {self.client_address}")
            except (BlockingIOError, asyncio.CancelledError):
                # 如果没有连接请求，等待一段时间再试
                await asyncio.sleep(1)
            except Exception as e:
                logger.error(f"接受连接错误: {e}", exc_info=True)
                await asyncio.sleep(5)  # 出错后等待较长时间

# 创建Socket服务器实例
socket_server = SocketStreamServer('0.0.0.0', 8002)

# 获取当前文件的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))

# 添加mmskeleton路径到系统路径
mmskeleton_dir = os.path.join(os.path.dirname(current_dir), 'mmskeleton')
sys.path.append(mmskeleton_dir)

# 导入mmskeleton相关模块
from mmskeleton.models.backbones.st_gcn_aaai18 import ST_GCN_18
from mmskeleton.ops.st_gcn import Graph

# 创建Socket.IO服务器
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')

# 初始化FastAPI应用
fastapi_app = FastAPI(title="Security Robot Control Interface")
fastapi_app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 设置静态文件和模板
fastapi_app.mount("/static", StaticFiles(directory=os.path.join(current_dir, "static")), name="static")
templates = Jinja2Templates(directory=os.path.join(current_dir, "templates"))

# 将Socket.IO应用与FastAPI应用集成
app = socketio.ASGIApp(sio, fastapi_app)

# 加载YOLO模型（用于人体检测和跟踪）
try:
    model_path = os.path.join(current_dir, 'ai_models', 'yolov8n.pt')
    pose_model_path = os.path.join(current_dir, 'ai_models', 'yolov8n-pose.pt')
    obb_model_path = os.path.join(current_dir, 'ai_models', 'yolov11n-obb.pt') # 假设yolov8n-obb
    
    # 加载普通检测模型
    if not os.path.exists(model_path):
        os.makedirs(os.path.dirname(model_path), exist_ok=True)
        logger.info(f"Model not found at {model_path}, downloading default YOLOv8n.pt")
        model = YOLO('yolov8n.pt')  # 使用默认模型
        model.save(model_path)
    else:
        model = YOLO(model_path)
    
    # 加载YOLOPose模型
    if not os.path.exists(pose_model_path):
        logger.info(f"Pose model not found at {pose_model_path}, downloading default YOLOPose.pt")
        pose_model = YOLO('yolov8n-pose.pt')  # 使用默认姿态模型
        pose_model.save(pose_model_path)
    else:
        pose_model = YOLO(pose_model_path)
    
    # 加载 YOLO11n-OBB 模型（用于带方向的车辆检测）
    if not os.path.exists(obb_model_path):
        logger.info(f"OBB model not found at {obb_model_path}, downloading default YOLOv8n-OBB.pt") # 修正为 YOLOv8n-OBB
        obb_model = YOLO('yolov8n-obb.pt')  # 使用默认OBB模型
        obb_model.save(obb_model_path)
    else:
        obb_model = YOLO(obb_model_path)

    # 设置模型使用CUDA
    if torch.cuda.is_available():
        logger.info("CUDA is available. Using GPU for inference.")
        model.to('cuda')
        pose_model.to('cuda')
        obb_model.to('cuda')
    else:
        logger.info("CUDA not available. Using CPU for inference.")
    
    logger.info("YOLO, YOLOPose and YOLO11n-OBB models loaded successfully")
except Exception as e:
    logger.error(f"Error loading YOLO models: {e}", exc_info=True)
    model = None
    pose_model = None
    obb_model = None

# 添加角度离散化函数
def discretize_angle(angle):
    """
    将角度离散化为八个方向，返回0-7的数字
    0: N, 1: NE, 2: E, 3: SE, 4: S, 5: SW, 6: W, 7: NW
    """
    angle = (angle + 360) % 360
    index = int(((angle + 22.5) % 360) // 45)
    return index

# 加载ST-GCN模型（用于动作识别）
try:
    # 定义ST-GCN模型配置 - 使用Kinetics数据集配置
    stgcn_model_cfg = {
        'in_channels': 3,  # 输入通道：x, y, confidence
        'num_class': 400,  # Kinetics数据集有400个动作类别
        'edge_importance_weighting': True,
        'graph_cfg': {
            'layout': 'openpose',  # 使用OpenPose格式的关节点
            'strategy': 'spatial'
        }
    }
    
    # 加载Kinetics数据集的动作类别
    kinetics_label_path = os.path.join(mmskeleton_dir, 'resource', 'kinetics_skeleton', 'label_map.txt')
    action_classes = []
    if os.path.exists(kinetics_label_path):
        with open(kinetics_label_path, 'r') as f:
            action_classes = [line.strip() for line in f.readlines()]
        logger.info(f"加载了 {len(action_classes)} 个Kinetics动作类别")
    else:
        logger.error(f"Kinetics标签文件不存在: {kinetics_label_path}")
        stgcn_model = None
        action_classes = []
    
    if action_classes:
        # 创建ST-GCN模型
        stgcn_model = ST_GCN_18(**stgcn_model_cfg)
        
        # 加载预训练权重 - 使用Kinetics预训练模型
        checkpoint_path = os.path.join(mmskeleton_dir, 'checkpoints', 'st_gcn.kinetics-6fa43f73.pth')
        if os.path.exists(checkpoint_path):
            # 直接使用torch加载模型权重
            state_dict = torch.load(checkpoint_path, map_location='cpu')
            stgcn_model.load_state_dict(state_dict)
            logger.info(f"ST-GCN Kinetics模型权重加载成功: {checkpoint_path}")
        else:
            logger.error(f"ST-GCN Kinetics模型权重文件不存在: {checkpoint_path}")
            stgcn_model = None
        
        # 将模型移至GPU（如果可用）
        if stgcn_model is not None and torch.cuda.is_available():
            stgcn_model = stgcn_model.cuda()
            stgcn_model.eval()
            logger.info("ST-GCN模型已移至GPU并设置为评估模式")
    
except Exception as e:
    logger.error(f"加载ST-GCN模型失败: {e}", exc_info=True)
    stgcn_model = None
    action_classes = []

# 创建决策器实例
decision_maker = DecisionMaker(stgcn_model=stgcn_model, action_classes=action_classes)

# 生成OpenPose格式的关键点
def generate_keypoints(x1, y1, x2, y2, center_x, center_y):
    """
    生成OpenPose格式的18个关键点
    关键点顺序：0-nose, 1-neck, 2-right_shoulder, 3-right_elbow, 4-right_wrist, 5-left_shoulder, 
               6-left_elbow, 7-left_wrist, 8-right_hip, 9-right_knee, 10-right_ankle, 11-left_hip,
               12-left_knee, 13-left_ankle, 14-right_eye, 15-left_eye, 16-right_ear, 17-left_ear
    """
    # 计算人体各部位的估计位置
    head_x, head_y = center_x, y1 + (y2 - y1) * 0.15  # 头部/鼻子
    neck_y = y1 + (y2 - y1) * 0.25  # 颈部
    left_shoulder_x, left_shoulder_y = center_x - (x2 - x1) * 0.2, y1 + (y2 - y1) * 0.3  # 左肩
    right_shoulder_x, right_shoulder_y = center_x + (x2 - x1) * 0.2, y1 + (y2 - y1) * 0.3  # 右肩
    left_elbow_x, left_elbow_y = center_x - (x2 - x1) * 0.3, y1 + (y2 - y1) * 0.5  # 左肘
    right_elbow_x, right_elbow_y = center_x + (x2 - x1) * 0.3, y1 + (y2 - y1) * 0.5  # 右肘
    left_wrist_x, left_wrist_y = center_x - (x2 - x1) * 0.35, y1 + (y2 - y1) * 0.65  # 左腕
    right_wrist_x, right_wrist_y = center_x + (x2 - x1) * 0.35, y1 + (y2 - y1) * 0.65  # 右腕
    left_hip_x, left_hip_y = center_x - (x2 - x1) * 0.2, y1 + (y2 - y1) * 0.6  # 左臀
    right_hip_x, right_hip_y = center_x + (x2 - x1) * 0.2, y1 + (y2 - y1) * 0.6  # 右臀
    left_knee_x, left_knee_y = center_x - (x2 - x1) * 0.2, y1 + (y2 - y1) * 0.75  # 左膝
    right_knee_x, right_knee_y = center_x + (x2 - x1) * 0.2, y1 + (y2 - y1) * 0.75  # 右膝
    left_ankle_x, left_ankle_y = center_x - (x2 - x1) * 0.2, y1 + (y2 - y1) * 0.9  # 左踝
    right_ankle_x, right_ankle_y = center_x + (x2 - x1) * 0.2, y1 + (y2 - y1) * 0.9  # 右踝
    left_eye_x, left_eye_y = center_x - (x2 - x1) * 0.05, y1 + (y2 - y1) * 0.12  # 左眼
    right_eye_x, right_eye_y = center_x + (x2 - x1) * 0.05, y1 + (y2 - y1) * 0.12  # 右眼
    left_ear_x, left_ear_y = center_x - (x2 - x1) * 0.1, y1 + (y2 - y1) * 0.15  # 左耳
    right_ear_x, right_ear_y = center_x + (x2 - x1) * 0.1, y1 + (y2 - y1) * 0.15  # 右耳
    
    # 生成18个关键点（OpenPose格式）
    keypoints = [
        [head_x, head_y, 0.9],                  # 0-nose
        [center_x, neck_y, 0.9],                # 1-neck
        [right_shoulder_x, right_shoulder_y, 0.9], # 2-right_shoulder
        [right_elbow_x, right_elbow_y, 0.8],    # 3-right_elbow
        [right_wrist_x, right_wrist_y, 0.7],    # 4-right_wrist
        [left_shoulder_x, left_shoulder_y, 0.9], # 5-left_shoulder
        [left_elbow_x, left_elbow_y, 0.8],      # 6-left_elbow
        [left_wrist_x, left_wrist_y, 0.7],      # 7-left_wrist
        [right_hip_x, right_hip_y, 0.9],        # 8-right_hip
        [right_knee_x, right_knee_y, 0.8],      # 9-right_knee
        [right_ankle_x, right_ankle_y, 0.7],    # 10-right_ankle
        [left_hip_x, left_hip_y, 0.9],          # 11-left_hip
        [left_knee_x, left_knee_y, 0.8],        # 12-left_knee
        [left_ankle_x, left_ankle_y, 0.7],      # 13-left_ankle
        [right_eye_x, right_eye_y, 0.8],        # 14-right_eye
        [left_eye_x, left_eye_y, 0.8],          # 15-left_eye
        [right_ear_x, right_ear_y, 0.7],        # 16-right_ear
        [left_ear_x, left_ear_y, 0.7]           # 17-left_ear
    ]
    
    return keypoints

# 机器人状态
robot_status = {
    'working_status': 'Idle',
    'battery_level': 85,
    'signal_strength': 92,
    'control_mode': 'auto',
    'camera_enabled': True,
    'position': {
        'x': 0,
        'y': 0,
        'orientation': 0  # 角度，0-359
    },
    'target_person': None,  # 目标人物ID
    'enforcement_status': 'standby',  # 执法状态：standby, approaching, enforcing
    'last_updated': time.time()
}

# 与树莓派的WebSocket连接 (目前未使用，通过SocketStreamServer和MQTT通信)
raspberry_pi_ws = None
last_detection_time = None
frame_counter = 0 # 用于GC计数
last_gc_time = time.time()
# 网页前端视频流的帧率控制
web_stream_last_frame_time = time.time()
web_stream_target_fps = 30
web_stream_frame_interval = 1.0 / web_stream_target_fps

# 人体检测结果存储
detected_persons = {}  # 格式: {person_id: {info...}}
detection_counts = {}  # 格式: {track_id: count} 用于记录连续检测次数
confirmed_tracks = set()  # 已确认的跟踪ID集合
position_history = {}  # 格式: {track_id: deque([(time, x, y), ...], maxlen=10)} 用于速度计算

# 人体关键点序列存储
keypoints_sequences = {}  # 格式: {person_id: deque([frame1_keypoints, frame2_keypoints, ...], maxlen=10)}

# 添加检测结果缓存和时间控制变量
# 这些缓存用于 process_frame，不管哪个源的帧都用统一的检测结果
last_detection_run_time = time.time()
detection_result_cache = None
detection_result_timestamp = 0
detection_interval = 1.0  # 每1秒运行一次检测 (针对模型推理)
result_display_time = 0.3  # 检测结果显示0.5秒 (指缓存结果的有效期)

# 添加全局变量用于锁定目标人物
locked_target_id = None

# 使用ST-GCN模型进行动作预测
def predict_action_with_stgcn(keypoints_sequence):
    """
    使用ST-GCN模型预测动作
    keypoints_sequence: 一个人物的关键点序列，形状为 [frames, joints, 3]
    返回: (预测的动作类别, 置信度)
    """
    if stgcn_model is None:
        return "unknown", 0.5 # 修正：如果模型未加载，直接返回未知
    
    try:
        if not keypoints_sequence: # 确保序列不为空
            return "unknown", 0.5
        
        # 准备输入数据
        # ST-GCN期望的输入形状为 [N, C, T, V, M]
        # N: batch size, C: channels (x,y,conf), T: frames, V: joints, M: number of people
        # Kinetics模型使用OpenPose格式的18个关节点
        # 确保 T (frames) 维度是 10，如果不足，则进行填充
        max_frames = 10
        current_seq_len = len(keypoints_sequence)
        
        data = np.zeros((1, 3, max_frames, 18, 1), dtype=np.float32)

        for i, frame_kps in enumerate(list(keypoints_sequence)[-max_frames:]): # 取最近的max_frames帧
            if frame_kps is not None:
                num_joints = min(len(frame_kps), 18)
                for j in range(num_joints):
                    if frame_kps[j] is not None and len(frame_kps[j]) >= 3:
                        x, y, conf = frame_kps[j][0], frame_kps[j][1], frame_kps[j][2]
                        # 坐标归一化到[-0.5, 0.5]范围
                        # 假设图像大小为1280x960 (process_frame 内部统一的推理尺寸)
                        img_width, img_height = 1280, 960
                        x_norm = x / img_width - 0.5
                        y_norm = y / img_height - 0.5
                        
                        if conf == 0:
                            x_norm, y_norm = 0, 0

                        data[0, 0, i, j, 0] = x_norm
                        data[0, 1, i, j, 0] = y_norm
                        data[0, 2, i, j, 0] = conf
        
        # 转换为Tensor
        data = torch.from_numpy(data)
        
        # 移至GPU（如果可用）
        if torch.cuda.is_available():
            data = data.cuda()
        
        # 推理
        with torch.no_grad():
            output, feature = stgcn_model.extract_feature(data)
            
            # 对整个序列的预测
            voting_label = output.sum(dim=3).sum(dim=2).sum(dim=0).argmax(dim=0).item()
            
            probabilities = torch.softmax(output.sum(dim=3).sum(dim=2).sum(dim=0), dim=0)
            confidence = probabilities[voting_label].item()
            
            if voting_label < len(action_classes):
                action = action_classes[voting_label]
            else:
                action = "unknown"
            
            return action, confidence
    
    except Exception as e:
        logger.error(f"ST-GCN推理错误: {e}", exc_info=True)
        return predict_action_with_rules(keypoints_sequence)

# 使用简单规则进行动作预测（备用方法）
def predict_action_with_rules(keypoints_sequence):
    """
    使用简单规则预测动作
    keypoints_sequence: 一个人物的关键点序列，形状为 [frames, joints, 3]
    返回: (预测的动作类别, 置信度)
    """
    if len(keypoints_sequence) < 5:
        return "unknown", 0.5
    
    motion = 0
    for i in range(1, len(keypoints_sequence)):
        prev_frame = keypoints_sequence[i-1]
        curr_frame = keypoints_sequence[i]
        if prev_frame is not None and curr_frame is not None:
            dists = []
            for j in range(min(len(prev_frame), len(curr_frame))):
                prev_pt = prev_frame[j]
                curr_pt = curr_frame[j]
                if prev_pt is not None and curr_pt is not None:
                    dist = math.sqrt((curr_pt[0] - prev_pt[0])**2 + (curr_pt[1] - prev_pt[1])**2)
                    dists.append(dist)
            if dists:
                motion += sum(dists) / len(dists)
    
    avg_motion = motion / (len(keypoints_sequence) - 1) if len(keypoints_sequence) > 1 else 0
    
    if avg_motion > 15:
        return "running", 0.8
    elif avg_motion > 5:
        return "walking", 0.9
    elif avg_motion < 2:
        last_frame = keypoints_sequence[-1]
        if last_frame is not None and len(last_frame) > 0:
            head_y = last_frame[0][1] if len(last_frame) > 0 else 0
            # 关键点13是左踝，10是右踝，检查是否头部低于脚踝，粗略判断摔倒
            ankle_y = (last_frame[13][1] + last_frame[10][1]) / 2 if len(last_frame) > 13 and len(last_frame) > 10 else 0
            if head_y > ankle_y and abs(head_y - ankle_y) > 50: # 假设头部明显低于脚踝
                return "falling", 0.85
        return "standing", 0.7
    
    return "standing", 0.6

# 小车位置和状态
robot_car = {
    'position': {'x': 0, 'y': 0},  # 像素坐标
    'speed': {'x': 0, 'y': 0},     # 像素/秒
    'target_person': None,         # 目标人物ID
    'last_update_time': time.time()
}

# 创建RobotTracker实例
robot_tracker = RobotTracker(
    camera_height=2.5,  # 假设相机高度2.5米
    camera_angle=45     # 假设相机俯角45度
)

# 视频捕获类
class VideoCapture:
    def __init__(self, use_test_video=False, test_video_path=""):
        self.cap = None
        self.is_video = False
        self.lock = threading.Lock()

        if use_test_video:
            if not os.path.exists(test_video_path):
                logger.error(f"测试视频文件不存在: {test_video_path}")
                raise FileNotFoundError(f"测试视频文件不存在: {test_video_path}")
            self.cap = cv2.VideoCapture(test_video_path)
            self.is_video = True
            logger.info(f"本地视频源：使用视频文件 {test_video_path}")
        else:
            self.cap = cv2.VideoCapture(0) # 尝试打开默认摄像头
            if not self.cap.isOpened(): # 如果默认摄像头打不开，尝试其他索引
                logger.warning("Default camera (index 0) not found, trying index 1.")
                actual_stream_url = f"http://127.0.0.1:8080/stream.mjpg"
                self.cap = cv2.VideoCapture(actual_stream_url)
            self.is_video = False
            logger.info("本地视频源：使用本地摄像头")
        
        if not self.cap.isOpened():
            raise RuntimeError("无法打开本地视频源")
        
        # 统一本地视频源的读取分辨率，以便后续处理
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        logger.info(f"本地视频源分辨率设置为: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    
    def read(self):
        with self.lock:
            if self.cap is None:
                return False, None
            ret, frame = self.cap.read()
            if not ret:
                if self.is_video:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                    if not ret:
                        return False, None
                else:
                    return False, None
            return ret, frame.copy()  # 返回帧的副本
    
    def release(self):
        with self.lock:
            if self.cap:
                self.cap.release()
                self.cap = None

# 创建视频捕获实例 (在startup_event中初始化)
local_video_source = None # 重命名以区分树莓派的流

def calculate_speed(track_id: int, current_x: float, current_y: float) -> Tuple[float, float]:
    """
    计算目标的速度（使用窗口平均）
    返回: (speed_x, speed_y) 单位：像素/秒
    """
    if track_id not in position_history:
        position_history[track_id] = collections.deque(maxlen=10)

    current_time = time.time()
    position_history[track_id].append((current_time, current_x, current_y))

    if len(position_history[track_id]) < 2:
        return 0.0, 0.0

    window_size = min(5, len(position_history[track_id]))
    recent_positions = list(position_history[track_id])[-window_size:]

    total_dx = 0.0
    total_dy = 0.0
    total_dt = 0.0

    for i in range(1, len(recent_positions)):
        t1, x1, y1 = recent_positions[i-1]
        t2, x2, y2 = recent_positions[i]
        dt = t2 - t1
        if dt > 0:
            dx = (x2 - x1) / dt
            dy = (y2 - y1) / dt
            total_dx += dx
            total_dy += dy
            total_dt += 1

    if total_dt > 0:
        avg_speed_x = total_dx / total_dt
        avg_speed_y = total_dy / total_dt
        return avg_speed_x, avg_speed_y

    return 0.0, 0.0

# 人物特征提取和匹配
def get_body_crop(image: np.ndarray, bbox: tuple, keypoints: List) -> Optional[np.ndarray]:
    """
    根据关键点获取肩膀以下的身体部分
    keypoints格式：[[x, y, conf], ...]
    关键点索引：2-右肩, 5-左肩
    """
    try:
        x1, y1, x2, y2 = map(int, bbox)
        
        # 确保 bbox 坐标在图像范围内
        h, w = image.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return None # 无效的bbox

        # 获取肩膀位置
        if keypoints and len(keypoints) >= 6:
            right_shoulder = keypoints[2]
            left_shoulder = keypoints[5]

            # 展开嵌套（有些情况下关键点为[[x, y, conf]]）
            if isinstance(right_shoulder[0], (list, tuple)):
                right_shoulder = right_shoulder[0]
            if isinstance(left_shoulder[0], (list, tuple)):
                left_shoulder = left_shoulder[0]

            # 检查置信度，并且肩膀关键点必须在 bbox 内部
            if (isinstance(right_shoulder[2], (int, float)) and isinstance(left_shoulder[2], (int, float)) and
                right_shoulder[2] > 0.3 and left_shoulder[2] > 0.3 and
                x1 <= right_shoulder[0] <= x2 and y1 <= right_shoulder[1] <= y2 and
                x1 <= left_shoulder[0] <= x2 and y1 <= left_shoulder[1] <= y2):
                
                shoulder_y = int((right_shoulder[1] + left_shoulder[1]) / 2)
                shoulder_y = max(y1, min(y2, shoulder_y)) # 确保在bbox范围内
                
                body_img = image[shoulder_y:y2, x1:x2]
                
                if body_img is not None and body_img.size > 0 and body_img.shape[0] > 20:
                    return body_img

        # 如果无法获取有效的肩膀位置，使用下半身
        half_height = (y2 - y1) // 2
        body_img = image[y1 + half_height:y2, x1:x2]

        if body_img is not None and body_img.size > 0 and body_img.shape[0] > 20:
            return body_img
        return None

    except Exception as e:
        logger.error(f"截取身体区域失败: {e}", exc_info=True)
        return None

def extract_features(image: np.ndarray) -> Optional[np.ndarray]:
    """提取图像特征"""
    try:
        if image is None or image.size == 0 or image.shape[0] < 10 or image.shape[1] < 10:
            return None

        # 调整图像大小到合适的尺寸
        target_height = 256
        aspect_ratio = image.shape[1] / image.shape[0]
        target_width = int(target_height * aspect_ratio)
        # 确保宽度至少为1，避免 cv2.resize 报错
        if target_width == 0: target_width = 1
        
        image = cv2.resize(image, (target_width, target_height))
            
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        features = extractor(image)
        return features.cpu().numpy()
    except Exception as e:
        logger.error(f"特征提取错误: {e}", exc_info=True)
        return None

def compute_similarity(features1: np.ndarray, features2: np.ndarray) -> float:
    """计算两个特征向量的相似度（余弦相似度）"""
    try:
        f1 = features1.flatten()
        f2 = features2.flatten()
        
        norm1 = np.linalg.norm(f1)
        norm2 = np.linalg.norm(f2)
        if norm1 == 0 or norm2 == 0:
            return 0
        return np.dot(f1, f2) / (norm1 * norm2)
    except Exception as e:
        logger.error(f"计算相似度错误: {e}", exc_info=True)
        return 0

def update_person_features(person_id: str, image: np.ndarray, bbox: tuple, keypoints: List):
    """更新人物特征库"""
    try:
        body_img = get_body_crop(image, bbox, keypoints)
        if body_img is None:
            return
        
        features = extract_features(body_img)
        if features is None:
            return
        
        with feature_lock:
            if person_id not in person_features:
                _, buffer = cv2.imencode('.jpg', body_img)
                img_base64 = base64.b64encode(buffer).decode('utf-8')
                
                person_features[person_id] = {
                    'features': [features],
                    'image': f"data:image/jpeg;base64,{img_base64}"
                }
            else:
                if 'features' not in person_features[person_id]:
                    person_features[person_id]['features'] = []
                
                person_features[person_id]['features'].append(features)
                if len(person_features[person_id]['features']) > 10:
                    person_features[person_id]['features'].pop(0)
    
    except Exception as e:
        logger.error(f"更新特征错误: {e}", exc_info=True)

def match_person(image: np.ndarray, bbox: tuple, keypoints: List) -> Optional[str]:
    """
    匹配人物ID
    返回：匹配到的person_id或None
    """
    try:
        body_img = get_body_crop(image, bbox, keypoints)
        if body_img is None:
            return None
        
        features = extract_features(body_img)
        if features is None:
            return None
        
        best_match = None
        highest_similarity = 0.9
        
        with feature_lock:
            for person_id, person_data in person_features.items():
                if 'features' not in person_data or not person_data['features']:
                    continue
                    
                for stored_features in person_data['features']:
                    similarity = compute_similarity(features, stored_features)
                    if similarity > highest_similarity:
                        highest_similarity = similarity
                        best_match = person_id
        
        return best_match
    
    except Exception as e:
        logger.error(f"特征匹配错误: {e}", exc_info=True)
        return None

def find_target_in_frame(frame: np.ndarray, detections: List[Dict], target_features: Dict) -> Optional[Dict]:
    """
    在当前帧中寻找目标人物
    target_features: { 'features': [np.ndarray, ...], 'image': 'base64_str' }
    返回：匹配到的检测结果或None
    """
    try:
        best_match = None
        highest_similarity = 0.1
        
        if not target_features or 'features' not in target_features or not target_features['features']:
            logger.warning("Target features are not available for matching.")
            return None

        # 尝试使用最新的目标特征进行匹配
        primary_target_feature = target_features['features'][-1]
        
        for detection in detections:
            det_bbox = (detection['x'], detection['y'], detection['x'] + detection['width'], detection['y'] + detection['height'])
            body_img = get_body_crop(frame, det_bbox, detection['keypoints'])
            
            if body_img is None:
                        continue
                    
            features = extract_features(body_img)
            if features is None:
                continue
            
            similarity = compute_similarity(features, primary_target_feature)
                
            if similarity > highest_similarity:
                highest_similarity = similarity
                best_match = detection
        
        if best_match:
            logger.info(f"找到目标人物匹配，相似度：{highest_similarity:.2f}")
        return best_match
    
    except Exception as e:
        logger.error(f"在帧中查找目标失败: {e}", exc_info=True)
        return None

# 初始化CLIP模型
device = "cuda" if torch.cuda.is_available() else "cpu"
clip_model, clip_preprocess = clip.load("ViT-B/32", device=device)

# 存储目标描述和特征
target_description = None
target_text_features = None
use_clip_matching = False

def compute_clip_similarity(image: np.ndarray, text_features: torch.Tensor) -> float:
    """计算图像和文本特征的相似度"""
    try:
        # CLIP模型期望的输入是PIL Image，且是RGB格式
        if image.shape[2] == 4: # RGBA to RGB
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
        else: # BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        image_pil = Image.fromarray(image)
        image_input = clip_preprocess(image_pil).unsqueeze(0).to(device)
        
        with torch.no_grad():
            image_features = clip_model.encode_image(image_input)
            image_features = image_features / image_features.norm(dim=-1, keepdim=True)
            
        similarity = torch.cosine_similarity(image_features, text_features.unsqueeze(0))
        return float(similarity[0])
    except Exception as e:
        logger.error(f"计算CLIP相似度失败: {e}", exc_info=True)
        return 0.0

def encode_text_clip(text: str) -> Optional[torch.Tensor]:
    """使用CLIP编码文本描述"""
    try:
        with torch.no_grad():
            text_tokens = clip.tokenize([text]).to(device)
            text_features = clip_model.encode_text(text_tokens)
            text_features = text_features / text_features.norm(dim=-1, keepdim=True)
            return text_features[0]
    except Exception as e:
        logger.error(f"CLIP文本编码失败: {e}", exc_info=True)
        return None

def send_full_auto_info(score, angle, direction, x1, y1, x2, y2):
    """发送AUTO_INFO MQTT消息，只包含score, direction和坐标信息"""
    try:
        score = int(score)
        direction = int(direction)
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
    except Exception as e:
        logger.error(f"MQTT参数类型转换失败: {e}")
        return

    if any(v is None for v in [score, direction, x1, y1, x2, y2]):
        logger.warning(f"MQTT消息字段有空值，取消发送: score={score}, direction={direction}, bbox=({x1}, {y1}, {x2}, {y2})")
        return

    data = {
        "command": "AUTO_INFO",
        "score": score,
        "direction": direction,
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2
    }

    logger.info(f"准备发送AUTO_INFO MQTT: {data}")
    if mqtt_client:
        try:
            mqtt_client.publish(MQTT_CONTROL_TOPIC, json.dumps(data))
            logger.info(f"已发送AUTO_INFO MQTT: {data}")
        except Exception as e:
            logger.error(f"MQTT发送失败: {e}", exc_info=True)
    else:
        logger.warning("MQTT client not available, cannot send AUTO_INFO.")

def calculate_real_world_coordinates(x, y, frame_height=960): # 调整为处理帧的统一尺寸
    """
    将图像坐标转换为实际世界坐标（考虑透视变换）
    使用简化的针孔相机模型
    """
    CAMERA_HEIGHT = 2.5
    CAMERA_ANGLE = math.radians(45)
    FOCAL_LENGTH = 800

    y_relative = (frame_height/2 - y) / FOCAL_LENGTH
    angle = CAMERA_ANGLE + math.atan(y_relative)
    distance_y = CAMERA_HEIGHT / math.tan(angle)

    x_relative = (x - frame_height/2) / FOCAL_LENGTH
    distance_x = distance_y * x_relative

    return distance_x, distance_y

def calculate_relative_angle_with_perspective(target_x, target_y, robot_x, robot_y, robot_orientation):
    """
    计算考虑透视变换的相对角度
    """
    # 传入的 target_x, target_y, robot_x, robot_y 是 1280x960 帧上的像素坐标
    # calculate_real_world_coordinates 会将它们转换为实际世界坐标
    target_real_x, target_real_y = calculate_real_world_coordinates(target_x, target_y, frame_height=960)
    robot_real_x, robot_real_y = calculate_real_world_coordinates(robot_x, robot_y, frame_height=960)

    dx = target_real_x - robot_real_x
    dy = target_real_y - robot_real_y

    target_angle = math.degrees(math.atan2(dy, dx)) % 360

    relative_angle = (target_angle - robot_orientation) % 360
    if relative_angle > 180:
        relative_angle -= 360

    return relative_angle, math.sqrt(dx*dx + dy*dy)

def calculate_iou(box1, box2):
    """计算两个边界框的IoU"""
    x1_1, y1_1, x2_1, y2_1 = box1
    x1_2, y1_2, x2_2, y2_2 = box2

    x1_inter = max(x1_1, x1_2)
    y1_inter = max(y1_1, y1_2)
    x2_inter = min(x2_1, x2_2)
    y2_inter = min(y2_1, y2_2)

    inter_area = max(0, x2_inter - x1_inter) * max(0, y2_inter - y1_inter)

    box1_area = (x2_1 - x1_1) * (y2_1 - y1_1)
    box2_area = (x2_2 - x1_2) * (y2_2 - y1_2)

    union_area = box1_area + box2_area - inter_area
    iou = inter_area / union_area if union_area > 0 else 0
    return iou


async def process_frame(frame_input):
    """
    处理输入的图像帧，执行AI推理并生成结果。
    这个函数会被不同的视频源（树莓派或本地摄像头）调用。
    frame_input: 原始尺寸的图像帧 (例如 640x480 for RPi/Local Camera)
    """
    global detected_persons, keypoints_sequences, robot_car, detection_counts
    global confirmed_tracks, position_history, target_text_features, use_clip_matching, robot_status
    global last_detection_run_time, detection_result_cache, detection_result_timestamp
    global locked_target_id
    
    current_time = time.time()
    
    # 检查是否需要运行新的检测 (基于固定的推理间隔)
    run_new_detection = (current_time - last_detection_run_time) >= detection_interval
    
    # 如果缓存的检测结果存在且未过期，直接返回缓存结果
    if not run_new_detection and detection_result_cache is not None:
        if (current_time - detection_result_timestamp) <= result_display_time:
            return detection_result_cache
    
    if model is None or pose_model is None or obb_model is None:
        logger.error("Model not loaded for process_frame.")
        return []

    try:
        if frame_input is None:
            logger.error("process_frame received None frame_input")
            return []
            
        # 复制并统一推理尺寸
        frame_for_inference = cv2.resize(frame_input, (640, 480)) # 所有AI推理都基于这个尺寸
        
        # 更新最后检测时间
        last_detection_run_time = current_time
        
        # --- 并行执行检测任务 ---
        async def run_yolo_track():
            with torch.no_grad():
                return model.track(frame_for_inference, persist=True, verbose=False, device=device)
        async def run_pose_track():
            try:
                with torch.no_grad():
                    return pose_model.track(frame_for_inference, persist=True, verbose=False, device=device)
            except Exception as e:
                logger.error(f"pose_model.track error: {e}", exc_info=True)
                return []
        async def run_obb_track():
            with torch.no_grad():
                return obb_model.track(frame_for_inference, persist=True, verbose=False, classes=[2, 7], device=device) # 2: car, 7: truck
        
        yolo_results_task = asyncio.create_task(run_yolo_track())
        pose_results_task = asyncio.create_task(run_pose_track())
        obb_results_task = asyncio.create_task(run_obb_track())
        yolo_results = await yolo_results_task
        pose_results = await pose_results_task
        obb_results = await obb_results_task
        
        # --- 处理检测结果 ---
        detections = []
        car_detections = []
        robot_bbox = None
        robot_orientation = 0
        
        for r in obb_results:
            if hasattr(r, 'boxes') and r.boxes is not None:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    class_name = obb_model.names.get(cls, 'unknown')
                    if class_name.lower() in ['car', 'truck']:
                        robot_bbox = (x1, y1, x2, y2)
                        if hasattr(box, 'obb') and box.obb is not None and len(box.obb.data) > 0:
                            if len(box.obb.data[0]) >= 5: # Assuming OBB format is [x,y,w,h,angle_rad]
                                robot_orientation = float(np.degrees(box.obb.data[0][4])) % 360
                                logger.debug(f"Robot orientation from OBB: {robot_orientation}°")
                        car_detections.append({
                            'bbox': robot_bbox,
                            'confidence': conf,
                            'orientation': robot_orientation
                        })
                        
        if not robot_bbox and yolo_results: # Fallback to YOLO for robot bbox if OBB misses
            for r in yolo_results:
                if hasattr(r, 'boxes') and r.boxes is not None:
                    for box in r.boxes:
                        cls = int(box.cls[0])
                        class_name = model.names.get(cls, 'unknown')
                        if class_name.lower() in ['car', 'truck']:
                            x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                            robot_bbox = (x1, y1, x2, y2)
                            break
                    if robot_bbox: break

        if yolo_results:
            for r in yolo_results:
                if hasattr(r, 'boxes') and r.boxes is not None:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])
                        class_name = model.names.get(cls, 'unknown')
                        track_id = int(box.id[0]) if box.id is not None else None

                        if class_name.lower() != 'person' or track_id is None:
                            continue

                        if track_id not in detection_counts:
                            detection_counts[track_id] = 0
                        detection_counts[track_id] += 1
                        
                        if track_id not in position_history:
                            position_history[track_id] = collections.deque(maxlen=10)
                        position_history[track_id].append((current_time, (x1 + x2) / 2, (y1 + y2) / 2))
                        
                        if detection_counts[track_id] >= 3:
                            confirmed_tracks.add(track_id)
                        if track_id not in confirmed_tracks:
                            continue

                        person_id = f"person_{track_id}"
                        matched_id = match_person(frame_for_inference, (x1, y1, x2, y2), keypoints_sequences.get(person_id, []))
                        if matched_id is not None:
                            person_id = matched_id
                        update_person_features(person_id, frame_for_inference, (x1, y1, x2, y2), keypoints_sequences.get(person_id, []))
                        
                        center_x = float((x1 + x2) / 2)
                        center_y = float(y2)

                        speed_x, speed_y = calculate_speed(track_id, center_x, center_y)
                        
                        keypoints = None
                        max_iou = 0
                        best_pose_keypoints = None
                        
                        for pose_r in pose_results:
                            if hasattr(pose_r, 'boxes') and pose_r.boxes is not None:
                                for j, pose_box in enumerate(pose_r.boxes):
                                    px1, py1, px2, py2 = map(float, pose_box.xyxy[0].cpu().numpy())
                                    iou = calculate_iou((x1, y1, x2, y2), (px1, py1, px2, py2))
                                    
                                    if iou > max_iou:
                                        max_iou = iou
                                        if hasattr(pose_r, 'keypoints') and pose_r.keypoints is not None and j < len(pose_r.keypoints):
                                            keypoints_obj = pose_r.keypoints[j]
                                            if hasattr(keypoints_obj, 'data') and keypoints_obj.data is not None:
                                                kp_array = keypoints_obj.data.cpu().numpy()
                                                if kp_array.shape[2] == 3:
                                                    best_pose_keypoints = [[float(kp[0]), float(kp[1]), float(kp[2])] for kp in kp_array[0]]
                        
                        if max_iou > 0.5 and best_pose_keypoints:
                            keypoints = best_pose_keypoints
                        else:
                            keypoints = generate_keypoints(x1, y1, x2, y2, center_x, center_y)
                        
                        # 更新关键点序列
                        if person_id not in keypoints_sequences:
                            keypoints_sequences[person_id] = collections.deque(maxlen=10) # 保持10帧历史
                            keypoints_sequences[person_id].append(keypoints)
                    
                        person_data = {
                            'id': person_id,
                            'track_id': track_id,
                            'keypoints_sequence': list(keypoints_sequences[person_id]), # 传入完整的序列
                            'center_x': center_x,
                            'center_y': center_y,
                            'speed_x': speed_x,
                            'speed_y': speed_y,
                            'width': float(x2 - x1),
                            'height': float(y2 - y1),
                            'x': float(x1),
                            'y': float(y1),
                        }
                        
                        decision = {}
                        robot_action = 'NONE'
                        
                        if decision_maker and hasattr(decision_maker, 'threat_analyzer') and decision_maker.threat_analyzer.wooden_man_mode:
                            decision = await decision_maker.make_decision(frame_for_inference, person_data, robot_status)
                            robot_action = decision.get('threat_analysis', {}).get('action', 'NONE')
                            
                            if 'threat_analysis' in decision and 'threat_assessment' in decision['threat_analysis']:
                                threat_assessment = decision['threat_analysis']['threat_assessment']
                                if 'Wooden Man Mode' in threat_assessment:
                                    await sio.emit('wooden_man_status', {
                                        'person_id': person_id,
                                        'status': threat_assessment,
                                        'action': decision['threat_analysis'].get('action', 'NONE'),
                                        'threat_score': decision['threat_analysis'].get('threat_score', 0.0)
                                    })
                        elif use_clip_matching and target_text_features is not None:
                            person_img = frame_for_inference[int(y1):int(y2), int(x1):int(x2)]
                            if person_img.size > 0:
                                similarity = compute_clip_similarity(person_img, target_text_features)
                                if similarity > CLIP_SIMILARITY_THRESHOLD:
                                    threat_level = 3.0
                                    robot_action = 'ATTACK'
                                else:
                                    threat_level = 0.0
                                    robot_action = 'NONE'
                                
                                decision = {
                                    'current_action': 'matching',
                                    'action_confidence': similarity,
                                    'threat_analysis': {
                                        'threat_score': threat_level,
                                        'action': robot_action,
                                        'confidence': similarity,
                                        'reasoning': f'CLIP similarity: {similarity:.2f}'
                                    },
                                    'similarity': similarity,
                                    'robot_action': robot_action
                                }
                            else:
                                logger.warning(f"Person image for CLIP matching is empty for {person_id}.")
                                decision = await decision_maker.make_decision(frame_for_inference, person_data, robot_status)
                                robot_action = decision.get('threat_analysis', {}).get('action', 'NONE')
                        else:
                            decision = await decision_maker.make_decision(frame_for_inference, person_data, robot_status)
                            robot_action = decision.get('threat_analysis', {}).get('action', 'NONE')
                        
                        if person_id not in detected_persons:
                            detected_persons[person_id] = {
                                'first_seen': current_time,
                                'action': decision.get('current_action', 'standby'),
                                'action_confidence': decision.get('action_confidence', 0.0)
                            }
                        
                        detected_persons[person_id].update({
                            'action': decision.get('current_action', 'standby'),
                            'action_confidence': decision.get('action_confidence', 0.0),
                            'last_seen': current_time
                        })
                        
                        stay_duration = current_time - detected_persons[person_id]['first_seen']
                        
                        person_img_b64 = None
                        if person_id in person_features and 'image' in person_features[person_id]:
                            person_img_b64 = person_features[person_id]['image'].split(',')[1] if ',' in person_features[person_id]['image'] else person_features[person_id]['image']
                        else:
                            try:
                                person_img = frame_for_inference[int(y1):int(y2), int(x1):int(x2)]
                                if person_img.size > 0:
                                    _, buffer = cv2.imencode('.jpg', person_img)
                                    person_img_b64 = base64.b64encode(buffer).decode('utf-8')
                                    if person_id not in person_features:
                                        person_features[person_id] = {
                                            'features': [],
                                            'image': f"data:image/jpeg;base64,{person_img_b64}"
                                        }
                            except Exception as e:
                                logger.error(f"Error creating person image for {person_id}: {e}", exc_info=True)
                        
                        action_display = decision.get('current_action', 'standby')
                        if decision_maker and hasattr(decision_maker, 'threat_analyzer') and decision_maker.threat_analyzer.wooden_man_mode:
                            if 'wooden_man_action' in decision.get('threat_analysis', {}):
                                action_display = decision['threat_analysis']['wooden_man_action']
                            elif 'threat_analysis' in decision and 'reasoning' in decision['threat_analysis']:
                                reasoning = decision['threat_analysis']['reasoning'].lower()
                                if 'movement detected' in reasoning or 'moving' in reasoning:
                                    action_display = 'moving'
                                else:
                                    action_display = 'stationary'
                        
                        detection_info = {
                        'id': person_id,
                            'track_id': track_id,
                        'x': int(x1),
                        'y': int(y1),
                        'width': int(x2 - x1),
                        'height': int(y2 - y1),
                        'center_x': center_x,
                        'center_y': center_y,
                        'class': class_name,
                        'confidence': conf,
                            'action': action_display,
                            'action_confidence': float(decision.get('action_confidence', 0.0)),
                            'stay_duration': int(stay_duration),
                            'image': f"data:image/jpeg;base64,{person_img_b64}" if person_img_b64 else None,
                        'speed_x': speed_x,
                        'speed_y': speed_y,
                            'keypoints': keypoints,
                            'threat_level': decision.get('threat_analysis', {}).get('threat_score', 0.0),
                            'detection_count': detection_counts.get(track_id, 0),
                            'similarity': decision.get('similarity', 0.0) if use_clip_matching else 0.0,
                            'robot_action': robot_action
                        }
                        detections.append(detection_info)
                        
        for track_id in list(detection_counts.keys()):
            if track_id in position_history and position_history[track_id]:
                if all(current_time - ts > 5.0 for ts, _, _ in position_history[track_id]):
                    logger.debug(f"Clearing track_id {track_id} due to inactivity (last seen more than 5s ago).")
                    detection_counts.pop(track_id, None)
                    confirmed_tracks.discard(track_id)
                    position_history.pop(track_id, None)
                    keypoints_sequences.pop(f"person_{track_id}", None) # 清理关键点序列
            else:
                 detection_counts.pop(track_id, None)
                 confirmed_tracks.discard(track_id)

        current_active_track_ids = {int(det['track_id']) for det in detections if 'track_id' in det and det['track_id'] is not None}
        for person_id, info in list(detected_persons.items()):
            try:
                track_id_from_person_id = int(person_id.split('_')[1])
                if track_id_from_person_id not in current_active_track_ids:
                    if current_time - info['last_seen'] > 5.0:
                        logger.debug(f"Clearing detected_persons entry for {person_id} (last seen more than 5s ago).")
                        del detected_persons[person_id]
            except (ValueError, IndexError): # Handles non-track_id based person_id if any (e.g. initial target)
                if current_time - info['last_seen'] > 5.0:
                    logger.debug(f"Clearing detected_persons entry for {person_id} (malformed ID or not a track, last seen > 5s ago).")
                    del detected_persons[person_id]


        robot_info = None
        if TEST_MODE: # TEST_MODE只影响本地摄像头/视频流
            # 如果是本地视频源，模拟一个机器人位置
            screen_width = frame_for_inference.shape[1]
            screen_height = frame_for_inference.shape[0]
            car_width = screen_width // 16
            car_height = screen_height // 16
            car_x = (screen_width - car_width) // 2
            car_y = (screen_height - car_height) // 2
            
            robot_bbox = (car_x, car_y, car_x + car_width, car_y + car_height)
            if robot_orientation == 0:
                robot_orientation = 0
            
            robot_info = {
                'position': {'x': car_x + car_width // 2, 'y': car_y + car_height // 2},
                'speed': {'x': 0, 'y': 0},
                'orientation': robot_orientation
            }
        else: # 非TEST_MODE下，robot_bbox和robot_orientation从OBB或YOLO检测中获取
            if car_detections:
                best_car = max(car_detections, key=lambda x: x['confidence'])
                robot_bbox = best_car['bbox']
                if best_car.get('orientation', 0) != 0:
                    robot_orientation = best_car['orientation']
                
                robot_info = robot_tracker.update_position(robot_bbox)
                robot_info['orientation'] = robot_orientation
            else:
                robot_info = robot_tracker.get_last_position()
                if robot_orientation == 0:
                    robot_orientation = robot_status['position']['orientation']
                robot_info['orientation'] = robot_orientation
                
        if robot_info:
            robot_status['position'].update({
                'x': robot_info['position']['x'],
                'y': robot_info['position']['y'],
                'orientation': robot_info['orientation']
            })
            robot_car.update({
                'position': robot_info['position'],
                'speed': robot_info['speed'],
                'orientation': robot_orientation,
                'last_update_time': current_time
            })
        
        # --- 计算相对角度和距离，并进行最终目标选择 ---
        # 目标锁定逻辑
        target_person = None
        max_threat_level = 0
        # 先查找当前锁定的目标是否还在detections中
        if locked_target_id is not None:
            for detection in detections:
                if detection.get('id') == locked_target_id:
                    target_person = detection
                    max_threat_level = detection.get('threat_level', 0)
                    break
        # 如果没有锁定目标或锁定目标已消失，则重新选择
        if target_person is None:
            for detection in detections:
                threat_level = detection.get('threat_level', 0)
                if threat_level > max_threat_level:
                    max_threat_level = threat_level
                    target_person = detection
            # 如果选中了新目标，更新锁定id
            if target_person is not None:
                locked_target_id = target_person.get('id')
        
        if target_person and robot_bbox:
            target_x_center = target_person['x'] + target_person['width'] / 2
            target_y_bottom = target_person['y'] + target_person['height']
            
            robot_center_x = (robot_bbox[0] + robot_bbox[2]) / 2
            robot_center_y = (robot_bbox[1] + robot_bbox[3]) / 2
            
            relative_angle, real_distance = calculate_relative_angle_with_perspective(
                target_x_center, target_y_bottom,
                robot_center_x, robot_center_y,
                robot_orientation
            )
            
            discrete_direction = discretize_angle(relative_angle)
            
            target_person.update({
                'relative_angle': relative_angle,
                'discrete_direction': discrete_direction,
                'distance': real_distance
            })
            
            logger.info(f"\n" + "="*50 +
                         f"\n目标信息摘要:" +
                         f"\n- ID: {target_person['id']}" +
                         f"\n- 威胁等级: {target_person['threat_level']:.2f}" +
                         f"\n- 当前动作: {target_person['action']}" +
                         f"\n- 动作置信度: {target_person['action_confidence']:.2f}" +
                         f"\n- 速度: dx={target_person.get('speed_x', 0):.2f}, dy={target_person.get('speed_y', 0):.2f}" +
                         f"\n相对位置信息:" +
                         f"\n- 机器人朝向: {robot_orientation:.2f}°" +
                         f"\n- 相对角度 (相对于机器人朝向): {relative_angle:.2f}° (方向: {discrete_direction})" +
                         f"\n- 相对距离: {real_distance:.2f}米" +
                         f"\n机器人响应:" +
                         f"\n- 威胁等级: {target_person['threat_level']:.2f}" +
                         f"\n- (Agent)相似度: {target_person['similarity']:.2f}" +
                         f"\n- 动作: {target_person['robot_action']}")
        
        robot_status['latest_detections'] = detections
        
        # 绘制检测结果到推理尺寸的帧上，用于发送到前端或保存到缓存
        display_frame_for_output = frame_for_inference.copy()
        if robot_bbox:
            x1_r, y1_r, x2_r, y2_r = map(int, robot_bbox)
            cv2.rectangle(display_frame_for_output, (x1_r, y1_r), (x2_r, y2_r), (0, 255, 255), 2)
            cv2.putText(display_frame_for_output, "Robot", (x1_r, y1_r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            center_rx, center_ry = (x1_r + x2_r) // 2, (y1_r + y2_r) // 2
            length = 50
            angle_rad = np.radians(robot_orientation)
            end_x = center_rx + length * np.sin(angle_rad)
            end_y = center_ry - length * np.cos(angle_rad)
            cv2.arrowedLine(display_frame_for_output, (center_rx, center_ry), (int(end_x), int(end_y)), (0, 0, 255), 2)
        
        for detection in detections:
            x, y, w, h = detection['x'], detection['y'], detection['width'], detection['height']
            person_id = detection['id']
            track_id = detection['track_id']
            robot_action = detection['robot_action']
            threat_level = detection['threat_level']
            cv2.rectangle(display_frame_for_output, (x, y), (x + w, y + h), (255, 0, 0), 2)
            label = f"{person_id} [{track_id}] Act:{robot_action} Thr:{threat_level:.1f}"
            cv2.putText(display_frame_for_output, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            if target_person and target_person.get('id') == person_id:
                cv2.rectangle(display_frame_for_output, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(display_frame_for_output, f"TARGET - {robot_action}", (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        _, buffer = cv2.imencode('.jpg', display_frame_for_output) # 编码的是 1280x960 的帧
        processed_frame_base64 = base64.b64encode(buffer).decode('utf-8')
        
        result = {
            'frame': f"data:image/jpeg;base64,{processed_frame_base64}",
            'detections': detections,
            'robot_status': robot_status
        }
        
        detection_result_cache = result
        detection_result_timestamp = current_time
        
        return result
    
    except Exception as e:
        logger.error(f"Error processing frame: {e}", exc_info=True)
        return []

# 添加一个函数用于在本地显示树莓派摄像头画面及检测结果
def show_pi_camera_locally(original_rpi_frame, matched_detection=None):
    try:
        display_frame = original_rpi_frame.copy() if original_rpi_frame is not None else None
        if display_frame is None:
            return

        # 只绘制 matched_detection
        if matched_detection:
            # 计算缩放比例
            inference_width, inference_height = 1280, 960
            display_width, display_height = display_frame.shape[1], display_frame.shape[0]
            scale_x = display_width / inference_width
            scale_y = display_height / inference_height

            # 绘制边界框
            x, y = matched_detection.get('x', 0), matched_detection.get('y', 0)
            w, h = matched_detection.get('width', 0), matched_detection.get('height', 0)
            cv2.rectangle(display_frame, (int(x*scale_x), int(y*scale_y)),
                          (int((x + w)*scale_x), int((y + h)*scale_y)), (0, 0, 255), 2)
            # 绘制标签
            label = f"{matched_detection.get('id', 'target')}"
            cv2.putText(display_frame, label, (int(x*scale_x), int(y*scale_y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5 * min(scale_x, scale_y), (0, 0, 255), 2)
            # 绘制关键点
            keypoints = matched_detection.get('keypoints', [])
            for kp in keypoints:
                if kp and len(kp) >= 3 and kp[2] > 0.5:
                    cv2.circle(display_frame, (int(kp[0]*scale_x), int(kp[1]*scale_y)),
                               int(3 * min(scale_x, scale_y)), (0, 255, 255), -1)

        # 显示
        cv2.imshow("Raspberry Pi Camera", display_frame)
        cv2.waitKey(1)
    except Exception as e:
        logger.error(f"显示树莓派摄像头画面错误: {e}", exc_info=True)

# 修改 convert_to_json_serializable 函数，用于处理 numpy 数组的 JSON 序列化
def convert_to_json_serializable(obj):
    """将对象转换为 JSON 可序列化的格式"""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, (np.bool_)):
        return bool(obj)
    else:
        return obj

# 处理树莓派图像队列的函数
async def process_pi_frame():
    """处理来自树莓派的图像，并将其显示在本地OpenCV窗口"""
    global frame_queue, robot_status, person_features, last_detection_run_time

    while True:
        try:
            # 清空队列，只保留最新帧
            latest_frame_from_queue = None
            while not frame_queue.empty():
                try:
                    frame = frame_queue.get_nowait()
                    if frame is not None:
                        latest_frame_from_queue = frame.copy()
                except queue.Empty:
                    break
            
            if latest_frame_from_queue is not None:
                result_for_inference = await process_frame(latest_frame_from_queue)
                if not result_for_inference:
                    await asyncio.sleep(0.01) # 短暂等待
                    continue
                
                # 在本地显示树莓派摄像头画面
                show_pi_camera_locally(latest_frame_from_queue) # 传入原始尺寸的树莓派帧

                # 1. 从广角摄像头结果中找到威胁度最高的目标
                target_person = None
                max_threat_level = 0
                score = 0
                direction = 0
                
                if robot_status.get('latest_detections'):
                    for detection in robot_status['latest_detections']:
                        threat_level = detection.get('threat_level', 0)
                        if threat_level > max_threat_level:
                            max_threat_level = threat_level
                            target_person = detection
                            score = int(min(3, threat_level))
                            direction = detection.get('discrete_direction', 0)
                
                # 2. 在树莓派画面中寻找该目标的匹配
                x1 = y1 = x2 = y2 = 0 # 默认坐标
                if target_person and target_person.get('id') in person_features:
                    target_features_data = person_features[target_person['id']] 
                    
                    matched_detection = find_target_in_frame(latest_frame_from_queue, result_for_inference.get('detections', []), target_features_data)
                    
                    if matched_detection:
                        x1 = int(matched_detection['x'])
                        y1 = int(matched_detection['y'])
                        x2 = int(matched_detection['x'] + matched_detection['width'])
                        y2 = int(matched_detection['y'] + matched_detection['height'])
                        logger.info(f"在树莓派画面中找到目标 {target_person['id']} 的匹配，坐标: ({x1}, {y1}, {x2}, {y2})")
                    else:
                        logger.info(f"未在树莓派画面中找到目标 {target_person['id']} 的匹配，使用默认坐标")
                
                frame_size = latest_frame_from_queue.shape[1], latest_frame_from_queue.shape[0]
                # 3. 发送MQTT消息（只包含score, direction和坐标）
                if TEST_MODE: # 修正：TEST_MODE只影响本地视频源，不影响树莓派交互
                    score = 3 # 仅用于测试MQTT消息发送
                
                send_full_auto_info(score, 0.0, direction, x1, y1, x2, y2)
            
            await asyncio.sleep(0.4) # 控制循环速率，稍微比SocketStreamServer快一点，确保处理最新帧
            
        except Exception as e:
            logger.error(f"处理树莓派图像错误: {e}", exc_info=True)
            await asyncio.sleep(1)

# 路由：主页
@fastapi_app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

# WebSocket：客户端连接
@sio.event
async def connect(sid, environ):
    logger.info(f'Client connected: {sid}')
    await sio.emit('status_update', {
        'type': 'status',
        'status': robot_status
    }, to=sid)

# WebSocket：客户端断开连接
@sio.event
async def disconnect(sid):
    logger.info(f'Client disconnected: {sid}')

# WebSocket：接收视频帧 (用于向网页前端发送本地视频源的帧)
@sio.event
async def video_frame(sid, data): # `data` 参数现在可以忽略，因为我们从本地文件或摄像头读取
    global frame_counter, last_gc_time, web_stream_last_frame_time, local_video_source
    
    current_time = time.time()
    elapsed = current_time - web_stream_last_frame_time
    if elapsed < web_stream_frame_interval:
        return
    web_stream_last_frame_time = current_time
    
    if local_video_source is None:
        logger.warning("Local video source is not initialized, cannot stream to web. Skipping video_frame event.")
        return

    try:
        ret, frame = local_video_source.read() # 从本地视频源读取帧
        if not ret or frame is None:
            logger.error("无法读取本地视频源帧")
            return
        
        # 对本地视频源的帧进行AI处理，结果将用于网页显示
        result = await process_frame(frame) # 传入原始尺寸的本地帧
        if not result:
                return
        
        await sio.emit('detection_result', {
            'type': 'detection',
                'detections': result, # result 包含了 1280x960 的 base64 图像
                'source': 'local'
        }, to=sid)
        
        frame_counter += 1
        
        if frame_counter >= 50 or (current_time - last_gc_time) > 5:
            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            frame_counter = 0
            last_gc_time = current_time
            logger.debug("执行垃圾回收，释放内存")
    
    except Exception as e:
        logger.error(f"处理本地视频源帧并发送到前端错误: {e}", exc_info=True)
        return

# WebSocket：控制模式切换
@sio.event
async def control_mode(sid, data):
    mode = data.get('mode')
    robot_status['control_mode'] = mode
    
    if mode == 'manual':
        robot_status['working_status'] = 'Manual'
    else:
        robot_status['working_status'] = 'Patrolling'
    
    await sio.emit('status_update', {
        'type': 'status',
        'status': robot_status
    })
    
    send_car_command("SET_MODE", mode=mode) # 通过MQTT通知小车

# WebSocket：相机开关控制
@sio.event
async def camera_toggle(sid, data):
    enabled = data.get('enabled', False)
    robot_status['camera_enabled'] = enabled
    
    await sio.emit('status_update', {
        'type': 'status',
        'status': robot_status
    })
    
    logger.info(f"Camera {'enabled' if enabled else 'disabled'}")
    # 可以通过MQTT通知树莓派调整摄像头状态，例如：send_car_command("CAMERA_TOGGLE", enabled=enabled)

# WebSocket：移动控制convert_to_json_serializable
@sio.event
async def movement(sid, data):
    direction = data.get('direction')
    speed = data.get('speed', 150)
    direction_map = {
        'forward': 'F',
        'backward': 'B',
        'left': 'L',
        'right': 'R',
        'stop': 'S',
        'auto': 'AUTO'
    }
    car_cmd = direction_map.get(direction, direction)
    send_car_command(car_cmd, speed)

# WebSocket：功能按钮
@sio.event
async def function(sid, data):
    action = data.get('action')
    logger.info(f"Function command: {action}")
    # 这里也可以通过MQTT发送给小车
    send_car_command("FUNCTION", action=action)

# WebSocket：Agent命令
@sio.event
async def agent_command(sid, data):
    global target_description, target_text_features, use_clip_matching
    
    command = data.get('command')
    command_type = data.get('type')
    logger.info(f"Agent command: {command} (type: {command_type})")
    
    if command_type == 'target_description':
        target_description = command
        target_text_features = encode_text_clip(command)
        use_clip_matching = True
        
        response = {
            'status': 'success',
            'message': f"目标描述已更新: {command}"
        }
    elif command_type == 'toggle_matching':
        use_clip_matching = data.get('enabled', False)
        response = {
            'status': 'success',
            'message': f"特征匹配模式已{'启用' if use_clip_matching else '禁用'}"
        }
    elif command_type == 'toggle_wooden_man':
        enabled = data.get('enabled', False)
        if decision_maker and hasattr(decision_maker, 'threat_analyzer'):
            decision_maker.threat_analyzer.toggle_wooden_man_mode(enabled)
            response = {
                'status': 'success',
                'message': f"123木头人模式已{'启用' if enabled else '禁用'}"
            }
        else:
            response = {
                'status': 'error',
                'message': "无法切换123木头人模式，威胁分析器未初始化"
            }
    else:
        response = {
            'status': 'error',
            'message': f"未知的命令类型: {command_type}"
        }
    
    await sio.emit('agent_response', response, to=sid)

# 更新机器人状态的异步函数
async def update_robot_status():
    while True:
        robot_status['battery_level'] = max(0, robot_status['battery_level'] - np.random.uniform(0, 0.1))
        signal_change = np.random.normal(0, 1)
        robot_status['signal_strength'] = max(0, min(100, robot_status['signal_strength'] + signal_change))
        
        await sio.emit('status_update', {
            'type': 'status',
            'status': robot_status
        })
        
        await asyncio.sleep(10)

# 修改启动事件，添加 OpenCV 窗口初始化
@fastapi_app.on_event("startup")
async def startup_event():
    # 初始化 OpenCV 窗口
    try:
        cv2.namedWindow("Raspberry Pi Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Raspberry Pi Camera", socket_server.rpi_stream_res[0], socket_server.rpi_stream_res[1])
    except Exception as e:
        logger.error(f"OpenCV 窗口初始化失败: {e}")

    # 修复：初始化本地视频源
    global local_video_source
    try:
        local_video_source = VideoCapture(use_test_video=CAMERA_MODE, test_video_path=TEST_VIDEO_PATH)
        logger.info("Local video source initialized.")
    except Exception as e:
        logger.error(f"Failed to initialize local video source: {e}")
        # 如果本地视频源初始化失败，但非测试模式，可以不退出，因为树莓派流仍可工作
        if TEST_MODE:
            sys.exit(1)

    # 启动Socket服务器 (接收树莓派流)
    try:
        await socket_server.start()
        logger.info("Socket服务器启动成功 (接收树莓派流)")
        # 启动树莓派图像处理任务 (显示到本地窗口)
        asyncio.create_task(process_pi_frame())
        logger.info("树莓派图像处理任务启动成功")
    except Exception as e:
        logger.error(f"Socket服务器启动失败: {e}")
        sys.exit(1)
    
    # 启动状态更新任务
    asyncio.create_task(update_robot_status())
    logger.info("Started robot status update task")

# 修改清理资源函数，添加关闭 OpenCV 窗口
@fastapi_app.on_event("shutdown")
async def shutdown_event():
    global local_video_source, socket_server, mqtt_client
    
    if socket_server:
        socket_server.running = False
        await asyncio.sleep(0.5)

    if local_video_source:
        try:
            local_video_source.release()
            logger.info("Local video source released.")
        except Exception as e:
            logger.error(f"释放本地视频源失败: {e}")

    if mqtt_client:
        try:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
            logger.info("MQTT client disconnected.")
        except Exception as e:
            logger.error(f"Failed to disconnect MQTT client: {e}")

    try:
        cv2.destroyAllWindows()
        for _ in range(5):
            cv2.waitKey(1)
    except Exception as e:
        logger.error(f"关闭 OpenCV 窗口失败: {e}")

    if torch.cuda.is_available():
        try:
            torch.cuda.empty_cache()
        except Exception as e:
            logger.error(f"清理CUDA缓存失败: {e}")
            
    logger.info("Cleaned up resources")

# 主函数
if __name__ == '__main__':
    logger.info("Starting Security Robot Control Interface with FastAPI")
    uvicorn.run("app:app", host="0.0.0.0", port=5000, reload=True) 