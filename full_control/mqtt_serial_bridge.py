#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt
import serial
import json
import time
import threading
import logging
from queue import Queue

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MQTTSerialBridge:
    def __init__(self):
        # 串口配置
        self.serial_port = '/dev/ttyACM0'  # 如果不行，尝试 /dev/ttyUSB0
        self.serial_baudrate = 9600
        self.serial_timeout = 1
        self.ser = None
        
        # MQTT配置
        self.mqtt_broker = "localhost"
        self.mqtt_port = 1883
        self.mqtt_user = "pi"
        self.mqtt_pass = "yjh123"
        self.control_topic = "car/control"
        self.status_topic = "car/status"
        
        # 初始化MQTT客户端
        self.client = mqtt.Client()
        self.client.username_pw_set(self.mqtt_user, self.mqtt_pass)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # 命令队列
        self.command_queue = Queue()
        
        # 运行状态
        self.running = False
        
    def initialize_serial(self):
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=self.serial_timeout,
                write_timeout=1
            )
            logger.info(f"串口 {self.serial_port} 连接成功")
            time.sleep(2)  # 等待Arduino重启
            return True
        except serial.SerialException as e:
            logger.error(f"串口连接失败: {e}")
            return False
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            logger.info("MQTT连接成功")
            client.subscribe(self.control_topic)
            # 发送状态消息
            client.publish(self.status_topic, json.dumps({"status": "connected"}))
        else:
            logger.error(f"MQTT连接失败，错误代码: {rc}")
    
    def on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        logger.warning("MQTT连接断开")
    
    def on_message(self, client, userdata, msg):
        """MQTT消息回调"""
        try:
            payload = msg.payload.decode('utf-8')
            logger.info(f"收到MQTT消息: {payload}")
            
            # 解析JSON命令
            if payload.startswith('{'):
                command_data = json.loads(payload)
                self.command_queue.put(command_data)
            else:
                # 简单字符命令
                self.command_queue.put({"command": payload.strip()})
                
        except Exception as e:
            logger.error(f"处理MQTT消息错误: {e}")
    
    def send_to_arduino(self, command_data):
        """发送命令到Arduino"""
        if not self.ser or not self.ser.is_open:
            logger.error("串口未连接")
            return False
        
        try:
            # 转换为JSON字符串
            command_str = json.dumps(command_data)
            
            # 发送到Arduino
            self.ser.write((command_str + '\n').encode('utf-8'))
            self.ser.flush()
            
            logger.info(f"发送到Arduino: {command_str}")
            
            # 发布状态更新
            self.client.publish(self.status_topic, json.dumps({
                "status": "command_sent",
                "command": command_data
            }))
            
            return True
            
        except Exception as e:
            logger.error(f"发送到Arduino失败: {e}")
            return False
    
    def command_processor(self):
        """命令处理线程"""
        while self.running:
            try:
                if not self.command_queue.empty():
                    command = self.command_queue.get(timeout=1)
                    self.send_to_arduino(command)
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"命令处理错误: {e}")
    
    def serial_reader(self):
        """串口读取线程"""
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    response = self.ser.readline().decode('utf-8').strip()
                    if response:
                        logger.info(f"Arduino响应: {response}")
                        # 发布Arduino响应
                        self.client.publish(self.status_topic, json.dumps({
                            "status": "arduino_response",
                            "response": response
                        }))
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"串口读取错误: {e}")
    
    def start(self):
        """启动桥接服务"""
        self.running = True
        
        # 初始化串口
        if not self.initialize_serial():
            logger.error("串口初始化失败，退出程序")
            return
        
        # 连接MQTT
        try:
            self.client.connect(self.mqtt_broker, self.mqtt_port, 60)
        except Exception as e:
            logger.error(f"MQTT连接失败: {e}")
            return
        
        # 启动线程
        command_thread = threading.Thread(target=self.command_processor)
        serial_thread = threading.Thread(target=self.serial_reader)
        
        command_thread.daemon = True
        serial_thread.daemon = True
        
        command_thread.start()
        serial_thread.start()
        
        logger.info("MQTT-串口桥接服务启动成功")
        
        try:
            # 启动MQTT循环
            self.client.loop_forever()
        except KeyboardInterrupt:
            logger.info("收到停止信号")
        finally:
            self.stop()
    
    def stop(self):
        """停止桥接服务"""
        self.running = False
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("串口连接已关闭")
        
        self.client.disconnect()
        logger.info("MQTT连接已断开")

if __name__ == "__main__":
    bridge = MQTTSerialBridge()
    bridge.start()
