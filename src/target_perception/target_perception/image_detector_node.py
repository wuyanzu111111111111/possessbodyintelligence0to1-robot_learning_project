import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from target_interfaces.msg import TargetInfo
from cv_bridge import CvBridge
import cv2
import numpy as np