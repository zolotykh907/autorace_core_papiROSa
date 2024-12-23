import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class LineDetector(Node):
    def __init__(self):
        super().__init__('LineDetector')

        # Publishers
        self.center_lane_pub = self.create_publisher(Float64, '/center_line', 1)
        self.image_pub = self.create_publisher(Image, '/color/detectline', 1)  # Публикация изображения с отрезком
        # Subscribers
        self.img_proj_sub = self.create_subscription(Image, '/color/image_projected', self.image_processing, 1)
        self.cv_bridge = CvBridge()

    def image_processing(self, msg):
        # Считывание изображения и перевод в HSV
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Центры линий
        centers_x = []
        # Маска желтой линии
        yellow_mask = cv2.inRange(hsv_image, (20, 100, 100), (30, 255, 255))
        yellow_mask = cv2.blur(yellow_mask, (3, 3))
        yellow_mask[yellow_mask != 0] = 255
        # Маска белой линии
        white_mask = cv2.inRange(hsv_image, (0, 0, 230), (255, 0, 255))
        white_mask = cv2.blur(white_mask, (3, 3))
        white_mask[white_mask != 0] = 255
        # Определение центров линий
        M_yellow = cv2.moments(yellow_mask, binaryImage=True)
        M_white = cv2.moments(white_mask, binaryImage=True)
        yellow_center_x = 0 if M_yellow['m00'] == 0 else M_yellow['m10'] // M_yellow['m00']
        white_center_x = hsv_image.shape[1] if M_white['m00'] == 0 else M_white['m10'] // M_white['m00']

        # Пропускать те случаи, когда желтая не левее белой
        if white_center_x < yellow_center_x:
            white_center_x = image.shape[1]

        centers_x.append(yellow_center_x)
        centers_x.append(white_center_x)

        # Публикация центра между линиями
        if len(centers_x) == 2:  # Убедимся, что найдены обе линии
            yellow_center_x, white_center_x = centers_x
            center_lane = (yellow_center_x + white_center_x) / 2  # Середина между линиями
            self.center_lane_pub.publish(Float64(data=center_lane))
            # Рисуем точки
            cv2.circle(image, (int(yellow_center_x), image.shape[0] // 2), 5, (0, 255, 255), -1)  # Желтая точка
            cv2.circle(image, (int(white_center_x), image.shape[0] // 2), 5, (255, 255, 0), -1)  # Белая точка
            cv2.circle(image, (int(center_lane), image.shape[0] // 2), 5, (0, 0, 255), -1)  # Середина
            # Рисуем отрезок между точками (желтая -> середина -> белая)
            cv2.line(image, (int(yellow_center_x), image.shape[0] // 2), (int(center_lane), image.shape[0] // 2), (0, 255, 0), 2)  # Желтая -> Середина
            cv2.line(image, (int(center_lane), image.shape[0] // 2), (int(white_center_x), image.shape[0] // 2), (0, 255, 0), 2)  # Середина -> Белая
            # Публикация изображения с нарисованными точками и отрезками
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(image, msg.encoding))
        self.center_lane_pub.publish(Float64(data = np.sum(centers_x) / len(centers_x)))



def main():
    rclpy.init()
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
