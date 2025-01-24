#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import WheelDropEvent
from kobuki_ros_interfaces.msg import BumperEvent

def main():
    rclpy.init()
    rosNode = Node("basic_move")

    control = StraightCtrl()
    control.initializeRosNode(rosNode)

    rclpy.spin(rosNode)

    rosNode.destroy_node()
    rclpy.shutdown()


class StraightCtrl:
    def initializeRosNode(self, rosNode):
        self._logger = rosNode.get_logger()
        self.obstacle_left = False
        self.obstacle_right = False
        self.obstacle_middle = False
        self.trop_proche = False
        self.min_distance_middle = float('inf')
        self.wheel_left_dropped = False
        self.wheel_right_dropped = False

        self._pubVelocity = rosNode.create_publisher(
            Twist, '/multi/cmd_nav', 10
        )

        self._subToScan = rosNode.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10
        )

        self._subToWheelDrop = rosNode.create_subscription(
            WheelDropEvent, '/events/wheel_drop',
            self.wheel_drop_callback, 10
        )

        self._subToBumper = rosNode.create_subscription(
            BumperEvent, '/events/bumper',
            self.contact_callback, 10
        )

        self._timForCtrl = rosNode.create_timer(
            0.1, self.control_callback
        )

    def scan_callback(self, scanMsg):
        angle = scanMsg.angle_min
        self.trop_proche = False
        self.min_distance_middle = float('inf')
        self.cote="middle"
        self.min=float('inf')

        obstacles_gauche = []
        obstacles_droite = []
        obstacles_centre = []

        for aDistance in scanMsg.ranges:
            if not math.isinf(aDistance) and scanMsg.range_min < aDistance < scanMsg.range_max:
                # Classify points into zones
                if angle > 0.9:
                    obstacles_gauche.append(aDistance)
                    if aDistance < self.min:
                        self.min=aDistance
                        self.cote="gauche"
                elif angle < -0.9:
                    obstacles_droite.append(aDistance)
                    if aDistance < self.min:
                        self.min=aDistance
                        self.cote="droite"
                elif -0.9 <= angle <= 0.9:
                    obstacles_centre.append(aDistance)
                    self.min_distance_middle = min(self.min_distance_middle, aDistance)

            angle += scanMsg.angle_increment

        # Update obstacle detection
        self.obstacle_left = len(obstacles_gauche) > 0 and min(obstacles_gauche, default=float('inf')) < 0.5
        self.obstacle_right = len(obstacles_droite) > 0 and min(obstacles_droite, default=float('inf')) < 0.5
        self.obstacle_middle = len(obstacles_centre) > 0 and self.min_distance_middle < 0.5

        # Update "trop proche"
        self.trop_proche = self.min_distance_middle < 0.2

    def contact_callback(self, contact):
        if contact.state == BumperEvent.PRESSED:
            self._logger.warning("Collision detected! Stopping.")
            twist = Twist()
            self._pubVelocity.publish(twist)
            quit()

    def wheel_drop_callback(self, msg):
        if msg.wheel == WheelDropEvent.LEFT:
            self.wheel_left_dropped = msg.state == WheelDropEvent.DROPPED
        elif msg.wheel == WheelDropEvent.RIGHT:
            self.wheel_right_dropped = msg.state == WheelDropEvent.DROPPED

        self._logger.info(
            f"Wheel drop event: LEFT={self.wheel_left_dropped}, RIGHT={self.wheel_right_dropped}"
        )

    def control_callback(self):
        if self.wheel_left_dropped or self.wheel_right_dropped:
            self._logger.warning("Wheel dropped! Stopping.")
            twist = Twist()
            self._pubVelocity.publish(twist)
            quit()
            return

        twist = Twist()

        if self.trop_proche and self.cote =="droite":
            twist.linear.x = 0.0
            twist.angular.z = 2.0  
        
        if self.trop_proche and self.cote =="gauche":
            twist.linear.x = 0.0
            twist.angular.z = -2.0

        elif not self.obstacle_middle:
            twist.linear.x = 0.5
            twist.angular.z = 0.0

        elif self.obstacle_middle :

            if self.obstacle_left and not self.obstacle_right:
                twist.linear.x = 0.2
                twist.angular.z = -2.0

            elif self.obstacle_right and not self.obstacle_left:
                twist.linear.x = 0.2
                twist.angular.z = 2.0
            
            elif self.cote=="droite":
                twist.linear.x = 0.0
                twist.angular.z = 2.0
            
            elif self.cote=="gauche":
                twist.linear.x = 0.0
                twist.angular.z = -2.0

        self._pubVelocity.publish(twist)


if __name__ == '__main__':
    main()
