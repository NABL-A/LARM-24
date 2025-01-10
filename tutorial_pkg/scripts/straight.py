#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from kobuki_ros_interfaces.msg import WheelDropEvent

def main():
    # Initialize ROS and a ROS node
    rclpy.init()
    rosNode = Node("basic_move")

    # Initialize our control:
    control = StraightCtrl()
    control.initializeRosNode(rosNode)

    # Infinite loop:
    rclpy.spin(rosNode)

    # Clean end
    rosNode.destroy_node()
    rclpy.shutdown()


# Ros Node Class:
class StraightCtrl:
    def initializeRosNode(self, rosNode):
        # Get logger from the node:
        self._logger = rosNode.get_logger()
        self.obstacle_left = False
        self.obstacle_right = False
        self.obstacle_middle = False
        self.wheel_left_dropped = False
        self.wheel_right_dropped = False

        # Initialize publisher:
        self._pubVelocity = rosNode.create_publisher(
            Twist, '/multi/cmd_nav', 10
        )

        # Initialize scan callback:
        self._subToScan = rosNode.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10
        )

        # Subscribe to WheelDropEvent:
        self._subToWheelDrop = rosNode.create_subscription(
            WheelDropEvent, '/events/wheel_drop',
            self.wheel_drop_callback, 10
        )

        # Initialize control callback:
        self._timForCtrl = rosNode.create_timer(
            0.1, self.control_callback
        )

    def scan_callback(self, scanMsg):
        angle = scanMsg.angle_min
        obstacles_gauche = []
        obstacles_droite = []
        obstacles_centre = []

        for aDistance in scanMsg.ranges:
            if not math.isinf(aDistance) and aDistance > 0.1:
                if angle > scanMsg.angle_min and angle < -0.9 and aDistance < 0.5:
                    aPoint = [
                        math.cos(angle) * aDistance,
                        math.sin(angle) * aDistance
                    ]
                    aPointCurrent = Point32()
                    aPointCurrent.x = aPoint[0]
                    aPointCurrent.y = aPoint[1]
                    obstacles_droite.append(aPointCurrent)

                if angle < scanMsg.angle_max and angle > 0.9 and aDistance < 0.3:
                    aPoint = [
                        math.cos(angle) * aDistance,
                        math.sin(angle) * aDistance
                    ]
                    aPointCurrent = Point32()
                    aPointCurrent.x = aPoint[0]
                    aPointCurrent.y = aPoint[1]
                    obstacles_gauche.append(aPointCurrent)

                if angle < 0.9 and angle > -0.9 and aDistance < 0.3:
                    aPoint = [
                        math.cos(angle) * aDistance,
                        math.sin(angle) * aDistance
                    ]
                    aPointCurrent = Point32()
                    aPointCurrent.x = aPoint[0]
                    aPointCurrent.y = aPoint[1]
                    obstacles_centre.append(aPointCurrent)

            angle += scanMsg.angle_increment

        self.obstacle_left = len(obstacles_gauche) != 0
        self.obstacle_right = len(obstacles_droite) != 0
        self.obstacle_middle = len(obstacles_centre) != 0

    def wheel_drop_callback(self, msg):
        # Update wheel drop state
        if msg.wheel == WheelDropEvent.LEFT:
            self.wheel_left_dropped = msg.state == WheelDropEvent.DROPPED
        elif msg.wheel == WheelDropEvent.RIGHT:
            self.wheel_right_dropped = msg.state == WheelDropEvent.DROPPED

        self._logger.info(
            f"Wheel drop event received: "
            f"LEFT={self.wheel_left_dropped}, RIGHT={self.wheel_right_dropped}"
        )

    def control_callback(self):
        if self.wheel_left_dropped or self.wheel_right_dropped:
            self._logger.warning("Wheel dropped! Stopping the robot.")
            twist = Twist()
            self._pubVelocity.publish(twist)
            return

        twist = Twist()
        if not self.obstacle_middle:
            twist.linear.x = 0.2
        elif self.obstacle_left and not self.obstacle_right:
            twist.angular.z = -0.5
        elif self.obstacle_right and not self.obstacle_left:
            twist.angular.z = 0.5
        else:
            twist.angular.z = 0.5

        self._pubVelocity.publish(twist)


# Go:
if __name__ == '__main__':
    main()