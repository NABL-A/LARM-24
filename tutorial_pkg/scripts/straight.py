#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

# Ros Node process:
def main():
    # Initialize ROS and a ROS node
    rclpy.init()
    rosNode = Node( "basic_move" )

    # Initialize our control:
    control= StraightCtrl()
    control.initializeRosNode( rosNode )

    # infinite Loop:
    rclpy.spin( rosNode )

    # clean end
    rosNode.destroy_node()
    rclpy.shutdown()

# Ros Node Class:
class StraightCtrl :
    def initializeRosNode(self, rosNode ):
        # Get logger from the node:
        self._logger= rosNode.get_logger()
        self.obstacle_left = False
        self.obstacle_right = False

        # Initialize publisher:
        self._pubVelocity= rosNode.create_publisher(
            Twist, '/multi/cmd_nav', 10
        )

        # Initialize scan callback:
        self._subToScan= rosNode.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10
        )

        # Initialize control callback:
        self._timForCtrl= rosNode.create_timer(
            0.05, self.control_callback
        )

    def scan_callback(self, scanMsg ):
        self._logger.info( '> get scan' )

        angle = scanMsg.angle_min
        obstacles = []

        for aDistance in scanMsg.ranges:
            if angle > scanMsg.angle_min and angle < 0 :
            
            if angle < scanMsg.angle_max and angle > 0 :


            
            if 0.1 < aDistance < 5.0:  
                aPoint = [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance
                ]
                
                aPointCurrent = Point32()  
                aPointCurrent.x = aPoint[0]
                aPointCurrent.y = aPoint[1]
                aPointCurrent.z = 0.0  

                obstacles.append(aPointCurrent)
                
            angle += scanMsg.angle_increment

    def control_callback(self):
        self._logger.info( '< define control' )

# Go:
if __name__ == '__main__' :
    main()
