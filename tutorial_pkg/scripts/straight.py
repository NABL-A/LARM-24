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
            2, self.control_callback
        )

    def scan_callback(self, scanMsg ):
        #self._logger.info( '> get scan' )

        angle = scanMsg.angle_min
        obstacles_gauche = []
        obstacles_droite = []

        for aDistance in scanMsg.ranges:
            if not math.isinf(aDistance) and aDistance > 0.1:  
                if angle > -1.57 and angle < 0 and aDistance < 0.5:
                    aPoint = [
                        math.cos(angle) * aDistance,
                        math.sin(angle) * aDistance
                    ]
                    
                    aPointCurrent = Point32()  
                    aPointCurrent.x = aPoint[0]
                    aPointCurrent.y = aPoint[1]
                    aPointCurrent.z = 0.0  

                    obstacles_gauche.append(aPointCurrent)
                    
                if angle < 1.57 and angle > 0 and aDistance < 0.5:
                    aPoint = [
                        math.cos(angle) * aDistance,
                        math.sin(angle) * aDistance
                    ]
                    
                    aPointCurrent = Point32()  
                    aPointCurrent.x = aPoint[0]
                    aPointCurrent.y = aPoint[1]
                    aPointCurrent.z = 0.0  

                    obstacles_droite.append(aPointCurrent)
                
            angle += scanMsg.angle_increment

        print(f"Obstacles left : {obstacles_gauche}")
        print(f"Obstacles right : {obstacles_droite}")


        if len(obstacles_gauche) != 0 :
            self.obstacle_left = True
            print("Oh ptn à gauche !")
        else :
            self.obstacle_left = False
            print("Rien à gauche !")


        if len(obstacles_droite) != 0 :
            self.obstacle_right = True  
            print("Attention à droite !")
        else :
            self.obstacle_right = False
            print("Rien à droite !")


    def control_callback(self):
        self._logger.info( '< define control' )

# Go:
if __name__ == '__main__' :
    main()
