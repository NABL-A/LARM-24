#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from kobuki_ros_interfaces.msg import WheelDropEvent


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
        self.obstacle_middle = False

        # Initialize publisher:
        self._pubVelocity= rosNode.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # Initialize scan callback:
        self._subToScan= rosNode.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10
        )

        # Initialize control callback:
        self._timForCtrl= rosNode.create_timer(
            0.1, self.control_callback
        )

    def scan_callback(self, scanMsg ):
        #self._logger.info( '> get scan' )

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
                    aPointCurrent.z = 0.0  

                    obstacles_droite.append(aPointCurrent)
                    
                if angle < scanMsg.angle_max and angle > 0.9 and aDistance < 0.3:
                    aPoint = [
                        math.cos(angle) * aDistance,
                        math.sin(angle) * aDistance
                    ]
                    
                    aPointCurrent = Point32()  
                    aPointCurrent.x = aPoint[0]
                    aPointCurrent.y = aPoint[1]
                    aPointCurrent.z = 0.0  

                    obstacles_gauche.append(aPointCurrent)
                
                if angle < 0.9 and angle > -0.9 and aDistance < 0.3:
                    aPoint = [
                        math.cos(angle) * aDistance,
                        math.sin(angle) * aDistance
                    ]
                    
                    aPointCurrent = Point32()  
                    aPointCurrent.x = aPoint[0]
                    aPointCurrent.y = aPoint[1]
                    aPointCurrent.z = 0.0  

                    obstacles_centre.append(aPointCurrent)

            angle += scanMsg.angle_increment

        print(f"Obstacles left : {obstacles_gauche}")
        print(f"Obstacles right : {obstacles_droite}")
        print(f"Obstacles middle (malcolm) : {obstacles_centre}")


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

        if len(obstacles_centre) != 0 :
            self.obstacle_middle = True  
            print("Devant toi nigaud !")
        else :
            self.obstacle_middle = False
            print("Allez y monsieur!")


    def control_callback(self):
        self._logger.info( '< define control' )
        
        if not self.obstacle_middle:
            twist = Twist()
            twist.linear.x = 0.2 
            self._pubVelocity.publish(twist)
        
        elif self.obstacle_left and not self.obstacle_right and self.obstacle_middle:
            twist = Twist()
            twist.angular.z = -0.5  
            self._pubVelocity.publish(twist)

        elif self.obstacle_right and not self.obstacle_left and self.obstacle_middle:
            twist = Twist()
            twist.angular.z = 0.5  
            self._pubVelocity.publish(twist)

        else:
            twist = Twist()
            twist.angular.z = 0.5  
            self._pubVelocity.publish(twist)
            

# Go:
if __name__ == '__main__' :
    main()
    
