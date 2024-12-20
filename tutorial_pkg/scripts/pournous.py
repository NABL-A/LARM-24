#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

print("test_move :: START...")

def test_move():
    print("C'est bon HAHA!")

    rclpy.init()

    aNode= Node( "simpleTalker" )

    aPublisher= ROSTalker(aNode)

    aNode._timer = aNode.create_timer(2.0, aPublisher.timer_callback_linear)

    aNode._timer2 = aNode.create_timer(2.0, aPublisher.timer_callback_angular)
    

    rclpy.spin(aNode)    

    print("C'est bon 2")


    aNode.destroy_node()
    print("C'est bon !")
    rclpy.shutdown()
    

class ROSTalker:
    def __init__(self, rosNode):
        self._publisher= rosNode.create_publisher( Twist, '/commands/velocity', 10 )

    def timer_callback_linear(self):
        velocity = Twist()
        velocity.linear.x = 0.5
        velocity.angular.z = 0.0
        self._publisher.publish(velocity)
        time.sleep(2.0)
            
    def timer_callback_angular(self):
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = 0.78
        self._publisher.publish(velocity)
        time.sleep(2.0)

# Execute the function.
if __name__ == "__main__":
    test_move()

