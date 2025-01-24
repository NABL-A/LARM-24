#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

print("test_move :: START...")

def oneTalk():
    print("C'est bon HAHA!")
    # Initialize ROS client
    rclpy.init()
    # Create a node
    aNode= Node( "simpleTalker" )
    # Attach a publisher to the node, with a specific type, the name of the topic, a history depth
    aPublisher= aNode.create_publisher( Twist, '/commands/velocity', 10 )
    # Create a message to send
    msg1 = Twist()
    msg2 = Twist()
    
    msg1.linear.x = 0.5
    msg1.angular.z = 0.0
    
    # Add the message to the list of messages to publish
    # Activate the ROS client with the node
    # (that will publish the message on testTopic topic)

    print("C'est bon 1")
    while 1 :
        for a in range(10) :
            aPublisher.publish(msg1)
            rclpy.spin_once(aNode, timeout_sec= 0.1)

        msg2.linear.x = 0.0
        msg2.angular.z = 0.78

        for a in range(25) :
            aPublisher.publish(msg2)
            rclpy.spin_once(aNode, timeout_sec=0.1)
    

    print("C'est bon 2")


    # Clean everything and switch the light off
    aNode.destroy_node()

    print("C'est bon !")
    rclpy.shutdown()
    

    

# Execute the function.
if __name__ == "__main__":
    oneTalk()

