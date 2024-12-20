#!python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

rosNode= None

def scan_callback(scanMsg):
    global rosNode
    #print(f"Nombre de ranges : {len(scanMsg.ranges)}")
    #rosNode.get_logger().info( f"scan:\n{scanMsg.header}")

    # détection obstacles
    obstacles= []
    angle= scanMsg.angle_min
    for aDistance in scanMsg.ranges :
        if 0.1 < aDistance and aDistance < 5.0 :
            aPoint= [
                math.cos(angle) * aDistance,
                math.sin(angle) * aDistance
            ]
            
            obstacles.append(aPoint)
            
        angle+= scanMsg.angle_increment

    sample= [ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[10:20] ]
    rosNode.get_logger().info( f" obs({len(obstacles)}) ...{sample}..." )
    print(sample)

rclpy.init()
rosNode= Node('scan_interpreter')
rosNode.create_subscription( LaserScan, 'scan', scan_callback, 10)



while True :
    rclpy.spin_once( rosNode )
scanInterpret.destroy_node()
rclpy.shutdown()