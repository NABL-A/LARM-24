import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

rosNode = None
aPublisher = None

def scan_callback(scanMsg):
    global rosNode, aPublisher
    obstacles = []
    angle = scanMsg.angle_min

    for aDistance in scanMsg.ranges:
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

    aPointCloud = PointCloud()
    aPointCloud.header = scanMsg.header  
    aPointCloud.points = obstacles  

    rosNode.get_logger().info(f"Nombre d'obstacles détectés : {len(obstacles)}")

    aPublisher.publish(aPointCloud)

def main():
    global rosNode, aPublisher
    rclpy.init()

    rosNode = Node('scan_interpreter')
    rosNode.create_subscription(LaserScan, 'scan', scan_callback, 10)
    aPublisher = rosNode.create_publisher(PointCloud, 'scan_results', 10)

    try:
        rclpy.spin(rosNode)
    except KeyboardInterrupt:
        pass

    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
