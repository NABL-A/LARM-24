import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class ObjectTfBroadcaster(Node):
    def __init__(self):
        super().__init__('object_tf_broadcaster')

        # Crée un broadcaster de transformation
        self.tf_broadcaster = TransformBroadcaster(self)

        # Exemple de coordonnées d'objet détecté (en mètres)
        self.object_position = [1.0, 0.5, 0.3]  # x, y, z dans le repère caméra

        # Timer pour publier périodiquement la transformation
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def broadcast_transform(self):
        # Créer un TransformStamped
        t = TransformStamped()

        # Remplir les informations de transformation
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_frame'  # Nom du frame parent
        t.child_frame_id = 'object_frame'  # Nom du frame enfant

        # Position (en mètres)
        t.transform.translation.x = self.object_position[0]
        t.transform.translation.y = self.object_position[1]
        t.transform.translation.z = self.object_position[2]

        # Orientation (aucune rotation, quaternion identité)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publier la transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
