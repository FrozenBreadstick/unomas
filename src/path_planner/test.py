import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from object_detection_msgs.msg import ObjectPositionArray
from std_msgs.msg import Bool


class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__("object_avoidance_node")

        
        self.create_subscription(
            ObjectPositionArray,
            "detected_objects",
            self.detection_callback,
            10
        )

        #
        self.create_subscription(
            LaserScan,
            "scan",
            self.lidar_callback,
            10
        )

        # Publisher for the avoidance decision
        self.alert_pub = self.create_publisher(Bool, "avoidance_alert", 10)

        self.object_in_frame = False
        self.close_object = False

        self.distance_threshold = 2.0  # meters

    def detection_callback(self, msg: ObjectPositionArray):
        
        self.object_in_frame = len(msg.objects) > 0
        self.evaluate_condition()

    def lidar_callback(self, msg: LaserScan):
        
        valid_ranges = [r for r in msg.ranges if r > 0.01]  # filter invalid returns
        
        if len(valid_ranges) > 0:
            closest_distance = min(valid_ranges)
            self.close_object = closest_distance < self.distance_threshold
        else:
            self.close_object = False

        self.evaluate_condition()

    def evaluate_condition(self):
     
        alert_msg = Bool()
        alert_msg.data = self.object_in_frame and self.close_object
        self.alert_pub.publish(alert_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
