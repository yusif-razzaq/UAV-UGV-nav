import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(Point, '/waypoints', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoint)
        self.counter = 0

    def publish_waypoint(self):
        waypoint = Point()
        waypoint.x = 1.0  # Replace with your desired x coordinate
        waypoint.y = 2.0  # Replace with your desired y coordinate
        waypoint.z = 0.0  # Replace with your desired z coordinate

        self.get_logger().info(f"Publishing waypoint: {waypoint.x}, {waypoint.y}, {waypoint.z}")
        self.publisher_.publish(waypoint)

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
