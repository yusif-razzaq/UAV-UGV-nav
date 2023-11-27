import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped

class WaypointSubscriber(Node):
    def __init__(self):
        super().__init__('waypoint_subscriber')
        self.subscription = self.create_subscription(
            Point,
            '/waypoints',
            self.waypoint_callback,
            10  # Adjust the queue size if needed
        )
        self.subscription  # Prevent unused variable warning
        self.nav_goal_publisher = self.create_publisher(PoseStamped, '/goal', 10)
        
    def waypoint_callback(self, msg):
        self.get_logger().info(f"Received waypoint: {msg.x}, {msg.y}, {msg.z}")

        # Convert Point message to PoseStamped message
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = "map"  # Adjust the frame ID if needed
        nav_goal.pose.position.x = msg.x
        nav_goal.pose.position.y = msg.y
        nav_goal.pose.position.z = msg.z

        # Assuming no orientation information is provided, you might set a default orientation
        nav_goal.pose.orientation.w = 1.0  # For instance, setting a default orientation

        # Publish the navigation goal
        self.nav_goal_publisher.publish(nav_goal)
        self.get_logger().info("Sent navigation goal to Nav2")

def main(args=None):
    rclpy.init(args=args)
    waypoint_subscriber = WaypointSubscriber()
    rclpy.spin(waypoint_subscriber)
    waypoint_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
