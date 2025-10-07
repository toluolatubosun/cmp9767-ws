import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds - faster for better control
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Square movement state variables
        self.state = 'forward'  # 'forward' or 'turning'
        self.side_count = 0  # tracks which side of square (0-3)
        self.state_timer = 0  # tracks time in current state
        self.forward_duration = 30  # ~3 seconds forward (30 * 0.1s)
        self.turn_duration = 16   # ~1.6 seconds for 90-degree turn (16 * 0.1s)

    def timer_callback(self):
        msg = Twist()
        
        if self.state == 'forward':
            # Move forward
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.state_timer += 1
            
            if self.state_timer >= self.forward_duration:
                # Switch to turning
                self.state = 'turning'
                self.state_timer = 0
                self.get_logger().info(f'Completed side {self.side_count + 1}, now turning')
                
        elif self.state == 'turning':
            # Turn 90 degrees left to form square corners
            msg.linear.x = 0.0  # No forward movement
            msg.linear.y = 0.0  # No sideways movement  
            msg.angular.z = 1.0  # 90 degrees over ~1.6 seconds
            self.state_timer += 1
            
            if self.state_timer >= self.turn_duration:
                # Switch back to forward
                self.state = 'forward'
                self.state_timer = 0
                self.side_count += 1
                
                if self.side_count >= 4:
                    # Completed square, reset
                    self.side_count = 0
                    self.get_logger().info('Square completed! Starting new square.')
                else:
                    self.get_logger().info(f'Turn completed, starting side {self.side_count + 1}')
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()