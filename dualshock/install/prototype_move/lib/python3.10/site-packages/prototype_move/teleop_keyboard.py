import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'nuri_vel', 10)
        self.twist = Twist()
        self.get_logger().info('Use WASD keys to move the robot')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key == 'w':
                    self.twist.linear.x += 0.1
                    self.twist.angular.z = 0.0
                elif key == 's':
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                elif key == 'a':
                    self.twist.angular.z += 0.1
                elif key == 'd':
                    self.twist.angular.z -= 0.1
                elif key == 'x':
                    self.twist.linear.x -= 0.1
                    self.twist.angular.z = 0.0
                elif key == 'q':
                    print("break")
                    break
                else:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                print(f"Linear: {self.twist.linear.x}, Angular: {self.twist.angular.z}")
                self.publisher_.publish(self.twist)

        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
