#ros2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

# mavproxy
import subprocess
import io
from time import sleep


# hold mavproxy
class Mav:
    def __init__(self):
        self.proc = subprocess.Popen(
                                    'mavproxy.py',
                                    shell=True,
                                    stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE
    )
        self.stdin = io.TextIOWrapper(
            self.proc.stdin,
            encoding='utf-8',
            line_buffering=True,  # send data on newline
        )
        self.stdout = io.TextIOWrapper(
            self.proc.stdout,
            encoding='utf-8',
        )
        self.stderr = io.TextIOWrapper(
            self.proc.stderr,
            encoding='utf-8',
        )


class Mavros2Read(Node):
    def __init__(self, mav, name=None):
        super().__init__('mavros2_read')
        self.node = rclpy.create_node(name or type(self).__name__)

        self.mav = mav

        self.publisher = self.node.create_publisher(String, 'mavros2_output', 10)
        self.publisher
        
        self.timer = self.create_timer(0.01, self.read_callback)

    def read_callback(self):
        # while rclpy.ok():
            #activates on newline
            output = self.mav.stdout.readline()

            if not not output:
                print(output)
                self.publish_msg(output)

        # subscribe

    def publish_msg(self, message):
        msg = String()
        if msg is None:
            msg.data = 'empty message'
        else:
            # print(message)
            msg.data = message
            self.publisher.publish(msg)
                

class Mavros2Write(Node):
    def __init__(self, mav, name=None):
        super().__init__('mavros2_write')
        self.node = rclpy.create_node(name or type(self).__name__)

        self.mav = mav

        self.subscription = self.create_subscription(
                                                          String,
                                                          'mavros2',
                                                          self.incomming_msg,
                                                          10)
        self.subscription  # prevent unused variable warning

    def incomming_msg(self, msg):
        print('I heard: "%s"' % msg.data)

def main(args=None):
    mav = Mav()
    rclpy.init(args=args)
    mavros2_read = Mavros2Read(mav)
    mavros2_write = Mavros2Write(mav)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(mavros2_read)
    executor.add_node(mavros2_write)
    executor.spin()
    executor.shutdown()
    mavros2_read.destroy_node()
    mavros2_write.destroy_node()

if __name__ == '__main__':
    main()