#ros2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

# mavproxy
import subprocess
from io import TextIOWrapper


# hold mavproxy
class Mav:
    def __init__(self):
        self.proc = subprocess.Popen(
                                    # 'python3 repeater.py',
                                    'mavproxy.py',
                                    shell=True,
                                    stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT
    )
        self.stdin = TextIOWrapper(
            self.proc.stdin,
            encoding='utf-8',
            write_through=True,
            line_buffering=True
        )
        self.stdout = TextIOWrapper(
            self.proc.stdout,
            encoding='utf-8',
            write_through=True
        )
        # self.stderr = TextIOWrapper(
        #     self.proc.stderr,
        #     encoding='utf-8',
        #     write_through=True
        # )


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
    #     self.timer = self.create_timer(1, self.read_callback)

    # def read_callback(self):
    #     # return
    #     # print(1)
    #     line = 'rr{}\n'.format(1)
    #     print(type(line))
    #     # self.mav.stdin.write(line)
    #     print(type('test'))
    #     self.mav.stdin.write('test\n')
        # self.mav.stdin.flush()

        # while rclpy.ok():
            #activates on newline
            # output = self.mav.stdout.readline()

            # if not not output:
            #     print(output)
            #     self.publish_msg(output)
        self.subscription = self.create_subscription(
                                                     String,
                                                     'mavros2',
                                                     self.incomming_msg,
                                                     10)
        self.subscription  # prevent unused variable warning

    def incomming_msg(self, msg):
        print('I heard: "%s"' % msg.data)
        # print(type(msg.data))
        self.mav.stdin.write(msg.data+'\n')
        self.mav.stdin.flush()

def main(args=None):
    #TODO init parameter
    # print(args)
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