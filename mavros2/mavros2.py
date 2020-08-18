#ros2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geographic_msgs.msg import GeoPath
# mavproxy
import subprocess
from multiprocessing import Lock


# run mavproxy.py in encapsulated shell
class Mav:
    def __init__(self):
        self.launch()
        #protect stdin from simultanious writes
        self.mutex = Lock()

    def launch(self):
        self.proc = subprocess.Popen(
                                    # 'ping 8.8.8.8',
                                    # 'python3 repeater.py',
                                    'mavproxy.py',
                                    shell=True,
                                    stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT,
                                    universal_newlines=True
                                    )
        self.stdin = self.proc.stdin
        self.stdout = self.proc.stdout

    def relaunch(self):
        self.proc.kill()
        self.proc.wait()
        self.launch()


class Mavros2Read(Node):
    def __init__(self, mav, name=None):
        super().__init__('mavros2_read')
        self.node = rclpy.create_node(name or type(self).__name__)
        self.mav = mav
        self.publisher = self.node.create_publisher(String, 'mavros2_output', 10)
        
        self.timer = self.create_timer(0.01, self.read_callback)

    def read_callback(self):
        output = self.mav.stdout.readline()
        # check if mav is still running, if not empty output and restart
        if self.mav.proc.poll() is None or len(output) > 0: 
            print(output)
            msg = String()
            msg.data = output
            self.publisher.publish(msg)
        else:
            self.mav.relaunch()


class Mavros2Write(Node):
    def __init__(self, mav, name=None):
        super().__init__('mavros2_write')
        self.id = 1
        self.node = rclpy.create_node(name or type(self).__name__)

        self.mav = mav
        self.generic_input = self.create_subscription(
                                                      String,
                                                      'mavros2_generic_input',
                                                      self.incomming_msg,
                                                      10
                                                      )

        self.mode_in = self.create_subscription(
                                                    String,
                                                    'mavros2_waypoint_in',
                                                    self.incomming_mode,
                                                    10
                                                    )

        self.geopath_in = self.create_subscription(
                                                    GeoPath,
                                                    'mavros2_geopath_in',
                                                    self.incomming_geopath,
                                                    10
                                                    )

    def write_string(self, message):
        #no simultanious writes
        self.mav.mutex.acquire()
        try:
            self.mav.stdin.write(message)
        finally:
            self.mav.mutex.release()


    def incomming_mode(self, msg):
        self.write_string('mode '+ msg.data)
                
    def incomming_msg(self, msg):
        self.write_string(msg.data)
    
    def incomming_geopath(self, msg):
        # create path as txt and load
        plan = open('path.txt', 'w')
        for i, line in msg.poses:
            if i == 0:
                plan.write('QGC WPL 1\n{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(
                                            i, 1, 0, 0, 0, 0, 0, 0, line.pose.position.latitude, line.pose.position.longitude, line.pose.position.latitude, line.pose.position.altitude, 1))
            else:
                plan.write('QGC WPL 1\n{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(
                                            i, 0, 0, 0, 0, 0, 0, 0, line.pose.position.latitude, line.pose.position.longitude, line.pose.position.latitude, line.pose.position.altitude, 1))
        plan.close()
        self.write_string('wp load path.txt')
    


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