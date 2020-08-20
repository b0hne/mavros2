#ros2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
#messages
from std_msgs.msg import String
from geographic_msgs.msg import GeoPath
# mavproxy
from subprocess import Popen, PIPE, STDOUT
from io import TextIOWrapper
from memory_tempfile import MemoryTempfile

# run mavproxy.py in encapsulated shell
class Mav:
    def __init__(self):
        self.launch()
        self.setup_tempfile()

    def launch(self):
        self.proc = Popen(
                                    'mavproxy.py',
                                    shell=True,
                                    stdin=PIPE,
                                    stdout=PIPE,
                                    stderr=STDOUT
                                    )
        self.stdin = TextIOWrapper(
            self.proc.stdin,
            encoding='utf-8',
            # write_through=True,
            line_buffering=True
            )
        self.stdout = TextIOWrapper(
            self.proc.stdout,
            encoding='utf-8',
            # write_through=True
            )

    def relaunch(self):
        self.proc.kill()
        self.proc.wait()
        self.launch()

    def setup_tempfile(self):
        tempfile_ = MemoryTempfile().NamedTemporaryFile(delete=False)
        self.tempfile = tempfile_.name
        tempfile_.close()



class Mavros2Read(Node):
    def __init__(self, mav, name=None):
        super().__init__('mavros2_read')
        self.node = rclpy.create_node(name or type(self).__name__)
        self.mav = mav
        self.publisher = self.node.create_publisher(String, 'mavros2_output', 10)
        
        self.timer = self.create_timer(0.01, self.read_callback)

    def read_callback(self):
        #blocks till message arrives
        output = self.mav.stdout.readline()
        print(output)
        # check if mav is still running, if not empty output and restart
        if self.mav.proc.poll() is None or len(output) > 0: 
            msg = String()
            msg.data = output
            self.publisher.publish(msg)
        else:
            print('relaunch')
            self.mav.relaunch()


class Mavros2Write(Node):
    def __init__(self, mav, name=None):
        super().__init__('mavros2_write')
        self.node = rclpy.create_node(name or type(self).__name__)
        self.group = MutuallyExclusiveCallbackGroup()
        self.mav = mav
        self.generic_input = self.create_subscription(
                                                      String,
                                                      'mavros2_generic_input',
                                                      self.incomming_msg,
                                                      10,
                                                      callback_group=self.group
                                                      )

        self.mode_in = self.create_subscription(
                                                String,
                                                'mavros2_mode_in',
                                                self.incomming_mode,
                                                10,
                                                    callback_group=self.group
                                                )

        self.geopath_in = self.create_subscription(
                                                   GeoPath,
                                                   'mavros2_geopath_in',
                                                   self.incomming_geopath,
                                                   10,
                                                   callback_group=self.group
                                                   )

    def write_string(self, msg):
            self.mav.stdin.write(msg+'\n')


    def incomming_mode(self, msg):
        mode = 'mode {}'.format(msg.data)
        self.write_string(mode)
                
    def incomming_msg(self, msg):
        self.write_string(msg.data)
    
    def incomming_geopath(self, msg):
        # create path as txt and load
        self.mav.tempfile
        plan = open(self.mav.tempfile, 'w')
        for i, line in msg.poses:
            if i == 0:
                plan.write('QGC WPL 1\n{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(
                                            i, 1, 0, 0, 0, 0, 0, 0, line.pose.position.latitude, line.pose.position.longitude, line.pose.position.latitude, line.pose.position.altitude, 1))
            else:
                plan.write('QGC WPL 1\n{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(
                                            i, 0, 0, 0, 0, 0, 0, 0, line.pose.position.latitude, line.pose.position.longitude, line.pose.position.latitude, line.pose.position.altitude, 1))
        plan.close()
        self.write_string('wp load {}').format(self.mav.tempfile)
    


def main(args=None):
    #TODO init parameter
    # print(args)
    mav = Mav()
    rclpy.init(args=args)
    mavros2_read = Mavros2Read(mav)
    mavros2_write = Mavros2Write(mav)
    executor = MultiThreadedExecutor()
    executor.add_node(mavros2_read)
    executor.add_node(mavros2_write)
    executor.spin()
    executor.shutdown()
    mavros2_read.destroy_node()
    mavros2_write.destroy_node()

if __name__ == '__main__':
    main()