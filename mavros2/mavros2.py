''' ros2 wrapper for mavlink using mavproxy'''
# mavproxy
from subprocess import Popen, PIPE, STDOUT
from io import TextIOWrapper
from multiprocessing import Process, Queue
from memory_tempfile import MemoryTempfile

#ros2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
#messages
from std_msgs.msg import String, Int8
from geographic_msgs.msg import GeoPath


class Mav:
    ''' run mavproxy.py in encapsulated shell'''
    def __init__(self):
        self.launch()
        self.setup_tempfile()
        self.booted = False
        #store commands to be send to mavproxy
        self.q = Queue()
        self.tempfile = None

    def launch(self):
        '''start mavproxy in Popen'''
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
            line_buffering=True
            )
        self.stdout = TextIOWrapper(
            self.proc.stdout,
            encoding='utf-8',
            )

    def relaunch(self):
        '''in case mavproxy terminates'''
        self.proc.kill()
        self.proc.wait()
        self.launch()

    def setup_tempfile(self):
        '''store readable waypoints in non disc file'''
        tempfile_ = MemoryTempfile().NamedTemporaryFile(delete=False)
        self.tempfile = tempfile_.name
        tempfile_.close()


def write_string(mav):
    '''send message from queue tu mavproxy(threadsave)'''
    while True:
        mav.stdin.write(mav.q.get() + '\n')
        print('send')


class Mavros2Read(Node):
    '''processing mavproxy output, publishing relevant imformation'''
    def __init__(self, mav, name=None):
        super().__init__('mavros2_read')
        self.node = rclpy.create_node(name or type(self).__name__)
        self.mav = mav
        self.publisher = self.node.create_publisher(String, 'mavros2_output', 10)

        self.timer0 = self.create_timer(0.01, self.read_callback)

    def read_callback(self):
        '''blocks till message arrives'''
        output = self.mav.stdout.readline()
        print(output)
        # check if mav is still running, if not empty output and restart
        if self.mav.proc.poll() is not None and len(output) == 0:
            print('relaunch')
            self.mav.relaunch()
        else:
            print(output[4:16])


class Mavros2RequestStatus(Node):
    '''requests statusoutput every x seconds'''
    def __init__(self, mav, name=None):
        super().__init__('mavros2_write')
        self.node = rclpy.create_node(name or type(self).__name__)
        self.mav = mav
        self.timer = self.create_timer(1, self.request_status)

    def request_status(self):
        '''adds status request to input queue'''
        self.mav.q.put('status')


class Mavros2Write(Node):
    '''adds incomming messages to queue to be send to mavproxy'''
    def __init__(self, mav, name=None):
        super().__init__('mavros2_write')
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
            'mavros2_set_mode',
            self.set_mode,
            10
            )

        self.geopath_in = self.create_subscription(
            GeoPath,
            'mavros2_set_geopath',
            self.set_geopath,
            10
            )

        self.speed_in = self.create_subscription(
            Int8,
            'mavros2_set_speed',
            self.set_speed,
            10
            )

        self.set_takeoff_altitude = self.create_subscription(
            Int8,
            'mavros2_set_takeoff_altitude',
            self.set_takeoff_alti,
            10
            )
        self.set_engine = self.create_subscription(
            String,
            'mavros2_set_engine_mode',
            self.set_engine_mode,
            10
            )

    def set_engine_mode(self, msg):
        '''prepares and adds incoming message to input queue'''
        eng = 'engine {}'.format(msg.data)
        self.mav.q.put(eng)

    def set_takeoff_alti(self, msg):
        '''prepares and adds incoming message to input queue'''
        alt = 'takeoff {}'.format(msg)
        self.mav.write_string(self, alt)

    def set_speed(self, msg):
        '''prepares and adds incoming message to input queue'''
        speed = 'setspeed {}'.format(msg)
        self.mav.q.put(speed)

    def set_mode(self, msg):
        '''prepares and adds incoming message to input queue'''
        mode = 'mode {}'.format(msg.data)
        self.mav.q.put(mode)

    def incomming_msg(self, msg):
        '''adds incoming message to input queue'''
        self.mav.q.put(msg.data)

    def set_geopath(self, msg):
        '''prepares and adds incoming message to input queue'''
        # create path as txt and load
        plan = open(self.mav.tempfile, 'w')
        for i, line in msg.poses:
            if i == 0:
                plan.write('QGC WPL 1\n{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(
                    i, 1, 0, 0, 0, 0, 0, 0,
                    line.pose.position.latitude,
                    line.pose.position.longitude,
                    line.pose.position.latitude,
                    line.pose.position.altitude,
                    1))
            else:
                plan.write('QGC WPL 1\n{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(
                    i, 0, 0, 0, 0, 0, 0, 0,
                    line.pose.position.latitude,
                    line.pose.position.longitude,
                    line.pose.position.latitude,
                    line.pose.position.altitude,
                    1))
        plan.close()
        self.mav.q.put('wp load {}').format(self.mav.tempfile)


def main(args=None):
    '''start interaction with mavproxy'''
    #TODO init parameter
    # print(args)
    mav = Mav()

    Process(target=write_string, args=(mav,)).start()

    rclpy.init(args=args)
    mavros2_read = Mavros2Read(mav)
    mavros2_write = Mavros2Write(mav)
    mavros2_request_status = Mavros2RequestStatus(mav)
    executor = MultiThreadedExecutor()
    executor.add_node(mavros2_read)
    executor.add_node(mavros2_write)
    executor.add_node(mavros2_request_status)
    executor.spin()
    executor.shutdown()
    mavros2_read.destroy_node()
    mavros2_write.destroy_node()

if __name__ == '__main__':
    main()
