import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import math

class IMUHeading:
    def __init__(self):
        self.sub = rospy.Subscriber('/imu/heading', Float32, self.imu_callback)
        self.sub = rospy.Subscriber('/obst/detected', Bool, self.detected_callback)
        self.pub = rospy.Publisher('/wpt/cmd_vel', Float32MultiArray, queue_size = 1)
        self.heading = 0
        self.object_detected = False
        self.turn_array = [0]*11
        self.vel_array = [0]*11
        self.previous_heading = self.heading
        self.updated_heading = False
        self.threshold = .2
        self.difference = 0
    def imu_callback(self, scan):
        if(not math.isnan(scan.data)):
            self.heading = -scan.data
    def detected_callback(self, scan):
        self.object_detected = scan.data
        #print self.object_detected
    def run(self):
        if(self.object_detected):
            if(not self.updated_heading):
                self.previous_heading = self.heading
                self.updated_heading = True
        else:
            self.difference = self.previous_heading - self.heading
            self.updated_heading = False
            if(abs(self.difference) < self.threshold):
                self.previous_heading = self.heading
                self.updated_heading = False
        self.turn_array = [0]*11
        index = int(self.difference * 7/(math.pi/6))
        if index < -4:
            index = -5
        if index > 4:
            index = 5
        print 'test'
        self.turn_array[5 - index] = .02
        msg = Float32MultiArray()
        vel_array = [0]*11
        total_array = vel_array
        total_array.extend(self.turn_array)
        msg.data = total_array
        self.pub.publish(msg)

if __name__ == '__main__':
   #print 'start'
    rospy.init_node('heading')
    imuHeading = IMUHeading()
    r = rospy.Rate(50)
    while not(rospy.is_shutdown()):
        imuHeading.run()
        r.sleep()