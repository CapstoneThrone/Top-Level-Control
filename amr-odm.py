#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class OdomPublisher(object):
    def __init__(self):
        self.begin()


    def begin(self):
        self.odom = Odometry()
        rospy.init_node('amr_odm', anonymous=True)
        #odmPub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imuPub = rospy.Publisher('imu/test', Imu, queue_size=10)
        self.last_update = rospy.Time.now()
        rate = rospy.Rate(1.0)
        self.first_ax = None
        self.first_ay = None
        while not rospy.is_shutdown():
            rospy.Subscriber('/odom', Odometry, self.getOdometry)
            rospy.Subscriber('mcBNO', Float32MultiArray, self.sensorTest)
            odmPub.publish(self.odom)


    def sensorTest(self, msg):
        imu = Imu()
        ax, ay, az = msg.data
        if not self.first_ax or not self.first_ay:
            self.first_ax = ax
            self.first_ay = ay
        imu.linear_acceleration = Vector3(x=(ax - self.first_ax), y=(ay - self.first_ay), z=0)
        self.imuPub.publsih(imu)


    def getAccel(self, msg):
        ax, ay, az = msg.data
        # BNO is a bit off because calibration is weird.
        # just run with what it gives us at the start as 0
        if not self.first_ax or not self.first_ay:
            self.first_ax = ax
            self.first_ay = ay
        current_time = rospy.Time.now()
        # Times are in nanosec
        dt = (current_time - self.last_update).to_sec()
        vx = float((ax - self.first_ax) * dt)
        vy = float((ay - self.first_ay) * dt)
        odom = Odometry()
        odom_quat = t
        odom.header.stamp =

        self.last_update = current_time


    def getOdometry(self, msg):
        self.odom = msg


if __name__ == '__main__':
    try:
        OdomPublisher()
    except KeyboardInterrupt:
        print('CTRL-C pressed. Program exiting...')
