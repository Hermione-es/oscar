import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

linear_vel = 0
angular_vel = 0
twist = Twist()

def joyCB(msg):
    # msg = Joy()
    linear_vel = msg.axes[7]/10
    angular_vel = msg.axes[3]/5


    twist.linear.x = linear_vel
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_vel
    pub.publish(twist)

# return msg_throttle, msg_steer

if __name__ == '__main__':
    rospy.init_node('set_vel')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/joy', Joy, joyCB)

    rospy.spin()