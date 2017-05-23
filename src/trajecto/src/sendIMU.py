
#!/usr/bin/evn python
import rospy
from sensor_msgs.msg import Imu

import tf


def SendIMU():
   pub = rospy.Publisher('imu/data_raw',Imu,queue_size=10)
   rospy.init_node('SendIMU',anonymous=True)
   
   imu_msg = Imu()
   
   
   quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
   
   imu_msg.header.frame_id = "base_link"
   
   pub.publish(imu_msg)

if __name__== '__main__':
   try:
      SendIMU()
   except rospy.ROSInterruptException:
      pass
