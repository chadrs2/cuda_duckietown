#!/usr/bin/env python3

import rospy
from icp.msg import Lidar  # Import the custom message type
from rplidar import RPLidar
import serial
import signal

class my_lidar:
    
    def __init__(self, device):
        # Initialize the ROS node
        rospy.init_node('lidar_publisher', anonymous=True)
        self.lidar = RPLidar(device)

        # Create a publisher for the custom topic with your custom message type
        self.lidar_pub = rospy.Publisher('lidar_scans', Lidar, queue_size=10)
         
        self.info = self.lidar.get_info()
        self.health = self.lidar.get_health()
        
        rospy.sleep(2) # give it some time to initialize
        signal.signal(signal.SIGINT, self.handle_keyboard_interrupt)
    
    def handle_keyboard_interrupt(self, signal, frame):
        print("Ctrl-C Interrupt Exception received. Cleaning up...")
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        print("Cleaned up, exiting")
        # sys.exit(0)

    def run(self):

        try:
            for i, scan in enumerate(self.lidar.iter_scans(scan_type='express', max_buf_meas=False)):
                print(f"Iter: {i}")
                distances = [distance for quality, angle, distance in scan]
                angles = [angle for quality, angle, distance in scan]

                msg = Lidar()
                msg.distances = distances
                msg.angles = angles
                self.lidar_pub.publish(msg)
                # print(scan)

        except rospy.ROSInterruptException:
            print("ROS Interrupt Exception received. Cleaning up...")
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print("Cleaned up, exiting")
            return
        
        except KeyboardInterrupt:
            print("Ctrl-C Interrupt Exception received. Cleaning up...")
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print("Cleaned up, exiting")
            return

        # while not rospy.is_shutdown():
        #     # Publish the custom message
        #     pub.publish(msg)
        #     rospy.loginfo("Publishing custom message: data = %f, count = %d", msg.data, msg.count)
        #     rate.sleep()

if __name__ == '__main__':
    try:
        node = my_lidar('/dev/ttyUSB0')
        node.run()
    except rospy.ROSInterruptException:
        pass
