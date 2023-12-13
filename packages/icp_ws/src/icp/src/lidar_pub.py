#!/usr/bin/env python3

import rospy
# from icp.msg import Lidar  # Import the custom message type
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar
import serial
import signal
import numpy as np
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

class my_lidar:
    
    def __init__(self, device):
        # Initialize the ROS node
        rospy.init_node('lidar_publisher', anonymous=True)
        self.lidar = RPLidar(device)

        # Create a publisher for the custom topic with your custom message type
        # self.lidar_pub = rospy.Publisher('lidar_scans', Lidar, queue_size=10)
        queue_size_default = 10
        icp_itr = rospy.get_param("lidar_queue_size", queue_size_default)
        self.lidar_pub = rospy.Publisher('lidar_scans', LaserScan, queue_size=queue_size_default)
        self.pointcloud_pub = rospy.Publisher('lidar_pointcloud', PointCloud, queue_size=queue_size_default)

        self.info = self.lidar.get_info()
        self.health = self.lidar.get_health()
        rospy.sleep(2) # give it some time to initialize
        signal.signal(signal.SIGINT, self.handle_keyboard_interrupt)
    
        # current_time = rospy.Time.now()
        # Print the current time in seconds
        # print("Current Time: {:.6f} seconds".format(current_time.to_sec()))
        self.prev_time = rospy.Time.now().to_sec()
        # self.current_time = 0


    def handle_keyboard_interrupt(self, signal, frame):
        print("Ctrl-C Interrupt Exception received. Cleaning up...")
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        print("Cleaned up, exiting")
        # sys.exit(0)

    # This function is not called
    def run(self):

        try:
            for i, scan in enumerate(self.lidar.iter_scans(scan_type='express', max_buf_meas=False)):
                print(f"Iter: {i}")
                distances = [distance/1000 for quality, angle, distance in scan]
                # distances = distances/1000 # convert to meters
                angles = [np.deg2rad(angle) for quality, angle, distance in scan]

                distances = np.array(distances)
                angles = np.array(angles)

                min_index = np.argmin(angles)

                # Rewrap the array starting from the minimum index
                angles = np.concatenate((angles[min_index:], angles[:min_index])).tolist()
                distances = np.concatenate((distances[min_index:], distances[:min_index])).tolist()

                # msg = Lidar()
                # msg.distances = distances
                # msg.angles = angles
                # self.lidar_pub.publish(msg)
                # # print(scan)
                msg = LaserScan()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "sonic/lidar_frame"
                curr_time = rospy.Time.now().to_sec()
                # msg.time_increment = (curr_time - self.prev_time) / len(angles)  # Assuming 1 Hz
                # msg.scan_time = (curr_time - self.prev_time) / len(angles)  # Assuming 1 Hz
                self.prev_time = curr_time
                # _Scan.angle_max = numpy.pi - numpy.radians(0.0)
                # _Scan.angle_min = numpy.pi - numpy.radians(360.0)
                # _Scan.angle_increment = -numpy.radians(1.0)
                msg.angle_min = np.pi - np.radians(0.0) #min(angles)
                msg.angle_max = np.pi - np.radians(360) #max(angles)
                msg.angle_increment = 2*np.pi/len(angles) #(max(angles) - min(angles)) / len(angles)
                msg.range_min = min(distances)
                msg.range_max = max(distances)
                msg.ranges = distances
                msg.intensities = [1.0] * len(distances)

                print(angles)
                print()
                print(distances)
                print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -")
                self.lidar_pub.publish(msg)

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
        
    def publish_lidar_pointcloud(self):
        try:
            # Your existing LIDAR data loop
            for i, scan in enumerate(self.lidar.iter_scans(scan_type='express', max_buf_meas=False)):
                # print(f"Iter pc: {i}")
                distances = [distance / 1000 for quality, angle, distance in scan]
                angles = [np.deg2rad(angle) for quality, angle, distance in scan]

                distances = np.array(distances)
                angles = np.array(angles)

                # Convert polar coordinates to Cartesian coordinates
                x = -(distances * np.cos(angles))
                y = distances * np.sin(angles)

                # Create PointCloud message
                pointcloud_msg = PointCloud()
                pointcloud_msg.header.stamp = rospy.Time.now()
                pointcloud_msg.header.frame_id = "sonic/lidar_frame"

                # Populate the PointCloud message with Point32 points
                points = [Point32(x=float(x[i]), y=float(y[i]), z=0.0) for i in range(len(x))]
                pointcloud_msg.points = points

                # Add intensity values as an additional channel with default value 1
                intensity_channel = ChannelFloat32()
                intensity_channel.name = "intensity"
                intensity_channel.values = [1.0] * len(scan)
                pointcloud_msg.channels.append(intensity_channel)

                # Publish the PointCloud message
                self.pointcloud_pub.publish(pointcloud_msg)
                
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

if __name__ == '__main__':
    try:
        node = my_lidar('/dev/ttyUSB0')
        # node.run()
        node.publish_lidar_pointcloud()
    except rospy.ROSInterruptException:
        pass
