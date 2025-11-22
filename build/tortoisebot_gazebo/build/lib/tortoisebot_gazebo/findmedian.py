import rclpy 
import math
from std_msgs.msg import String
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from scipy.signal import medfilt
import numpy as np
from geometry_msgs.msg import Twist

class FindMedian(Node):
    def __init__(self):
        super().__init__('Find_median_node')
        self.findMedianSubscriber=self.create_subscription(LaserScan,'/scan',self.laser_callback,10)
        self.MAX_INVALID_DISTANCE = 0.08
        self.MEDIAN_KERNEL_SIZE = 3
        self.FOV_DEGREE=15

        # self.filter_median_pub=self.create_publisher(String,'closest_object_distance',10)
        self.twist_publish=self.create_publisher(Twist,'/cmd_vel',10)


    def laser_callback(self,msg_data):

        self.twist_keyboard = Twist()


        find_radians=math.radians(self.FOV_DEGREE)
        angle_min_target=-find_radians
        angle_max_target=find_radians

        start_index=int((angle_min_target - msg_data.angle_min) / msg_data.angle_increment)
        end_index= int((angle_max_target-msg_data.angle_min) / msg_data.angle_increment)

        total_range=len(msg_data.ranges)
        start_index=max(0,start_index)
        end_index=min(total_range-1,end_index)

        frontal_ranges = msg_data.ranges[start_index:end_index + 1]

        
        valid_distances = [] # Initialize an empty list
        for d in frontal_ranges: 
            if not math.isinf(d) and not math.isnan(d) and d > self.MAX_INVALID_DISTANCE:
                valid_distances.append(d)

        filtered_slice = medfilt(valid_distances, kernel_size=self.MEDIAN_KERNEL_SIZE)
        print(start_index)
        print(filtered_slice)
        print(len(filtered_slice))
        if valid_distances:
            closest_distance = min(filtered_slice)
            print(np.argmin(filtered_slice))
            min_index = np.argmin(filtered_slice) + start_index
            print(min_index)
            closest_angle = msg_data.angle_min + min_index * msg_data.angle_increment
            KEEP_DISTANCE=0.4
            if closest_angle < -0.1:
                if closest_distance>KEEP_DISTANCE:
                    self.twist_keyboard.angular.z=-0.3
            elif closest_angle > 0.1:
                if closest_distance>KEEP_DISTANCE:
                    self.twist_keyboard.angular.z=0.3
            else:
                self.twist_keyboard.angular.z=0.0
                if closest_distance>KEEP_DISTANCE:
                    self.twist_keyboard.linear.x=0.3
                else:     
                    self.twist_keyboard.linear.x=0.0
            
            self.twist_publish.publish(self.twist_keyboard)
        else:
            self.get_logger().warn("No valid LiDAR data! Skipping this cycle.")
            self.twist_keyboard.linear.x=0.0
            self.twist_keyboard.angular.z=0.0
            self.twist_publish.publish(self.twist_keyboard)
            return  
            
                
        # distance_msg = String()
        

        # if closest_distance < msg_data.range_max:
        #      distance_str = f"{closest_distance:.3f} meters"
        #      self.get_logger().info(f'Closest object in full scan: {closest_distance:.3f} meters')
        # else:
        #      distance_str = "No valid object detected in full scan FOV."
        #      self.get_logger().info('No valid object detected in full scan FOV.')

        # distance_msg.data = distance_str
        # self.filter_median_pub.publish(distance_msg)


def main(args=None):
    rclpy.init(args=args)
    find_median=FindMedian()

    try: 
        rclpy.spin(find_median)
    except KeyboardInterrupt:
        pass

    find_median.destroy_node()
    # rclpy.shutdown()

if __name__=='__main__':
    main()