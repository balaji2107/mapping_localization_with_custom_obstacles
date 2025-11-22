import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from scipy.signal import medfilt
import numpy as np
import time
class FindDirection(Node):
    def __init__(self):
        super().__init__('find_direction_node')
        self.FOV_DEGREE=15
        self.MAX_INVALID=0.08
        self.laser_data=self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)

        self.twist_publish=self.create_publisher(Twist,'/cmd_vel',10)

    def lidar_callback(self,msg_data):

        self.twist_keyboard = Twist()
        
        valid_ranges=msg_data.ranges

        filtered_slice = medfilt(valid_ranges, kernel_size=3)


        # for i in range(len(msg_data.ranges)):
        #     if not math.isnan(msg_data.ranges[i]) and not math.isinf(msg_data.ranges[i]) and msg_data.ranges[i]>self.MAX_INVALID:
        #         valid_ranges.append(msg_data.ranges[i])

        min_dist = min(filtered_slice)
        
        min_index = np.argmin(filtered_slice)
        closest_angle = msg_data.angle_min + min_index * msg_data.angle_increment
        # print(closest_angle)
        if math.isinf(min_dist):
            self.get_logger().warn("No valid LiDAR data! Skipping this cycle.")
            self.twist_keyboard.linear.x=0.0
            self.twist_keyboard.angular.z=0.0
            self.twist_publish.publish(self.twist_keyboard)
            return  

       
        KEEP_DISTANCE=0.6

        distance_error = min_dist - KEEP_DISTANCE

        if closest_angle < -0.1 or closest_angle > 0.1:
            if min_dist>KEEP_DISTANCE:
                self.twist_keyboard.angular.z= max(-1.0, min(1.0, closest_angle))
        # elif closest_angle > 0.1:
        #     if min_dist>KEEP_DISTANCE:
        #         self.twist_keyboard.angular.z=0.5
        else:
            self.twist_keyboard.angular.z=0.0
            if min_dist>KEEP_DISTANCE: 
                self.twist_keyboard.linear.x=max(0.0, min(0.5, distance_error)) 
                print(self.twist_keyboard.linear.x)
            # else:     
            #     self.twist_keyboard.linear.x=0.0
        
        self.twist_publish.publish(self.twist_keyboard)



        # if valid_ranges and valiprint("hi")d_direction: 
        #     twist_keyboard = Twist()
            
        
        #     KP_FOLLOW = 0.8         
        #     DESIRED_SPEED = 0.3     # Constant forward speed
        #     KEEP_DISTANCE = 0.4     # Target distance to stop at (e.g., 40 cm from the ball)
            
        #     min_dist = min(valid_ranges)
        #     min_index = valid_ranges.index(min_dist)
        #     closest_angle = valid_direction[min_index]

        #     distance_error = min_dist - KEEP_DISTANCE # bot stop before that KEEP_DISTANCE
            
        #     twist_keyboard.linear.x = max(0.0, min(0.5, distance_error)) 
        #     twist_keyboard.angular.z = max(-1.0, min(1.0, closest_angle))
        #     self.twist_publish.publish(twist_keyboard)

        # else:
        #     twist_board=Twist()
        #     twist_board.linear.x = 0.0          
        #     twist_board.angular.z = 0.0
        #     self.twist_publish.publish(twist_board)

            # return  



def main(args=None):
    rclpy.init(args=args)
    find_direction=FindDirection()
    try: 
        rclpy.spin(find_direction)
    except KeyboardInterrupt:
        pass
    finally:
        find_direction.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
