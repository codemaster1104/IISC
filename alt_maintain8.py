import rospy
import math
import time
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class FCU_connection:

    def __init__(self):
        rospy.init_node('mavros_fcu')
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=100)
        self.rate = rospy.Rate(20)  # 20 Hz

        self.laser_down_sub = rospy.Subscriber("/spur/laser/scan2", LaserScan, self.laser_down_callback)
        self.laser_front_sub = rospy.Subscriber("/spur/laser/scan", LaserScan, self.laser_front_callback)

        # PID controller parameters
        self.Kp = 0.2
        self.Ki = 0.05
        self.Kd = 0.10
        
        # Target altitude
        self.target_altitude = 3.0
        self.prev_alt=0

        # Initialize bottom distance altitude
        self.bottom_altitude = 0.0
        self.bottomr=0
        self.bottom_angle=0
        
        #initialse front altitude variables
        self.farr=0
        self.farh=0
        self.fard=10
        
        # Variables for PID controller
        self.prev_error = 0.0
        self.integral = 0.0
        
        # arrays
        self.rangess_d=[]
        self.rangess_f=[]
        self.rangess=[]
        
        # forwad velocity
        self.f_vel=0.4
        

    def laser_down_callback(self, data):
        self.rangess_d=list(data.ranges)
        
    def laser_front_callback(self, data):
        self.rangess_f=list(data.ranges)            

    def pid_control(self):
        # Error calculation
        error = self.target_altitude 
    
        # Proportional term
        P = self.Kp * error
        # Integral term
        self.integral =(self.integral + error) * self.rate.sleep_dur.to_sec()
        I = self.Ki * self.integral
        # Derivative term
        derivative = (error - self.prev_error) / self.rate.sleep_dur.to_sec()
        D = self.Kd * derivative
        # PID output
        output = P + I + D
        
        # Update previous error for next iteration
        self.prev_error = error
        
        # Apply the output to velocity
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = self.f_vel
        vel_msg.linear.z = output
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        
        print("output: ",output)

        # Publish the velocity
        self.vel_pub.publish(vel_msg)

    def calculator(self):

        # Initialize arrays to store sine and cosine values
        h_values = []
        d_values = []
        wh_values=[]

        # Calculate angle increment
        angle_increment = (45 + 90) / (len(self.rangess)+1)

        # Iterate through each element of the array
        for i, distance in enumerate(self.rangess):
            # Convert index to angle
            angle = -90 + (i * angle_increment)

            # Check if the distance is infinity
            if math.isinf(distance):
                continue
            else:
                # Calculate sine and cosine values
                h_value = distance*math.sin(math.radians(angle))
                d_value = distance*math.cos(math.radians(angle))

                # Append values to new arrays
                h_values.append(h_value)
                d_values.append(d_value)
                
        if(len(h_values)==0):
            h_values.append(-5)
            d_values.append(1)
        
        plt.clf()  # Clear previous plot
        plt.plot(d_values, h_values, 'o')  # Plot new data
        plt.xlabel('X Axis Label')
        plt.ylabel('Y Axis Label')
        plt.title('Plot of Y vs X')
        plt.grid(True)  # Add grid lines
        plt.pause(0.1)
        
        for i in range(0,len(h_values)):
                x=d_values[i]
                wh_values.append(((-(0.001/self.f_vel)*math.pow(x,3))+1)*h_values[i])
        
        ind = 0    #ind variable to store the index of maximum value in the list
        max_element = wh_values[0]

        for i in range (1,len(wh_values)): #iterate over array
            if math.isinf(h_values[i]):
                continue
            elif wh_values[i] > max_element: #to check max value
                max_element = wh_values[i]
                ind = i
            
        return h_values[ind],d_values[ind]

    def maintain_altitude(self):
        time.sleep(1)
        plt.figure()
        while not rospy.is_shutdown():
            self.rangess=self.rangess_d+self.rangess_f
            self.farh,self.fard=self.calculator()
            
            self.target_altitude=((self.farh+3)/self.fard)*math.exp(self.f_vel)
                
            print("fard: ",self.fard)
            print("farh: ",self.farh)
            print("tar alt: ",self.target_altitude)
            print(" ")
            
            self.pid_control()
            self.rate.sleep()


if __name__ == '__main__':
    fcu1 = FCU_connection()
    fcu1.maintain_altitude()
