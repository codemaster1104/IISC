# import rospy
# from sensor_msgs.msg import Image
# import cv2
# import cv_bridge
# import numpy as np

# class DepthCameraProcessor:
#     def __init__(self):
#         self.bridge = cv_bridge.CvBridge()
#         self.downward_depth_image = None

#         # Subscribe to depth image topic from the downward-facing camera
#         rospy.Subscriber('/down_depth_camera/depth/image_raw', Image, self.depth_callback)

#     def depth_callback(self, data):
#         # Convert ROS Image message to OpenCV image
#         self.downward_depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

#     def weighted_average_depth(self, depth_image):
#         # Get the dimensions of the image
#         height, width = depth_image.shape
        
#         # Create a Gaussian kernel
#         center_x = width // 2
#         center_y = height // 2
#         sigma_x = width / 6
#         sigma_y = height / 6

#         x = np.linspace(0, width - 1, width)
#         y = np.linspace(0, height - 1, height)
#         x, y = np.meshgrid(x, y)

#         gaussian_kernel = np.exp(-((x - center_x) ** 2 / (2 * sigma_x ** 2) + (y - center_y) ** 2 / (2 * sigma_y ** 2)))

#         # Apply the Gaussian kernel to the depth image
#         weighted_depth = depth_image * gaussian_kernel

#         # Compute the weighted average depth value
#         weighted_average = np.nansum(weighted_depth) / np.nansum(gaussian_kernel)

#         return weighted_average

#     def run(self):
#         rate = rospy.Rate(10)  # 10 Hz
#         while not rospy.is_shutdown():
#             if self.downward_depth_image is not None:
#                 weighted_avg = self.weighted_average_depth(self.downward_depth_image)
#                 rospy.loginfo(f"Weighted Average Depth: {weighted_avg:.2f} meters")
#             rate.sleep()

# if __name__ == '__main__':
#     rospy.init_node('depth_camera_processor')
#     processor = DepthCameraProcessor()
#     processor.run()


import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np

class DepthCameraProcessor:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.downward_depth_image = None

        # Subscribe to depth image topic from the downward-facing camera
        rospy.Subscriber('/down_depth_camera/depth/image_raw', Image, self.depth_callback)

    def depth_callback(self, data):
        # Convert ROS Image message to OpenCV image
        self.downward_depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def weighted_average_depth(self, depth_image):
        # Get the dimensions of the image
        height, width = depth_image.shape
        
        # Create a Gaussian kernel
        center_x = width // 2
        center_y = height // 2
        sigma_x = width / 6
        sigma_y = height / 6

        x = np.linspace(0, width - 1, width)
        y = np.linspace(0, height - 1, height)
        x, y = np.meshgrid(x, y)

        gaussian_kernel = np.exp(-((x - center_x) ** 2 / (2 * sigma_x ** 2) + (y - center_y) ** 2 / (2 * sigma_y ** 2)))

        # Apply the Gaussian kernel to the depth image
        weighted_depth = depth_image * gaussian_kernel

        # Compute the weighted average depth value
        weighted_average = np.nansum(weighted_depth) / np.nansum(gaussian_kernel)

        return weighted_average

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.downward_depth_image is not None:
                weighted_avg = self.weighted_average_depth(self.downward_depth_image)
                rospy.loginfo(f"Weighted Average Depth: {weighted_avg:.2f} meters")
                
                # Normalize the depth image for display
                normalized_depth = cv2.normalize(self.downward_depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_display = np.uint8(normalized_depth)
                
                # Display the depth image
                cv2.imshow("Downward Depth Camera", depth_display)
                
                # Check for key press (wait for 1 ms)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('depth_camera_processor')
    processor = DepthCameraProcessor()
    processor.run()
