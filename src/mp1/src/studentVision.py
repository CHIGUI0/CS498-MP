import time
import math
import numpy as np
import cv2
import rospy

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology



class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        # self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)


    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to unint8, then apply threshold to get binary image

        self.show_img(img)

        ## TODO
        gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        blurredimg = cv2.GaussianBlur(gray_img, (15, 15),0)

        self.show_img(blurredimg)

        sobelx = cv2.Sobel(blurredimg,cv2.CV_64F,1,0,ksize=3)

        sobely = cv2.Sobel(blurredimg,cv2.CV_64F,0,1,ksize=3)

        sobel = cv2.addWeighted(np.abs(sobelx), 0.5, np.abs(sobely), 0.5, 0)
        sobel_combined_uint8 = cv2.convertScaleAbs(sobel)

        # First lower bound threshold
        _, lower_output = cv2.threshold(sobel_combined_uint8, thresh_min, 1, cv2.THRESH_BINARY)
        # Second upper bound threshold
        _, upper_inverse_output = cv2.threshold(sobel_combined_uint8, thresh_max, 1, cv2.THRESH_BINARY_INV)

        binary_output = cv2.bitwise_and(lower_output, upper_inverse_output)

        ####

        return binary_output


    def color_thresh(self, img, thresh=(100, 255)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass
        ## TODO
        image_hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)

        # Uncomment this block for lane detection of Bags 0011&0056
        # Parameters for Bags 0011&0056
        # Extended Yellow HLS ranges
        yellow_lower = np.array([15, 100, 100])  # Lower hue to capture more yellow shades
        yellow_upper = np.array([45, 255, 255])  # Higher hue to capture more yellow shades

        # Extended White HLS ranges
        white_lower = np.array([0, 200, 0])      # Lower saturation to include near-whites
        white_upper = np.array([255,250,100])  # Higher lightness to include brighter whites


        # Uncomment this block for lane detection of Test
        # Parameters for Test
        # Extended Yellow HLS ranges
        # yellow_lower = np.array([15, 100, 100])  # Lower hue to capture more yellow shades
        # yellow_upper = np.array([45, 255, 255])  # Higher hue to capture more yellow shades

        # # Extended White HLS ranges
        # white_lower = np.array([0, 180, 0])      # Lower saturation to include near-whites
        # white_upper = np.array([255, 255, 150])  # Higher lightness to include brighter whites        

        # Uncomment this block for lane detection of Bag 0484
        # Parameter for Bag 0484
        # # Extended Yellow HLS ranges
        # yellow_lower = np.array([15, 100, 100])  # Lower hue to capture more yellow shades
        # yellow_upper = np.array([45, 255, 255])  # Higher hue to capture more yellow shades

        # # Extended Grey HLS ranges
        # white_lower = np.array([0, 255*0.3, 0])      # Lower saturation to include near-whites
        # white_upper = np.array([255, 255*0.6, 50])  # Higher lightness to include brighter whites

        # Step 4: Create masks for yellow and white
        yellow_mask = cv2.inRange(image_hls, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(image_hls, white_lower, white_upper)

        # Step 5: Combine masks (optional)
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

        # Step 6: Convert the mask to a single channel with 255 for yellow/white and 0 otherwise
        single_channel = np.zeros_like(combined_mask)
        single_channel[combined_mask > 0] = 1  # Set the selected colors to white

        # Step 6: Apply the mask to the original image
        binary_output = single_channel

        return binary_output
    
    def show_img(self, img):
        # return
        umat_sobel = cv2.UMat(img)
        umat_img = umat_sobel
        window_name = "origin"
        cv2.imshow(window_name,umat_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        # cv2.imwrite("/home/lab-station2/Desktop/mp-release-fa24/src/mp1/test.png", img)
        ## TODO

        # Uncomment this block for lane detection of Bags 0484
        # For 0484
        # SobelOutput = self.gradient_thresh(img,35,255)
        
        # Uncomment this block for lane detection of Others
        # For Others
        SobelOutput = self.gradient_thresh(img,50,150)
        self.show_img(SobelOutput*255)

        ColorOutput = self.color_thresh(img,(230,1))
        self.show_img(ColorOutput*255)

        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
        # Remove noise from binary image
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)

        binaryImage = (binaryImage*255).astype(np.uint8)

        return binaryImage


    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO
        
        shape_x = img.shape[1]
        shape_y = img.shape[0]
        
        # Uncomment this block for lane detection of Bags 0011&0056
        # For Bag-0011&0056
        src_points = np.float32([[1/3*shape_x,3/5*shape_y],[2/3*shape_x,3/5*shape_y],[shape_x-20,shape_y-20],[20,shape_y-20]])
        dst_points = np.float32([[20,20],[shape_x-20,20],[shape_x-20,shape_y-20],[20,shape_y-20]])

        # Uncomment this block for lane detection of Test
        # For test
        # src_points = np.float32([[1/3*shape_x,1/2*shape_y],[2/3*shape_x,1/2*shape_y],[shape_x-20,shape_y-20],[20,shape_y-20]])
        # dst_points = np.float32([[20,20],[shape_x-20,20],[shape_x-20,shape_y-20],[20,shape_y-20]])

        # Uncomment this block for lane detection of Bag 0484
        # For 0484
        # src_points = np.float32([[570,4/5*shape_y],[2/3*shape_x,4/5*shape_y],[shape_x-20,shape_y-20],[420,shape_y-20]])
        # dst_points = np.float32([[20,20],[shape_x-20,20],[shape_x-20,shape_y-20],[20 + 0.45*shape_x,shape_y-20]])

        M = cv2.getPerspectiveTransform(src_points, dst_points)
        Minv = cv2.getPerspectiveTransform(dst_points, src_points)

        warped_img = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]),flags=cv2.INTER_LINEAR)  

        return warped_img, M, Minv


    def detection(self, img):

        # self.show_img(img)
        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)
        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            else:
                print("Unable to detect lanes")

            return combine_fit_img, bird_fit_img


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
