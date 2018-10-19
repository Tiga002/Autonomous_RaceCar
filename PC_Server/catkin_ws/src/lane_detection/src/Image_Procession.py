#!/usr/bin/env python
#from __future__ import print_function
import roslib
#roslib.load_manifest('barc')
import sys
import rospy
import cv2
from std_msgs.msg import String, Int32, Float32, UInt16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np




point_num = 0

# Parameter Default Values
display_image = False
publish_image = False
calibrate_transform = False
image_calibrated = True
calibration_pts = None


def find_offset_in_lane(img,x,y,width):
    """
    Returns the difference in x and y positions
    operates on pixels. Return value is pixel offset from nominal
    """
    x_left = x
    x_right = x
    while(not img[y, x_left].any()):
        x_left = x_left - 1
    while(not img[y, x_right].any()):
        x_right = x_right + 1
    return (x_left, x_right)

def get_roi(img, vertices):
    '''
    Transforms an image
     -->by preserving only the ROI represented by the the 'vertices'
    and
     -->removes the remainder of the image by setting the pixel intensity to 0
    :param img (ndarray): Image
    :param vertices (ndarray): Region of Interest'(ROI)' of the image
    :return : Modified image
    '''

    vertices = np.array(vertices, ndmin=3, dtype=np.int32)
    if len(img.shape) == 3:
        fill_color = (255,) * 3
    else:
        fill_color = 255

    mask = np.zeros_like(img)
    mask = cv2.fillPoly(mask, vertices, fill_color)
    return cv2.bitwise_and(img, mask)

def warp_image(img, warp_shape, src, dst):
    '''
    Performs perspective transformation (PT)
    :param img (ndarray): Image
    :param warp_shape: Shape of the warped image
    :param src (ndarray): Source points
    :param dst (ndarray): Destination points
    :return : Tuple (Transformed image, PT matrix, PT inverse matrix)
    '''

    # Get the perspective transformation matrix and its inverse
    M = cv2.getPerspectiveTransform(src, dst)
    invM = cv2.getPerspectiveTransform(dst, src)

    # Warp the image
    warped = cv2.warpPerspective(img, M, warp_shape, flags=cv2.INTER_LINEAR)
    return warped, M, invM

def perspective_transform(cv_image):
    ysize = cv_image.shape[0]
    xsize = cv_image.shape[1]


    src = np.float32([
    (66,478),
    (537,477),
    (439,298),
    (180,296)
    ])

    dst = np.float32([
    (122,480),
    (488,480),
    (488,190),
    (122,190)
    ])

    warped, M, invM = warp_image(cv_image, (xsize, ysize), src, dst)

    vertices = np.array([
        [8, 480],
        [633, 480],
        [636, 4],
        [9, 4],
    ])

    roi = get_roi(warped, vertices)
    return roi, (M, invM)

class image_processor:
    """
    This class takes image messages from the USB Camera and converts them to a cv2 format
    subsequently it performs a perspective transform, then different filters are used to produced
    a threshold binary image
    Finally, it outputs a delta value indicating the offset of the vehicle from the center of the lane
    """

    def __init__(self):
        global display_image, publish_image, calibrate_transform
        global calibration_pts

        #Create ROS Interfaces
        # Publisher : offset_lane_pub , topic : "lane_offset", msg type = Float32
       # self.offset_lane_pub = rospy.Publisher("lane_offset", Float32,queue_size=10)

        self.bridge = CvBridge()

        # Subscriber : Image Sub, subscribed content type = Image, once subscribed --> callback_image()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image,self.callback_image)

        #Get Launch File Parameters and configure node
        #calibrate_transform = rospy.get_param("/image_processing/calibrate_transform")
        #display_image = rospy.get_param("/image_processing/display_image")
        #publish_image = rospy.get_param("/image_processing/publish_image")
        display_image = True        
        global image_calibrated

        if publish_image:
            # Publisher : image_pub, topic : "cv_image" , type = image
            self.image_pub = rospy.Publisher("cv_image", Image, queue_size = 10)
        # Initialize the offset as 0 at the beginning
        self.prev_offset = 0
        self.lane_offset_cm = 0
        #cv2.namedWindow("Calibrate Image Transform")

   # Once message is subscribed , pass it to callback_image
    def callback_image(self,data):
        """
        Callback for incoming image message
        """
        # Global Variables
        global display_image, publish_image, image_calibrated, lane_offset_cm, detected_red_flag

        # Convert ROS Image Message to CV2 image (np array)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
	# Step 1: Perspective Transform : Wrapped --> ROI
        roi, _ = perspective_transform(cv_image)
        if display_image:
            cv2.imshow("Bird's View", roi)
            cv2.waitKey(3)
       
        gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)          
        
        # lane detecting
       
        # Step 2: Thresholded Binary Filtering (white 1, Black 0)
        #filtered = get_binary_image(roi)
        blur = cv2.bilateralFilter(gray_image,3,255,255)
        binary_image = cv2.threshold(blur, 165, 255, cv2.THRESH_BINARY)[1]
        #view = np.dstack((binary_image, binary_image, binary_image)) * 255
        #plot_images([(out, 'Out')], figsize=(30, 40))
        cv2.line(binary_image,(640,385),(575,480),(255,255,255),5)
        cv2.line(binary_image,(1,385),(71,480),(255,255,255),5)

        # If one day i want publish_image
        if publish_image:
            backtorgb = cv2.cvtColor(filtered, cv2.COLOR_GRAY2RGB)

    # =========== Lane Detection Here !!! ==============
        height, width = binary_image.shape
        index_x = width//2 + self.prev_offset
        index_y = 30
        x_left, x_right = find_offset_in_lane(binary_image, index_x, height - index_y, width)
        lane_midpoint = x_left + ((x_right - x_left)//2)
        lane_offset = lane_midpoint - width//2
        self.prev_offset = lane_offset
        # if lane_offset > 0:
        #     print("Turn Right")
        # elif lane_offset < 0:
        #     print("Turn Left")
        # else:
        #     print("Okay")
        #
        # print('x_left is')
        # print(x_left)
        # print('x_right is')
        # print(x_right)
        # print('lane midpt is')
        # print(lane_midpoint)
        # print('lane offset')
        # print(lane_offset)

        cv2.circle(binary_image, (x_left, height - index_y), 3, (0,255,255), 3)
        cv2.circle(binary_image, (x_right, height - index_y), 3, (0,255,255), 3)
        cv2.circle(binary_image, (lane_midpoint, height - index_y), 3, (255,0,255), 3)
        cv2.circle(binary_image, (width//2, height - index_y), 3, (255,255,255), 2)
        x_cm_per_px = 0.085
        self.lane_offset_cm = lane_offset * x_cm_per_px
        print('lane offset in meter is')
        print(self.lane_offset_cm)
	#np.set_printoptions(threshold=np.nan)
	#filtered = filtered * 255.
        if display_image:
            #cv2.imshow("Car View", view)
	    #cv2.waitKey(3)
            backtorgb = binary_image * 255.
            cv2.imshow("Car View2", backtorgb)
            cv2.waitKey(3)
# ==========================================================================
        # Publish the lane offset here !!!!! [ in cm]
       # self.offset_lane_pub.publish(Float32(lane_offset_cm))
# ==========================================================================
        if publish_image:
            try:
                backtorgb = cv2.cvtColor(view, cv2.COLOR_GRAY2RGB)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(backtorgb, "bgr8"))
            except CvBridgeError as e:
                print(e)
        
        
def shutdown_func():
    cv2.destroyAllWindows()

image_processor_global = None

def main(args):
    rospy.on_shutdown(shutdown_func)
    global image_processor_global
    image_processor_global = image_processor()
    
    rospy.init_node('image_processing', anonymous=True)
    rate = rospy.Rate(10)
    offset_lane_pub = rospy.Publisher("lane_offset", Float32,queue_size=10)
    detected_red_pub = rospy.Publisher("detected_red", UInt16, queue_size=10) 
    while not rospy.is_shutdown():
        rospy.loginfo("Offset Published")
        offset_pub = image_processor_global.lane_offset_cm
        print(offset_pub)
        offset_lane_pub.publish(Float32(offset_pub))
      
        rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
