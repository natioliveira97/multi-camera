from kinect_camera import KinectCamera
from sensor_msgs.msg import  Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import roslaunch
import rospy
import time
import argparse
import os

bridge = CvBridge()
username = ''
folder = ''
counter = 0

def callback(msg):
    global counter
    try:
        image = bridge.imgmsg_to_cv2(msg, "passthrough")
        cv2.imshow("Image Window", image) 

        key = cv2.waitKey(30)  

        if key == ord('p'):
            print("Save image "+folder+'/'+username+"/"+str(counter)+".jpg")
            cv2.imwrite(folder+'/'+username+"/"+str(counter)+".jpg", image)
            counter+=1
            

    except Exception as e:
        print(e)

 


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-rf', "--register_folder", help='path to register dataset', required=True)
    parser.add_argument('-un', "--user_name",  help='user name', required=True)
    args = vars(parser.parse_args())

    username = args['user_name']
    folder = args['register_folder']

    if not os.path.exists(folder+"/"+username):
        os.makedirs(folder+"/"+username)

    rospy.init_node("cadastro", anonymous=True)

    kinect = KinectCamera(camera_name="kinect1", device_id='A00363904314053A')
    kinect.start()

    sub = rospy.Subscriber("kinect1/rgb/image_color", Image, callback)
    
    while not rospy.is_shutdown():

        rospy.sleep(1)

    print("Exit")