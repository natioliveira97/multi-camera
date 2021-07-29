
from API.kinect_camera import KinectCamera
from std_msgs.msg import String
from sensor_msgs.msg import  Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import roslaunch
import rospy
import time
import argparse
import os
from API.faceRecognition import FaceDetection
from API.faceRecognition import FaceRecognition
from queue import Queue
import numpy as np



class FaceRecognitionModule:
    def __init__(self, cameras, device_ids, model_path = 'API/models/pretrained/version-RFB-640.pth', register_path = 'registro.pickle'):

        self.cameras = cameras
        self.device_ids = device_ids

        self.fd = FaceDetection(model_path, 480, 640)
        self.fr = FaceRecognition()
        self.fr.loadRegister(register_path)

        self.running = False

        self.sub_image_dict = {}
        self.sub_cam_info_dict = {}
        self.cam_info_dict = {}
        self.cam_dist_dict = {}


        self.bridge = CvBridge()

        self.query_queue=[]

        self.query_sub = rospy.Subscriber("/recognition_query", String, self.query_callback)
        self.response_pub = rospy.Publisher("/recognition_response", String)

        for i, camera in enumerate(self.cameras):
            KinectCamera(camera_name=camera, device_id=self.device_ids[i]).start()
            self.sub_image_dict[camera]=rospy.Subscriber(camera+"/rgb/image_color", Image, self.img_callback, callback_args=camera)
            self.sub_cam_info_dict[camera]=rospy.Subscriber(camera+"/rgb/camera_info", CameraInfo, self.cam_info_callback)

    
    def cam_info_callback(self, msg):
        camera= str(msg.header.frame_id).split("_")[0]
        self.cam_info_dict[camera]=np.array(msg.K).reshape((3,3))
        self.cam_dist_dict[camera]=np.array(msg.D)
        self.sub_cam_info_dict[camera].unregister()



    def img_callback(self,msg,topic):
        if len(self.query_queue):
            camera = self.query_queue[0].split(" ")[0]
            if(topic == camera):
                user = self.query_queue[0].split(" ")[1]
                head_pos = np.array([float(self.query_queue[0].split(" ")[2]),-float(self.query_queue[0].split(" ")[3]),float(self.query_queue[0].split(" ")[4])])
                head_cor = cv2.projectPoints(head_pos,np.array([0.0,0.0,0.0]),np.array([0.0,0.0,0.0]),self.cam_info_dict[camera],self.cam_dist_dict[camera])
                head_cor=head_cor[0][0][0]

                try:
                    image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                    image, boxes = self.fd.detect(image)
                    if len(boxes)>0:
                        image, names, boxes= self.fr.recognize(image, boxes, min_recognition=6, draw_in_image=True)
                        if len(names)>0:
                            for i, name in enumerate(names):
                                image = cv2.circle(image, (int(head_cor[0]), int(head_cor[1])), 2, (255,0,0), 2)
                                cv2.imshow("image", image)
                                cv2.waitKey(33)
                                if head_cor[0]>boxes[i][0] and head_cor[0]<boxes[i][2] and head_cor[1]>boxes[i][1] and head_cor[1]<boxes[i][3]:
                                    self.response_pub.publish(camera+" "+user+" "+"Recognized"+" "+name)
                        else:
                            self.response_pub.publish(camera+" "+user+" "+"NotRecognized"+" "+"Desconhecido")
                    else:
                        self.response_pub.publish(camera+" "+user+" "+"NotDetected"+" "+"Desconhecido")
                        
    
                except Exception as e:
                    print(e)

                self.query_queue.pop(0)

    def query_callback(self, msg):
        self.query_queue.append(str(msg.data))


if __name__ == '__main__':
    try:
        import roslaunch
        import rospy
        import time
        from aruco_detect import ArucoDetect

        rospy.init_node("kinect_test", anonymous=True)

        reco = FaceRecognitionModule(cameras=["kinect1"], device_ids=['A00363904314053A'], register_path="registro_NK.pickle")

        rospy.spin()
    except Exception as ex:
        print(ex)