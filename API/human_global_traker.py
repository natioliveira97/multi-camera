import rospy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from pyquaternion import Quaternion
import numpy as np
import numpy.matlib as npm
import json
import logging
import geometry_msgs
import tf2_msgs
from threading import Thread, Lock

class HumanJointEKF:
    def __init__(self, user, joint, time_threshold = 2):
        self.user = user
        self.joint = joint

        self.time_threshold = time_threshold


        sigma_camera = 0.5
        sigma_process = 0.5
        self.R = (sigma_camera**2)*np.eye(7)
        self.R2 = np.array([[0.000000001]])
        self.Q = (sigma_process**2)*np.eye(10)
        self.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])


        self.time = 0


    def reset(self, time, measure):
        self.x = np.array([measure[0],
                           measure[1],
                           measure[2],
                           0,
                           0,
                           0,
                           measure[3],
                           measure[4],
                           measure[5],
                           measure[6]])

        self.P = self.Q

    def predict(self, time):
        dt = time-self.time
        self.F = np.array([[1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, dt, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, dt, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])


        self.x_pred = self.F.dot(self.x)
        self.P_pred = self.F.dot(self.P).dot(self.F.transpose())+self.Q

    def update(self, time, measure):
        
        K = self.P_pred.dot(self.H.transpose()).dot(np.linalg.inv((self.H.dot(self.P_pred).dot(self.H.transpose())+self.R)))
        self.x = self.x_pred+K.dot(measure-self.H.dot(self.x_pred))

        self.P = (np.eye(10)-K.dot(self.H)).dot(self.P_pred).dot((np.eye(10)-K.dot(self.H)).transpose())+K.dot(self.R).dot(K.transpose())


        self.H2 = np.array([[0, 0, 0, 0, 0, 0, 2*self.x[6], 2*self.x[7],2*self.x[8],2*self.x[9]]])
        K2 = self.P.dot(self.H2.transpose()).dot((np.linalg.inv((self.H2.dot(self.P).dot(self.H2.transpose())+self.R2))))
        self.x = self.x+K2.dot([1-(self.x[6]**2 + self.x[7]**2 + self.x[8]**2 + self.x[9]**2)])
        self.P = (np.eye(10)-K2.dot(self.H2)).dot(self.P).dot((np.eye(10)-K2.dot(self.H2)).transpose())+K2.dot(self.R2).dot(K2.transpose())

    def run(self, time, measure):
        if self.time == 0 or time-self.time>self.time_threshold:
            self.reset(time, measure)

        else:
            self.predict(time)
            if ((measure[3]-self.x[6])**2 + (measure[4]-self.x[7])**2 + (measure[5]-self.x[8])**2 + (measure[6]-self.x[9])**2 >
                (measure[3]+self.x[6])**2 + (measure[4]+self.x[7])**2 + (measure[5]+self.x[8])**2 + (measure[6]+self.x[9])**2) :

                measure[3]=-measure[3]
                measure[4]=-measure[4]
                measure[5]=-measure[5]
                measure[6]=-measure[6]

                
            self.update(time,measure)

        self.time = time

        return self.x


class User:
    def __init__(self, user):
        self.joints = ["head", "neck", "torso", "left_shoulder", "left_elbow", "left_hand",
                  "right_shoulder", "right_elbow", "right_hand", "left_hip", "left_knee",
                  "left_foot", "right_hip", "right_knee", "right_foot"]

        self.joint_filters = {} 
        for joint in self.joints:
            self.joint_filters[joint] = HumanJointEKF(user, joint)

    def update_user(self, user):
        for joint in self.joints:
            self.joint_filters[joint].user = user


class GlobalHumanTracker:
    def __init__(self, conf_file, with_recognition=True):
        f = open(conf_file)
        self.cameras = json.load(f)

        self.cam_user_to_person = {}
        self.temp_cam_user_to_person = {}

        for camera in self.cameras.keys():
            self.cameras[camera]['r']=Quaternion(w=self.cameras[camera]['r']['w'],
                                                x=self.cameras[camera]['r']['x'],
                                                y=self.cameras[camera]['r']['y'],
                                                z=self.cameras[camera]['r']['z'])

            self.cameras[camera]['t']=np.array([self.cameras[camera]['t']['x'],
                                                self.cameras[camera]['t']['y'],
                                                self.cameras[camera]['t']['z']])

            self.cam_user_to_person[camera]={}
            self.temp_cam_user_to_person[camera]={}

        self.user_filters = {"Joao":User("Joao")}
        self.temp_user_filters = {}

        self.sub = rospy.Subscriber("/skeleton_tf", TFMessage, self.callback)
        self.pub = rospy.Publisher("/global_skeleton_tf", TFMessage, queue_size=10)
        self.query_pub = rospy.Publisher("/recognition_query", String, queue_size=10)
        self.response_sub = rospy.Subscriber("/recognition_response", String, self.response_callback)

        self.clean_up_time = 4
        self.clean_up_thread = Thread(target = self.clean_up)
        self.clean_up_thread.start()
        self.mutex = Lock()

    def clean_up(self):
        while True:
            for user in list(self.user_filters.keys()):
                if rospy.get_time()-self.user_filters[user].joint_filters["head"].time > self.clean_up_time:
                    del self.user_filters[user]
            for user in list(self.temp_user_filters.keys()):
                if rospy.get_time()-self.temp_user_filters[user].joint_filters["head"].time > self.clean_up_time:
                    del self.temp_user_filters[user]
            for camera in list(self.cam_user_to_person.keys()):
                for user in list(self.cam_user_to_person[camera].keys()):
                    if rospy.get_time()-self.cam_user_to_person[camera][user]["time"] >self.clean_up_time:
                        del self.cam_user_to_person[camera][user]
            for camera in list(self.temp_cam_user_to_person.keys()):
                for user in list(self.temp_cam_user_to_person[camera].keys()):
                    if rospy.get_time()-self.temp_cam_user_to_person[camera][user]["time"] > self.clean_up_time:
                        del self.temp_cam_user_to_person[camera][user]

            # print(self.cam_user_to_person)
            # print(self.temp_cam_user_to_person)
            rospy.sleep(self.clean_up_time)




    def response_callback(self, msg):
        camera, user, reco, name = str(msg.data).split(" ")
        if user  in self.cam_user_to_person[camera].keys():
            return

        if user in self.temp_cam_user_to_person[camera].keys():
            if reco == "Recognized":
                try:
                    self.temp_user_filters[self.temp_cam_user_to_person[camera][user]["name"]].update_user(name)
                    self.user_filters[name]=self.temp_user_filters[self.temp_cam_user_to_person[camera][user]["name"]]
                    self.cam_user_to_person[camera][user] = {"name":name, "time":rospy.get_time()}

                    self.mutex.acquire()
                    del self.temp_user_filters[self.temp_cam_user_to_person[camera][user]["name"]]
                    del self.temp_cam_user_to_person[camera][user]
                    self.mutex.release()
                except Exception as ex:
                    print(ex)


    def callback(self, data):
        try:
            for transform in data.transforms:

                camera = transform.header.frame_id
                user_id = transform.child_frame_id.split("_")[-1]
                joint = transform.child_frame_id.split(("_"+user_id))[0]

                
                if user_id  in self.cam_user_to_person[camera].keys():
                    time = float(transform.header.stamp.secs)+float(transform.header.stamp.nsecs)*(10**(-9))

                    joint_r = Quaternion(w = transform.transform.rotation.w,
                                            x = transform.transform.rotation.x,
                                            y = transform.transform.rotation.y,
                                            z = transform.transform.rotation.z)
                                

                    joint_t = np.array([transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z])


                    global_joint_r = self.cameras[camera]['r']*joint_r
                    global_joint_t = self.cameras[camera]['r'].rotate(joint_t)+self.cameras[camera]['t']


                    measure = [global_joint_t[0],
                            global_joint_t[1],
                            global_joint_t[2],
                            global_joint_r[0],
                            global_joint_r[1],
                            global_joint_r[2],
                            global_joint_r[3]]

                    x = self.user_filters[self.cam_user_to_person[camera][user_id]["name"]].joint_filters[joint].run(time,measure)

                    self.cam_user_to_person[camera][user_id]["time"]=rospy.get_time()
                    t = geometry_msgs.msg.TransformStamped()
                    t.header.frame_id = "world"
                    t.header.stamp = rospy.Time.now()
                    t.child_frame_id = joint+"_"+self.cam_user_to_person[camera][user_id]["name"]
                    t.transform.translation.x = x[0]
                    t.transform.translation.y = x[1]
                    t.transform.translation.z = x[2]

                    t.transform.rotation.x = x[7]
                    t.transform.rotation.y = x[8]
                    t.transform.rotation.z = x[9]
                    t.transform.rotation.w = x[6]

                    tfm = tf2_msgs.msg.TFMessage([t])
                    self.pub.publish(tfm)
                    
                else:
                    self.mutex.acquire()
                    if user_id not in self.temp_cam_user_to_person[camera].keys():
                        self.temp_cam_user_to_person[camera][user_id]={"name":"NaoReconhecidoC"+camera+"U"+user_id, "time":rospy.get_time()}
                        self.temp_user_filters["NaoReconhecidoC"+camera+"U"+user_id] = User("NaoReconhecidoC"+camera+"U"+user_id)

                    if(joint=='head'):
                        self.query_pub.publish(camera+" "+user_id+" "+str(transform.transform.translation.x)+" "+str(transform.transform.translation.y)+" "+str(transform.transform.translation.z))

                
                    time = float(transform.header.stamp.secs)+float(transform.header.stamp.nsecs)*(10**(-9))

                    joint_r = Quaternion(w = transform.transform.rotation.w,
                                            x = transform.transform.rotation.x,
                                            y = transform.transform.rotation.y,
                                            z = transform.transform.rotation.z)
                                

                    joint_t = np.array([transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z])


                    global_joint_r = self.cameras[camera]['r']*joint_r
                    global_joint_t = self.cameras[camera]['r'].rotate(joint_t)+self.cameras[camera]['t']


                    measure = [global_joint_t[0],
                            global_joint_t[1],
                            global_joint_t[2],
                            global_joint_r[0],
                            global_joint_r[1],
                            global_joint_r[2],
                            global_joint_r[3]]

                    x = self.temp_user_filters[self.temp_cam_user_to_person[camera][user_id]["name"]].joint_filters[joint].run(time,measure)

                    self.temp_cam_user_to_person[camera][user_id]["time"]=rospy.get_time()
                    t = geometry_msgs.msg.TransformStamped()
                    t.header.frame_id = "world"
                    t.header.stamp = rospy.Time.now()
                    t.child_frame_id = joint+"_"+self.temp_cam_user_to_person[camera][user_id]["name"]
                    t.transform.translation.x = x[0]
                    t.transform.translation.y = x[1]
                    t.transform.translation.z = x[2]

                    t.transform.rotation.x = x[7]
                    t.transform.rotation.y = x[8]
                    t.transform.rotation.z = x[9]
                    t.transform.rotation.w = x[6]

                    tfm = tf2_msgs.msg.TFMessage([t])
                    self.pub.publish(tfm)
                    self.mutex.release()
                    
        except Exception as ex:
            print("lala"+ex)

        # self.mutex.release()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    GlobalHumanTracker("kinects.json")
    rospy.spin()