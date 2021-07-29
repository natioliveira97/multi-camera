import rospy
from tf2_msgs.msg import TFMessage
from pyquaternion import Quaternion
import numpy as np
import numpy.matlib as npm
import json
import logging
import geometry_msgs
import tf2_msgs

class MarkerEKF:
    def __init__(self, marker, time_threshold = 2):
        self.marker = marker

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


        # self.H2 = np.array([[0, 0, 0, 0, 0, 0, 2*self.x[6], 2*self.x[7],2*self.x[8],2*self.x[9]]])
        # K2 = self.P.dot(self.H2.transpose()).dot((np.linalg.inv((self.H2.dot(self.P).dot(self.H2.transpose())+self.R2))))
        # self.x = self.x+K2.dot([1-(self.x[6]**2 + self.x[7]**2 + self.x[8]**2 + self.x[9]**2)])
        # self.P = (np.eye(10)-K2.dot(self.H2)).dot(self.P).dot((np.eye(10)-K2.dot(self.H2)).transpose())+K2.dot(self.R2).dot(K2.transpose())

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
        
class GlobalMarkerTracker:
    def __init__(self, conf_file):
        f = open(conf_file)
        self.cameras = json.load(f)

        for camera in self.cameras.keys():
            self.cameras[camera]['r']=Quaternion(w=self.cameras[camera]['r']['w'],
                                                x=self.cameras[camera]['r']['x'],
                                                y=self.cameras[camera]['r']['y'],
                                                z=self.cameras[camera]['r']['z'])

            self.cameras[camera]['t']=np.array([self.cameras[camera]['t']['x'],
                                                self.cameras[camera]['t']['y'],
                                                self.cameras[camera]['t']['z']])


        self.marker_filters = {}
        self.sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        self.pub = rospy.Publisher("/global_tf", TFMessage, queue_size=10)

    def callback(self, data):

        for transform in data.transforms:

            camera = transform.header.frame_id
            fiducial = transform.child_frame_id

            if fiducial.split("_")[0] != "fiducial":
                break

            
            time = float(transform.header.stamp.secs)+float(transform.header.stamp.nsecs)*(10**(-9))

            fiducial_r = Quaternion(w = transform.transform.rotation.w,
                                    x = transform.transform.rotation.x,
                                    y = transform.transform.rotation.y,
                                    z = transform.transform.rotation.z)
                        

            fiducial_t = np.array([transform.transform.translation.x,
                                   transform.transform.translation.y,
                                   transform.transform.translation.z])


            global_fid_r = self.cameras[camera]['r']*fiducial_r
            global_fid_t = self.cameras[camera]['r'].rotate(fiducial_t)+self.cameras[camera]['t']

            print("t", global_fid_t)


            measure = [global_fid_t[0],
                       global_fid_t[1],
                       global_fid_t[2],
                       global_fid_r[0],
                       global_fid_r[1],
                       global_fid_r[2],
                       global_fid_r[3]]

            print("m", measure)

            if fiducial in self.marker_filters.keys():
                x = self.marker_filters[fiducial].run(time,measure)
            else:
                ekf = MarkerEKF(fiducial)
                self.marker_filters[fiducial] = ekf
                x = self.marker_filters[fiducial].run(time, measure)


            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "world"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = fiducial
            t.transform.translation.x = x[0]
            t.transform.translation.y = x[1]
            t.transform.translation.z = x[2]

            t.transform.rotation.x = x[7]
            t.transform.rotation.y = x[8]
            t.transform.rotation.z = x[9]
            t.transform.rotation.w = x[6]

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub.publish(tfm)

            print("x", x)


            
if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)
    GlobalMarkerTracker("cameras.json")
    rospy.spin()