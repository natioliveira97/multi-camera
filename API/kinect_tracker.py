import roslaunch

class KinectTracker:
    def __init__(self, camera_name, device_id):

        self.camera_name = camera_name
        self.device_id = device_id
        self.running = False


    def start(self):

        try:
            if not self.running:
                args = ['openni_tracker', 'openni_tracker.launch', 'desired_serial:={}'.format(self.device_id), 'camera_frame_id:={}'.format(self.camera_name)]

                roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(args)[0]
                roslaunch_args = args[2:]

                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                self.parent = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)])
                self.parent.start()
                self.running = True    

        except Exception as ex:
            print(ex) 
            self.running == False



    def stop(self):
        self.parent.shutdown()
        self.running == False


if __name__ == '__main__':
    import roslaunch
    import rospy
    import time

    rospy.init_node("tracker", anonymous=True)

    kinect = KinectTracker(camera_name="kinect1", device_id='A00363904314053A')
    kinect.start()

    t = time.time()

    rospy.spin()
    