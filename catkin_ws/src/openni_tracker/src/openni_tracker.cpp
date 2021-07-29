#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;


XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

// get the USB port that the deviced passed in is plugged into
std::string getUsbPort(xn::NodeInfo deviceNodeInfo) {
  // get the id of the usb port the device is plugged into
  const XnChar *creationInfo = deviceNodeInfo.GetCreationInfo();
  std::string creationInfoString = std::string(creationInfo);
  std::size_t lastSlashIdx = creationInfoString.find_last_of('/');
  std::string usbPort = creationInfoString.substr(lastSlashIdx + 1);
  return usbPort;
}

// read the ros parameter desired_serial
std::string getDesiredSerialNumber() {
  std::string desiredSerial;

  if (!ros::param::get("~desired_serial", desiredSerial)) {
    if (!ros::param::get("desired_serial", desiredSerial)) {
      if (!ros::param::get("/desired_serial", desiredSerial)) {
        ROS_ERROR("Parameter 'desired_serial' not found.");
        exit(1);
      }
    }
  }
  ROS_INFO("Desired Serial Number %s", desiredSerial.c_str());
  return desiredSerial;
}

// get the actual serial number of the device passed in
std::string getSerialNumber(xn::NodeInfo deviceNodeInfo) {
  std::string usbPort = getUsbPort(deviceNodeInfo);

  // call lsusb
  FILE *fp;
  char path[1035];
  fp = popen("lsusb -v -d 045e:02ae", "r");
  if (fp == NULL) {
    ROS_ERROR("Failed to run lsusb");
    exit(1);
  }

  // look for the line starting with 'Bus' and containing the device's usbPort
  // After it, look for the line containing 'Serial' and get the serial number
  bool thisEntry = false;
  std::string usbPortWithColon = usbPort + ":";
  std::string serialFound("");
  while (fgets(path, sizeof(path)-1, fp) != NULL) {
    std::string strLine = std::string(path);
    // determine whether this multiline entry represents the device in the given port
    if (strLine.find("Bus") == 0) {
      if (strLine.find(usbPortWithColon) != std::string::npos) {
        thisEntry = true;
      } else {
        thisEntry = false;
      }
    }

    // get the serial number
    if ((strLine.find("Serial") != std::string::npos) && thisEntry) {
      std::size_t lastSpaceIdx = strLine.find_last_of(" ");
      serialFound = strLine.substr(lastSpaceIdx + 1, 16);
    }
  }
  return serialFound;
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
    ROS_INFO("New User %d", nId);

    if (g_bNeedPose)
        g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    else
        g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
    ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
    ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
    if (bSuccess) {
        ROS_INFO("Calibration complete, start tracking user %d", nId);
        g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
    else {
        ROS_INFO("Calibration failed for user %d", nId);
        if (g_bNeedPose)
            g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
        else
            g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double z = joint_position.position.X / 1000.0;
    double x = joint_position.position.Y / 1000.0;
    double y = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
                           m[3], m[4], m[5],
                           m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));//g_Device
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishTransforms(const std::string& frame_id, const std::string& prefix) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;


        publishTransform(user, XN_SKEL_HEAD,           frame_id, prefix + "head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, prefix + "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, prefix + "torso");

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, prefix + "left_shoulder");
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, prefix + "left_elbow");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, prefix + "left_hand");

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, prefix + "right_shoulder");
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, prefix + "right_elbow");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, prefix + "right_hand");

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, prefix + "left_hip");
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, prefix + "left_knee");
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, prefix + "left_foot");

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, prefix + "right_hip");
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, prefix + "right_knee");
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, prefix + "right_foot");
    }
}

#define CHECK_RC(nRetVal, what)                                     \
    if (nRetVal != XN_STATUS_OK)                                    \
    {                                                               \
        ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
        return nRetVal;                                             \
    }

int main(int argc, char **argv) {

    ros::init(argc, argv, "openni_tracker", ros::init_options::AnonymousName);

    ros::NodeHandle nh;

    string strDesiredSerial = getDesiredSerialNumber();
    

    string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    //XnStatus nRetVal = XN_STATUS_OK;
    nRetVal = XN_STATUS_OK;

    //xnLogInitFromXmlFile(csXmlFile);

    nRetVal = g_Context.Init();
    XN_IS_STATUS_OK(nRetVal);


   // SELECTION OF THE DEVICE
    int auto_select = 0; // device automatically selected

    xn::EnumerationErrors errors;
    xn::Device g_Device;
        // find devices
    xn::NodeInfoList list;
    xn::NodeInfoList list_depth;
        nRetVal = g_Context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, list, &errors);
    XN_IS_STATUS_OK(nRetVal);

    ROS_INFO("The following devices were found:");
        int i = 1;
        for (xn::NodeInfoList::Iterator it = list.Begin(); it != list.End(); ++it, ++i)
        {
            xn::NodeInfo deviceNodeInfo = *it;

            xn::Device deviceNode;
            deviceNodeInfo.GetInstance(deviceNode);
            XnBool bExists = deviceNode.IsValid();
            if (!bExists)
            {
                g_Context.CreateProductionTree(deviceNodeInfo, deviceNode);
                // this might fail.
            }

            if (deviceNode.IsValid() && deviceNode.IsCapabilitySupported(XN_CAPABILITY_DEVICE_IDENTIFICATION))
            {
                const XnUInt32 nStringBufferSize = 200;
                XnChar strDeviceName[nStringBufferSize];
                XnChar strSerialNumber[nStringBufferSize];

                XnUInt32 nLength = nStringBufferSize;
                deviceNode.GetIdentificationCap().GetDeviceName(strDeviceName, nLength);
                nLength = nStringBufferSize;
                deviceNode.GetIdentificationCap().GetSerialNumber(strSerialNumber, nLength);

                std::string strUsbPort = getUsbPort(deviceNodeInfo);
                std::string strActualSerial = getSerialNumber(deviceNodeInfo);
                ROS_INFO("[%d] name:%s serial(xn):%s usb:%s serial(lsusb):%s",
                       i, strDeviceName, strSerialNumber, strUsbPort.c_str(),
                       strActualSerial.c_str());

                if (strActualSerial.compare(strDesiredSerial) == 0) {
                  auto_select = i;
                }
            }
            else
            {
              ROS_INFO("[%d] %s\n", i, deviceNodeInfo.GetCreationInfo());
            }

            // release the device if we created it
            if (!bExists && deviceNode.IsValid())
            {
                deviceNode.Release();
            }
        }

        if (auto_select == 0) {
          auto_select = 1;
          ROS_INFO("No serial number match found.");
          exit(1);
        } else {
          ROS_INFO("Device %d automatically selected based on serial number match.",
                 auto_select);
        }
        ROS_INFO("");
        int chosen = auto_select;

        // create it
        xn::NodeInfoList::Iterator it = list.Begin();
        for (i = 1; i < chosen; ++i)
        {
            it++;
        }

        xn::NodeInfo deviceNode = *it;
        nRetVal = g_Context.CreateProductionTree(deviceNode, g_Device);
        ROS_INFO("Production tree of the device created.");

     // SELECTION OF THE DEPTH GENERATOR
        nRetVal = g_Context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, list_depth, &errors);
        XN_IS_STATUS_OK(nRetVal);

        ROS_INFO("The following devices were found:");
            int i_depth = 1;
            for (xn::NodeInfoList::Iterator it_depth = list_depth.Begin(); it_depth != list_depth.End(); ++it_depth, ++i_depth)
            {
                xn::NodeInfo depthNodeInfo = *it_depth;

                xn::Device depthNode;
                depthNodeInfo.GetInstance(depthNode);
                XnBool bExists_depth = depthNode.IsValid();
                if (!bExists_depth)
                {
                    g_Context.CreateProductionTree(depthNodeInfo, depthNode);
                    // this might fail.
                }

                if (depthNode.IsValid() && depthNode.IsCapabilitySupported(XN_CAPABILITY_DEVICE_IDENTIFICATION))
                {
                    const XnUInt32 nStringBufferSize = 200;
                    XnChar strDeviceName[nStringBufferSize];
                    XnChar strSerialNumber[nStringBufferSize];

                    XnUInt32 nLength = nStringBufferSize;
                    depthNode.GetIdentificationCap().GetDeviceName(strDeviceName, nLength);
                    nLength = nStringBufferSize;
                    depthNode.GetIdentificationCap().GetSerialNumber(strSerialNumber, nLength);
                    ROS_INFO("[%d] %s (%s)", i, strDeviceName, strSerialNumber);
                }
                else
                {
                    ROS_INFO("[%d] %s", i, depthNodeInfo.GetCreationInfo());
                }

                // release the device if we created it
                if (!bExists_depth && depthNode.IsValid())
                {
                    depthNode.Release();
                }
            }

        int chosen_depth = 1;
        ROS_INFO("Device %d automatically selected.", chosen_depth);
        ROS_INFO("");

        // create it
        xn::NodeInfoList::Iterator it_depth = list_depth.Begin();
        for (i = 1; i < chosen_depth; ++i)
        {
            it_depth++;
        }

        xn::NodeInfo depthNode = *it_depth;
        nRetVal = g_Context.CreateProductionTree(depthNode, g_DepthGenerator);
        ROS_INFO("Production tree of the DepthGenerator created.");

        nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
        ROS_INFO("Production tree of the depth generator created.");
        XN_IS_STATUS_OK(nRetVal);
        ROS_INFO("XN_IS_STATUS_OK(nRetVal).");



    CHECK_RC(nRetVal, "Find depth generator");
     ROS_INFO("CHECK_RC(nRetVal, Find depth generator);");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    ROS_INFO("User generator found.");
    if (nRetVal != XN_STATUS_OK) {
        nRetVal = g_UserGenerator.Create(g_Context);
        ROS_INFO("User generator created.");
        CHECK_RC(nRetVal, "Find user generator");
    }

    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        ROS_ERROR("Supplied user generator doesn't support skeleton");
        return 1;
    }

    XnCallbackHandle hUserCallbacks;
    g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

    XnCallbackHandle hCalibrationCallbacks;
    g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
        g_bNeedPose = TRUE;
        if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
            ROS_INFO("Pose required, but not supported");
            return 1;
        }

        XnCallbackHandle hPoseCallbacks;
        g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

        g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

    ros::Rate r(30);


        // ros::NodeHandle pnh("~");
        // string frame_id("/kinect1_depth_frame");
        // pnh.getParam("camera_frame_id", frame_id);

    std::string frame_id("/camera_depth_frame");
    if (!ros::param::get("~camera_frame_id", frame_id)) {
      if (!ros::param::get("camera_frame_id", frame_id)) {
        ROS_WARN("Parameter 'camera_frame_id' not found, using default.");
      }
    }
    ROS_INFO("camera_frame_id %s", frame_id.c_str());

    std::string tf_prefix("");
    if (!ros::param::get("~tf_prefix", tf_prefix)) {
      if (!ros::param::get("tf_prefix", tf_prefix)) {
        ROS_WARN("Parameter 'tf_prefix' not found, using default.");
      }
    }
    ROS_INFO("tf_prefix %s", tf_prefix.c_str());

    while (ros::ok()) {
        g_Context.WaitAndUpdateAll();
        publishTransforms(frame_id, tf_prefix);
        r.sleep();
    }

    g_Context.Shutdown();
    return 0;
}