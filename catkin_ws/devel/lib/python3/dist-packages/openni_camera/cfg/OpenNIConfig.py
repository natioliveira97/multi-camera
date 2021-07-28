## *********************************************************
##
## File autogenerated for the openni_camera package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [{'name': 'image_mode', 'type': 'int', 'default': 2, 'level': 0, 'description': 'Image output mode for the color/grayscale image', 'min': 1, 'max': 9, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'SXGA_15Hz', 'type': 'int', 'value': 1, 'srcline': 10, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '1280x1024@15Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'VGA_30Hz', 'type': 'int', 'value': 2, 'srcline': 11, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '640x480@30Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'VGA_25Hz', 'type': 'int', 'value': 3, 'srcline': 12, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '640x480@25Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QVGA_25Hz', 'type': 'int', 'value': 4, 'srcline': 13, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '320x240@25Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QVGA_30Hz', 'type': 'int', 'value': 5, 'srcline': 14, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '320x240@30Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QVGA_60Hz', 'type': 'int', 'value': 6, 'srcline': 15, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '320x240@60Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QQVGA_25Hz', 'type': 'int', 'value': 7, 'srcline': 16, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '160x120@25Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QQVGA_30Hz', 'type': 'int', 'value': 8, 'srcline': 17, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '160x120@30Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QQVGA_60Hz', 'type': 'int', 'value': 9, 'srcline': 18, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '160x120@60Hz', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'output mode'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'depth_mode', 'type': 'int', 'default': 2, 'level': 0, 'description': 'depth output mode', 'min': 2, 'max': 9, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'SXGA_15Hz', 'type': 'int', 'value': 1, 'srcline': 10, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '1280x1024@15Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'VGA_30Hz', 'type': 'int', 'value': 2, 'srcline': 11, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '640x480@30Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'VGA_25Hz', 'type': 'int', 'value': 3, 'srcline': 12, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '640x480@25Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QVGA_25Hz', 'type': 'int', 'value': 4, 'srcline': 13, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '320x240@25Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QVGA_30Hz', 'type': 'int', 'value': 5, 'srcline': 14, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '320x240@30Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QVGA_60Hz', 'type': 'int', 'value': 6, 'srcline': 15, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '320x240@60Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QQVGA_25Hz', 'type': 'int', 'value': 7, 'srcline': 16, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '160x120@25Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QQVGA_30Hz', 'type': 'int', 'value': 8, 'srcline': 17, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '160x120@30Hz', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'QQVGA_60Hz', 'type': 'int', 'value': 9, 'srcline': 18, 'srcfile': '/home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/cfg/OpenNI.cfg', 'description': '160x120@60Hz', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'output mode'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'depth_registration', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Depth data registration', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'data_skip', 'type': 'int', 'default': 0, 'level': 0, 'description': 'Skip N images for every image published (rgb/depth/depth_registered/ir)', 'min': 0, 'max': 1000, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'depth_time_offset', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'depth image time offset in seconds', 'min': -1.0, 'max': 1.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'image_time_offset', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'image time offset in seconds', 'min': -1.0, 'max': 1.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'depth_ir_offset_x', 'type': 'double', 'default': 5.0, 'level': 0, 'description': 'X offset between IR and depth images', 'min': -10.0, 'max': 10.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'depth_ir_offset_y', 'type': 'double', 'default': 4.0, 'level': 0, 'description': 'Y offset between IR and depth images', 'min': -10.0, 'max': 10.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'z_offset_mm', 'type': 'int', 'default': 0, 'level': 0, 'description': 'Z offset in mm', 'min': -200, 'max': 200, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'z_scaling', 'type': 'double', 'default': 1.0, 'level': 0, 'description': 'Scaling factor for depth values', 'min': 0.5, 'max': 1.5, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}], 'groups': [], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

OpenNI_SXGA_15Hz = 1
OpenNI_VGA_30Hz = 2
OpenNI_VGA_25Hz = 3
OpenNI_QVGA_25Hz = 4
OpenNI_QVGA_30Hz = 5
OpenNI_QVGA_60Hz = 6
OpenNI_QQVGA_25Hz = 7
OpenNI_QQVGA_30Hz = 8
OpenNI_QQVGA_60Hz = 9