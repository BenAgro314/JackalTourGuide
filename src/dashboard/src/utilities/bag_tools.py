import rospy
import rosbag
import numpy
from ros_numpy import point_cloud2
from sensor_msgs.msg import PointCloud2 
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
import yaml

def bag_metadata(bagfile):
    res = yaml.load(bagfile._get_yaml_info())
    return res

def trajectory_to_array(traj):
    ''' given a list of PoseStamped messages, return a structured numpy array'''
    arr = []
    for msg in traj:
        pose = (msg.pose.position.x,msg.pose.position.y,msg.pose.position.z, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w, msg.header.stamp.to_sec())
        arr.append(pose)

    arr = numpy.array(arr, dtype = [('pos_x',numpy.double),('pos_y',numpy.double),('pos_z',numpy.double),('rot_x',numpy.double),('rot_y',numpy.double),('rot_z',numpy.double),('rot_w',numpy.double), ('time',numpy.double)])
    return arr


def num_messages(topic_name, bagfile):
    count = 0
    for topic,msg,t in bagfile.read_messages(topics = [topic_name]):
        count+=1
        
    
    return count

def read_pointcloud_frames(topic_name, bagfile):
    '''returns a list of tuples: (stamp,numpy_array) where stamp is the time and numpy_array is a labelled array with the pointcloud data'''
    frames = []

    for topic,msg,t in bagfile.read_messages(topics = [topic_name]):
        pc_array = point_cloud2.pointcloud2_to_array(msg)
        frames.append((msg.header.stamp, pc_array))

    return frames

def read_nav_odometry(topic_name, bagfile, nav_msg = True):
    '''returns an array of geometry_msgs/PoseStamped'''

    arr = []

    for topic,msg,t in bagfile.read_messages(topics = [topic_name]):
 
        if (not nav_msg):
            arr.append(msg)
        else: 
            geo_msg = PoseStamped()
            geo_msg.header = msg.header
            geo_msg.pose = msg.pose.pose
            arr.append(geo_msg)
            #pose = (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z, msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w, msg.header.stamp.to_sec())
            #arr.append(pose)

    #arr = numpy.array(arr, dtype = [('pos_x',numpy.double),('pos_y',numpy.double),('pos_z',numpy.double),('rot_x',numpy.double),('rot_y',numpy.double),('rot_z',numpy.double),('rot_w',numpy.double), ('time',numpy.double)])
    return arr

def read_tf_transform(parent_frame, child_frame, bagfile, static = False):
    ''' returns a list of time stamped transforms between parent frame and child frame '''
    arr = []
    if (static):
        topic_name = "/tf_static"
    else:
        topic_name = "/tf"

    for topic,msg,t in bagfile.read_messages(topics = [topic_name]):
        for transform in msg.transforms:
            if (transform.header.frame_id == parent_frame and transform.child_frame_id == child_frame):
                arr.append(transform)

    return arr

def read_action_result(topic_name, bagfile):
    ''' reads move_base action result info. Returns an array of the times and status of each message '''
    
    arr = []
    
    for topic,msg,t in bagfile.read_messages(topics = [topic_name]):
        arr.append((msg.header.stamp.to_sec(), msg.status.status))

    arr = numpy.array(arr, dtype = [('time', numpy.double), ('status', numpy.int8)])

    return arr
    
def transforms_to_trajectory(transforms):
    ''' converts a list of time stamped transforms to a list of pose stamped messages, where the pose is that of the child frame relative to it's parent'''
    traj = []
    for transform in transforms:
        geo_msg = PoseStamped()
        geo_msg.header = transform.header
        geo_msg.header.frame_id = transform.child_frame_id
        geo_msg.pose.position = transform.transform.translation
        geo_msg.pose.orientation = transform.transform.rotation
        traj.append(geo_msg)

    return traj


