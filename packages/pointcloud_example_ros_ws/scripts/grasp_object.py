from hsrb_interface import Robot, geometry
import rospy
import geometry_msgs.msg

robot = Robot()
base = robot.try_get('omni_base')
whole_body = robot.try_get('whole_body')
target_pose = None

def pose_cb(msg):
    global target_pose
    target_pose = msg


def main():
    rospy.Subscriber("/detect_plane_node/object_pose", geometry_msgs.msg.PoseStamped, pose_cb)
    try:
        whole_body.move_to_neutral()
    except:
        rospy.logerr("Failed")
    rospy.loginfo("moved to go, waiting for pose")

    global target_pose
    while target_pose is None:
        rospy.sleep(1)
    print(target_pose.pose.position)
    base.go_rel(0, target_pose.pose.position.y - 0.078, 0)
    target_pose = None
    whole_body.move_to_neutral()
    whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': -0.3})
    while target_pose is None:
        rospy.sleep(1)
    print(target_pose.pose.position)
    rospy.loginfo("Got object pose!")
    pos = target_pose.pose.position
    hsr_pose = geometry.pose(0.287, 0.078, pos.z, 0.041, -1.569, 3.101)
    whole_body.move_end_effector_pose(pose, ref_frame_id='base_link')
    hsr_pose = geometry.pose(pos.x, 0.078, pos.z, 0.041, -1.569, 3.101)
    try:
        whole_body.move_end_effector_pose(pose, ref_frame_id='base_link')
    except:
        rospy.logerr("Failed to move")

    rospy.loginfo("move to pose")


if __name__ == "__main__":
    main()
