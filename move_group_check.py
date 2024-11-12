import rospy
import moveit_commander

# 초기화
rospy.init_node('move_group_list_node')
moveit_commander.roscpp_initialize([])

# RobotCommander 객체 생성
robot = moveit_commander.RobotCommander()

# move group 이름 가져오기
group_names = robot.get_group_names()
print("사용 가능한 Move Group들:", group_names)