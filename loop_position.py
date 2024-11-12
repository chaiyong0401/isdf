import rospy
import moveit_commander
import time
from std_msgs.msg import Bool


rospy.init_node('periodic_motion_controller')
move_group = moveit_commander.MoveGroupCommander('panda_manipulator')  # 로봇의 MoveGroup 이름


period = 5.0

state_pub = rospy.Publisher('/robot/goal_reached',Bool,queue_size=10)
start_pub = rospy.Publisher('/isdf/start', Bool, queue_size =1)

joint_goals = [
    [-1.5, -1.0, 0.0, -2.5, 0.0, 2.5, 1.0],
    [-1.0, -1.0, 0.0, -2.5, 0.0, 2.5, 1.0],   
    [-0.5, -1.0, 0.0, -2.5, 0.0, 2.5, 1.0],   
    [0.0, -1.0, 0.0, -2.5, 0.0, 2.5, 1.0],
    [0.5, -1.0, 0.0, -2.5, 0.0, 2.5, 1.0],
    [1.0, -1.0, 0.0, -2.5, 0.0, 2.5, 1.0]
]

tolerance = 0.05

while not rospy.is_shutdown():
    new_start = True
    start_pub.publish(new_start)
    print("1")
    for joint_goal in joint_goals:
        move_group.set_joint_value_target(joint_goal)

        # 이동 명령 실행 (비동기식)
        move_group.go(wait=False)

        rate = rospy.Rate(100)  # 10Hz로 상태 확인

        while not rospy.is_shutdown():
            goal_reached = False
            # 현재 조인트 값 가져오기
            current_joints = move_group.get_current_joint_values()

            # 목표 도달 여부 판단
            goal_reached = all(
                abs(current - goal) < tolerance
                for current, goal in zip(current_joints, joint_goal)
            )


            # print("Current joints:", current_joints)
            # print("Goal joints:", joint_goal)
            # print("Goal reached:", goal_reached)

            state_pub.publish(goal_reached)


            if goal_reached:
                break

            rate.sleep()

        # 로봇 정지
        move_group.stop()

        # 다음 목표로 이동하기 전 대기
        rospy.sleep(period)
        
        
