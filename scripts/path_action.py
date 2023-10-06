import rospy
import actionlib
from girona_utils.msg import PathAction, PathGoal
from nav_msgs.msg import Path


def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient(
        'path_manager', PathAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("before the wait")
    client.wait_for_server()
    print("after the wait")

    # Creates a goal to send to the action server.
    goal = PathGoal()

    goal.path = rospy.wait_for_message("planner/path_result", Path)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('ation_path_test')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
