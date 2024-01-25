
import rclpy
from rclpy.duration import Duration
import sys, termios, tty, select, os
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml

# waypoint sets
waypoint_set_1 = yaml.safe_load('''
waypoints:
  - position:
      x: 0.8006443977355957
      y: 0.5491957664489746
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.0055409271259092485
      w: 0.9999846489454652
  - position:
      x: 1.8789787292480469
      y: 0.5389942526817322
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.010695864295550759
      w: 0.9999427976074288
  - position:
      x: 3.0792641639709473
      y: 0.6118782758712769
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.01899610435153287
      w: 0.9998195577300264
''')

waypoint_set_2 = yaml.safe_load('''
waypoints:
  - position:
      x: 3.8347740173339844
      y: 0.012513279914855957
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.7548200584119721
      w: 0.6559319167558071
  - position:
      x: 3.084421157836914
      y: -0.5701640844345093
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.9998472894684893
      w: 0.01747563282157926
  - position:
      x: 2.19096302986145
      y: -0.609535813331604
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.9999322787364863
      w: 0.011637780753125607
''')


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """

Press q to command robot to go to waypoint set 1
Press w to command robot to go to waypoint set 2
Press s to force (emergency stop) robot

CTRL-C to quit
"""

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_key_commander')

    navigator = BasicNavigator()

    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        # linear position
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        # quaternions
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]

        return pose

    def start_navigation(waypoints):
        goal_poses = list(map(create_pose, waypoints["waypoints"]))

        # Wait for navigation to fully activate, since autostarting nav2
        navigator.waitUntilNav2Active(localizer="smoother_server")

        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_poses)

        i = 0
        while not navigator.isTaskComplete():
        
            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()

            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=600):
                    navigator.cancelTask()

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')


    try:
        print(msg)
        while rclpy.ok():
            key = getKey(settings)
            
            # emergency stop
            if key == 's':
                navigator.cancelTask()
            # navigate to waypoint set 1
            elif key == 'q': 
                start_navigation(waypoint_set_1)
            # navigate to waypoint set 2
            elif key == 'w':
                start_navigation(waypoint_set_2)
            elif key == '\x03':
                navigator.cancelTask()
                break   # Ctrl+C t exit

    except Exception as e:
        print(e)

    finally:
        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()