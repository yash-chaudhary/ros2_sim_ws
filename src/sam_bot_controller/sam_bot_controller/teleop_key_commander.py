
import rclpy
from rclpy.duration import Duration
import sys, termios, tty, select, os
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml

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

    try:
        print(msg)
        while rclpy.ok():
            key = getKey(settings)
            
            # emergency stop
            if key == 's':
                print("typed s")
            # navigate to waypoint set 1
            elif key == 'q': 
                print("typed q")
            # navigate to waypoint set 2
            elif key == 'w':
                print("typed w")
            elif key == '\x03':
                break   # Ctrl+C t exit

    except Exception as e:
        print(e)

    finally:
        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()