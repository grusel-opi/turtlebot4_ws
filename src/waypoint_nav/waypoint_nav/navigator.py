import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from math import atan2, pi, sqrt
import numpy as np


def quat_to_euler(q):

    x, y, z, w = q[0], q[1], q[2], q[3]
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = sqrt(1 + 2 * (w * y - x * z))
    cosp = sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * atan2(sinp, cosp) - pi / 2

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def get_table_star_pattern_poses():

    orientations = np.array([
        [0.0, 0.0, -0.23960277223372525, 0.9708710066419295],
        [0.0, 0.0, -0.16760538653097257, 0.9858541648772415],
        [0.0, 0.0, -0.007654370100478947, 0.9999707048800804],
        [0.0, 0.0, -0.020348112114218008, 0.9997929557330294],
        [0.0, 0.0, 0.036883672671461344, 0.9993195658498158],
        [0.0, 0.0, 0.07136405733291329, 0.9974503352653628],
        [0.0, 0.0, 0.2347456285699664, 0.9720568346898711],
        [0.0, 0.0, 0.2835574140031261, 0.9589552611899367]
    ])

    positions = np.array([
        [-2.042957305908203, 1.0581555366516113, 0.0],   
        [-0.10832256078720093, 0.03204554319381714, 0.0],
        [-1.9471354484558105, 0.04899853467941284, 0.0], 
        [-0.1241503655910492, -0.02994781732559204, 0.0],
        [-1.9590061902999878, -0.7734025120735168, 0.0], 
        [0.02765178680419922, -0.7057451605796814, 0.0], 
        [-1.6529576778411865, -1.6361885070800781, 0.0], 
        [-0.1291159987449646, -1.0230827331542969, 0.0], 
    ])

    return positions[:, :2], [quat_to_euler(q) for q in orientations] # positions (x, y), orientation (x, y, z, w)


def main():
    rclpy.init()

    naviagator = TurtleBot4Navigator()

    naviagator.info("Checking dock status")

    if not naviagator.getDockedStatus():
        naviagator.info("Docking before init..")
        naviagator.dock()

    initial_pose = naviagator.getPoseStamped([0.0, 0.4], TurtleBot4Directions.NORTH)
    naviagator.setInitialPose(initial_pose)

    naviagator.info("Waiting for nav2 to become active")

    naviagator.waitUntilNav2Active()

    naviagator.info("Nav2 ready, lets go!")

    positions, orientations = get_table_star_pattern_poses()
    
    goals = []

    for p, o in zip(positions, orientations):
        goals.append(naviagator.getPoseStamped(p, o[2]))

    goals.append(initial_pose)

    naviagator.undock()

    naviagator.startFollowWaypoints(goals)

    naviagator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
