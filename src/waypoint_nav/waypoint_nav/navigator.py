import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


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

    goals = []

    goals.append(naviagator.getPoseStamped([-0.98, 0.42], 0  ))
    goals.append(naviagator.getPoseStamped([-0.98, 0.42], 45 ))
    goals.append(naviagator.getPoseStamped([-0.98, 0.42], 90 ))
    goals.append(naviagator.getPoseStamped([-0.98, 0.42], 135))
    goals.append(naviagator.getPoseStamped([-0.98, 0.42], 180))
    goals.append(naviagator.getPoseStamped([-0.98, 0.42], 225))
    goals.append(naviagator.getPoseStamped([-0.98, 0.42], 270))
    goals.append(naviagator.getPoseStamped([-0.98, 0.42], 315))

    goals.append(naviagator.getPoseStamped([-0.97, -0.58], 0  ))
    goals.append(naviagator.getPoseStamped([-0.97, -0.58], 45 ))
    goals.append(naviagator.getPoseStamped([-0.97, -0.58], 90 ))
    goals.append(naviagator.getPoseStamped([-0.97, -0.58], 135))
    goals.append(naviagator.getPoseStamped([-0.97, -0.58], 180))
    goals.append(naviagator.getPoseStamped([-0.97, -0.58], 225))
    goals.append(naviagator.getPoseStamped([-0.97, -0.58], 270))
    goals.append(naviagator.getPoseStamped([-0.97, -0.58], 315))

    goals.append(naviagator.getPoseStamped([-0.93, -2.04], 0  ))
    goals.append(naviagator.getPoseStamped([-0.93, -2.04], 45 ))
    goals.append(naviagator.getPoseStamped([-0.93, -2.04], 90 ))
    goals.append(naviagator.getPoseStamped([-0.93, -2.04], 135))
    goals.append(naviagator.getPoseStamped([-0.93, -2.04], 180))
    goals.append(naviagator.getPoseStamped([-0.93, -2.04], 225))
    goals.append(naviagator.getPoseStamped([-0.93, -2.04], 270))
    goals.append(naviagator.getPoseStamped([-0.93, -2.04], 315))

    goals.append(naviagator.getPoseStamped([-0.98, -2.97], 0  ))
    goals.append(naviagator.getPoseStamped([-0.98, -2.97], 45 ))
    goals.append(naviagator.getPoseStamped([-0.98, -2.97], 90 ))
    goals.append(naviagator.getPoseStamped([-0.98, -2.97], 135))
    goals.append(naviagator.getPoseStamped([-0.98, -2.97], 180))
    goals.append(naviagator.getPoseStamped([-0.98, -2.97], 225))
    goals.append(naviagator.getPoseStamped([-0.98, -2.97], 270))
    goals.append(naviagator.getPoseStamped([-0.98, -2.97], 315))

    goals.append(naviagator.getPoseStamped([-0.95, -4.2], 0  ))
    goals.append(naviagator.getPoseStamped([-0.95, -4.2], 45 ))
    goals.append(naviagator.getPoseStamped([-0.95, -4.2], 90 ))
    goals.append(naviagator.getPoseStamped([-0.95, -4.2], 135))
    goals.append(naviagator.getPoseStamped([-0.95, -4.2], 180))
    goals.append(naviagator.getPoseStamped([-0.95, -4.2], 225))
    goals.append(naviagator.getPoseStamped([-0.95, -4.2], 270))
    goals.append(naviagator.getPoseStamped([-0.95, -4.2], 315))

    goals.append(naviagator.getPoseStamped([-0.95, -5.2], 0  ))
    goals.append(naviagator.getPoseStamped([-0.95, -5.2], 45 ))
    goals.append(naviagator.getPoseStamped([-0.95, -5.2], 90 ))
    goals.append(naviagator.getPoseStamped([-0.95, -5.2], 135))
    goals.append(naviagator.getPoseStamped([-0.95, -5.2], 180))
    goals.append(naviagator.getPoseStamped([-0.95, -5.2], 225))
    goals.append(naviagator.getPoseStamped([-0.95, -5.2], 270))
    goals.append(naviagator.getPoseStamped([-0.95, -5.2], 315))

    goals.append(initial_pose)

    naviagator.undock()

    naviagator.startFollowWaypoints(goals)

    naviagator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
