import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    naviagator = TurtleBot4Navigator()

    naviagator.info("Checking dock status")

    if not naviagator.getDockedStatus():
        naviagator.info("Docking before init..")
        naviagator.dock()

    initial_pose = naviagator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    naviagator.setInitialPose(initial_pose)

    naviagator.info("Waiting for nav2 to become active")

    naviagator.waitUntilNav2Active()

    naviagator.info("Nav2 ready, lets go!")

    goals = []
    goals.append(naviagator.getPoseStamped([-0.96, -0.92], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-0.96, -0.92], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-0.96, -0.92], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-0.96, -0.92], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-0.96, -2.66], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-0.96, -2.66], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-0.96, -2.66], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-0.96, -2.66], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-0.9, -4.95], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-0.9, -4.95], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-0.9, -4.95], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-0.9, -4.95], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-2.5, -1.75], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-2.5, -1.75], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-2.5, -1.75], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-2.5, -1.75], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-4.96, -2.0], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-4.96, -2.0], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-4.96, -2.0], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-4.96, -2.0], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-5.2, -4.45], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-5.2, -4.45], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-5.2, -4.45], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-5.2, -4.45], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-7.3, -2.0], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-7.3, -2.0], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-7.3, -2.0], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-7.3, -2.0], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-9.85, -5.2], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-9.85, -5.2], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-9.85, -5.2], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-9.85, -5.2], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-9.7, -3.5], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-9.7, -3.5], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-9.7, -3.5], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-9.7, -3.5], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-9.7, -1.27], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-9.7, -1.27], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-9.7, -1.27], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-9.7, -1.27], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-9.7, 0.33], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-9.7, 0.33], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-9.7, 0.33], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-9.7, 0.33], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-7.2, 0.53], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-7.2, 0.53], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-7.2, 0.53], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-7.2, 0.53], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-4.9, 0.85], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-4.9, 0.85], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-4.9, 0.85], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-4.9, 0.85], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-2.17, 0.85], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-2.17, 0.85], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-2.17, 0.85], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-2.17, 0.85], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-0.56, -0.02], TurtleBot4Directions.NORTH))
    goals.append(naviagator.getPoseStamped([-0.56, -0.02], TurtleBot4Directions.EAST))
    goals.append(naviagator.getPoseStamped([-0.56, -0.02], TurtleBot4Directions.SOUTH))
    goals.append(naviagator.getPoseStamped([-0.56, -0.02], TurtleBot4Directions.WEST))

    goals.append(naviagator.getPoseStamped([-0.2, 0.0], TurtleBot4Directions.NORTH))
    
    naviagator.undock()

    naviagator.startFollowWaypoints(goals)

    naviagator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
