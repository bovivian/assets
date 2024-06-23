import rclpy
import turtlebot3_move
import sys

SPEED = 0.4

def main(args=None):
    rclpy.init(args=args)

    cmdLine = sys.argv[1:]

    print("-------- Start --------")
    turtlebot_mover = turtlebot3_move.TurtleBotMover()

    try:
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(0)
        turtlebot_mover.move_by_distance(SPEED, 8.3)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(270)
        turtlebot_mover.move_by_distance(SPEED, 1.5)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(90)
        turtlebot_mover.move_by_distance(SPEED, 5.5)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(180)
        turtlebot_mover.move_by_distance(SPEED, 3.4)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(270)
        turtlebot_mover.move_by_distance(SPEED, 3.8)
        turtlebot_mover.print_gazebo_and_slam_positions()

        turtlebot_mover.turn_to_angle(180)
        turtlebot_mover.move_by_distance(SPEED, 4.0)
        turtlebot_mover.print_gazebo_and_slam_positions()
        
        # left side of house
        turtlebot_mover.turn_to_angle(90)
        turtlebot_mover.move_by_distance(SPEED, 3.2)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(180)
        turtlebot_mover.move_by_distance(SPEED, 5.2)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(-90)
        turtlebot_mover.move_by_distance(SPEED, 3.5)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(90)
        turtlebot_mover.move_by_distance(SPEED, 3.5)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(0)
        turtlebot_mover.move_by_distance(SPEED, 2.5)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(270)
        turtlebot_mover.move_by_distance(SPEED, 3.2)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(0)
        turtlebot_mover.move_by_distance(SPEED, 5.0)
        turtlebot_mover.print_gazebo_and_slam_positions()
        turtlebot_mover.turn_to_angle(270)

        file = '/home/ros/ws/slam_tests/'+ cmdLine[1] + '/out'
        turtlebot_mover.calculate_and_save_errors(file, cmdLine[0])

    except KeyboardInterrupt:
        pass

    turtlebot_mover.stop()
    print("-------- End --------")
    turtlebot_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
