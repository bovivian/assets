import rclpy
import turtlebot3_move
import sys

SPEED = 0.6

def main(args=None):
    rclpy.init(args=args)

    cmdLine = sys.argv[1:]

    print("-------- Start --------")
    turtlebot_mover = turtlebot3_move.TurtleBotMover()

    try:
        turtlebot_mover.print_gazebo_and_slam_positions()

        turtlebot_mover.turn_to_angle(45)
        turtlebot_mover.move_by_distance(SPEED, 10.0)
        turtlebot_mover.print_gazebo_and_slam_positions()

        turtlebot_mover.turn_to_angle(160)
        turtlebot_mover.move_by_distance(SPEED, 1.4)
        turtlebot_mover.print_gazebo_and_slam_positions()

        turtlebot_mover.turn_to_angle(180)
        turtlebot_mover.move_by_distance(SPEED, 10.0)
        turtlebot_mover.print_gazebo_and_slam_positions()

        turtlebot_mover.turn_to_angle(240)
        turtlebot_mover.move_by_distance(SPEED, 7.0)
        turtlebot_mover.print_gazebo_and_slam_positions()
        
        turtlebot_mover.turn_to_angle(270)
        turtlebot_mover.move_by_distance(SPEED, 3.0)
        turtlebot_mover.print_gazebo_and_slam_positions()
        
        turtlebot_mover.turn_to_angle(0)
        turtlebot_mover.move_by_distance(SPEED, 5.0)
        turtlebot_mover.print_gazebo_and_slam_positions()

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