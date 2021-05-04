import utils
from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM

OBSTACLE = 255
UNOCCUPIED = 0

if __name__ == '__main__':
    x_dim = 100
    y_dim = 100
    print("Input Start Position for Testudo :\nX Co-ordinate :")
    testudo_x = int(input())
    if testudo_x > 100 or testudo_x < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()
    print("Y Co-ordinate :")
    testudo_y = int(input())
    if testudo_y > 100 or testudo_x < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()
    print("Input Goal Position for Testudo :\nX Co-ordinate :")
    goal_x = int(input())
    if goal_x > 100 or goal_x < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()
    print("Y Co-ordinate :")
    goal_y = int(input())

    if goal_y > 100 or goal_y < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()
    if utils.obstacle(testudo_x, testudo_y):
        print("The start is on a obstacle. RETRY!!")
        raise SystemExit()

    if utils.obstacle(goal_x, goal_y):
        print("The goal is on a obstacle. RETRY!!")
        raise SystemExit()
    print("Please input left RPM for your Robot :")
    left_rpm = int(input())
    print("Please input right RPM for your Robot :")
    right_rpm = int(input())
    # file = open('rpm.txt', 'w')
    # string = str(left_rpm)
    # string += ',' + str(right_rpm)
    # file.write(string)
    # file.close()
    start = (99 - testudo_y, testudo_x)
    file = open('start.txt', 'w')
    string = str(99 - testudo_y)
    string += ',' + str(testudo_x)
    string += ',' + str(0)
    file.write(string)
    file.close()
    goal = (99 - goal_y, goal_x)
    view_range = 10

    gui = Animation(title="D* Lite Path Planning",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    start=start,
                    goal=goal,
                    viewing_range=view_range)

    new_map = gui.world
    old_map = new_map

    new_position = start
    last_position = start

    # new_observation = None
    # type = OBSTACLE

    # D* Lite (optimized)
    dstar = DStarLite(map=new_map,
                      s_start=start,
                      s_goal=goal)

    # SLAM to detect vertices
    slam = SLAM(map=new_map,
                view_range=view_range)

    # move and compute path
    path, g, rhs = dstar.move_and_replan(robot_position=new_position)

    while not gui.done:
        # update the map
        # print(path)
        # drive gui
        gui.run_game(path=path)

        new_position = gui.current
        new_observation = gui.observation
        new_map = gui.world

        """
        if new_observation is not None:
            if new_observation["type"] == OBSTACLE:
                dstar.global_map.set_obstacle(pos=new_observation["pos"])
            if new_observation["pos"] == UNOCCUPIED:
                dstar.global_map.remove_obstacle(pos=new_observation["pos"])
        """

        if new_observation is not None:
            old_map = new_map
            slam.set_ground_truth_map(gt_map=new_map)

        if new_position != last_position:
            last_position = new_position

            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

            # d star
            path, g, rhs = dstar.move_and_replan(robot_position=new_position)
