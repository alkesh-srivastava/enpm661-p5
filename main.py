import utils
from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM

OBSTACLE = 255
UNOCCUPIED = 0

if __name__ == '__main__':
    x_dim = 100
    y_dim = 100

    r_1position = list()
    r2_position = list()
    #############################     ROBOT-1     ###############################################
    print("Input Start Position for Robot 1 :\nX Co-ordinate :")
    testudo_x = int(input())
    if testudo_x > 100 or testudo_x < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()
    print("Y Co-ordinate :")
    testudo_y = int(input())
    if testudo_y > 100 or testudo_y< 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()
    if utils.obstacle(testudo_x, testudo_y):
        print("The start is on a obstacle. RETRY!!")
        raise SystemExit()

    print("Input Goal Position for Robot 1 :\nX Co-ordinate :")
    goal_x = int(input())
    if goal_x > 100 or goal_x < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()

    print("Y Co-ordinate :")
    goal_y = int(input())

    if goal_y > 100 or goal_y < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()

    if utils.obstacle(goal_x, goal_y):
        print("The goal is on a obstacle. RETRY!!")
        raise SystemExit()

    start = (99 - testudo_y, testudo_x)
    goal = (99 - goal_y, goal_x)

    #############################     ROBOT-2     ###############################################

    print("Input Start Position for Robot 2 :\nX Co-ordinate :")
    testudo2_x = int(input())
    if testudo2_x > 100 or testudo2_x < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()
    print("Y Co-ordinate :")
    testudo2_y = int(input())
    if testudo2_y > 100 or testudo2_y < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()
    if utils.obstacle(testudo2_x, testudo2_y):
        print("The start is on a obstacle. RETRY!!")
        raise SystemExit()

    print("Input Goal Position for Robot 2 :\nX Co-ordinate :")
    goal2_x = int(input())
    if goal2_x > 100 or goal2_x < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()

    print("Y Co-ordinate :")
    goal2_y = int(input())

    if goal2_y > 100 or goal2_y < 0:
        print("Your input contains an obstacle or there was an invalid entry.\nPLEASE RESTART THE PROGRAM!")
        raise SystemExit()

    if utils.obstacle(goal2_x, goal2_y):
        print("The goal is on a obstacle. RETRY!!")
        raise SystemExit()

    start_r2 = (99 - testudo2_y, testudo2_x)
    goal_r2 = (99 - goal2_y, goal2_x)
    view_range = 100
    r_1position.append(start)
    r2_position.append(start_r2)
    gui = Animation(title="Project 5 - Namala & Srivastava",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    start=start,
                    goal=goal,
                    start2=start_r2,
                    goal2=goal_r2,
                    viewing_range=view_range)

    new_map = gui.world
    old_map = new_map
    new_map2 = gui.world
    old_map2 = new_map2
    new_position = start
    last_position = start

    # new_observation = None
    # type = OBSTACLE

    # D* Lite (optimized)
    dstar = DStarLite(map=new_map,
                      s_start=start,
                      s_goal=goal)
    dstar2 = DStarLite(map=new_map2,
                      s_start=start_r2,
                      s_goal=goal_r2)
    new_position2 = start_r2
    last_position2 = start_r2
    # SLAM to detect vertices
    slam = SLAM(map=new_map,
                view_range=view_range)
    slam2 = SLAM(map=new_map2,
                view_range=view_range)
    # move and compute path
    path, g, rhs = dstar.move_and_replan(robot_position=new_position)
    path2, g2, rhs2 = dstar2.move_and_replan(robot_position=new_position2)
    counter = -1
    while not gui.done:

        # update the map
        # print(path)
        # drive gui
        gui.run_game(path=path, path2=path2)

        new_position = gui.current
        new_position2 = gui.current2
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
            r_1position.append(new_position)
            counter = counter + 1
            print("Counter:", counter)
            f = open("position.txt", "w")
            f.write(str(counter))
            f.close()
            f = open("R1_Path.txt", "w")
            for each in r_1position:
                f.write(str(each)+"!")
            f.close()
            last_position = new_position

            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

            # d star
            path, g, rhs = dstar.move_and_replan(robot_position=new_position)
        if new_position2!= last_position2:
            r2_position.append(new_position2)
            f = open("R2_Path.txt", "w")
            for each in r2_position:
                f.write(str(each)+"!")

            f.close()
            last_position2 = new_position2

            # slam
            new_edges_and_old_costs2, slam_map2 = slam2.rescan(global_position=new_position2)

            dstar2.new_edges_and_old_costs = new_edges_and_old_costs2
            dstar2.sensed_map = slam_map2

            # d star
            path2, g2, rhs2 = dstar2.move_and_replan(robot_position=new_position2)

