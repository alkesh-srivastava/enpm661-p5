import pygame
import time
import numpy as np
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM
import matplotlib.pyplot as plt  # plot, title, xlabel, ylabel, savefig, legend
from numpy import array
from typing import List

# Define some colors
from utils import obstacle
BLACK = (0, 0, 0)  # BLACK
# UNOCCUPIED = (255, 255, 255)  # WHITE
UNOCCUPIED = (0, 0, 0)  # BLACK
GOAL = (0, 255, 0)  # GREEN
GOAL2 = (255, 20, 147)  # DEEP PINK
START = (255, 0, 0)  # RED
START2 = (119, 136, 153)  # Slight Gray
GRAY1 = (145, 145, 102)  # GRAY1
# OBSTACLE = (77, 77, 51)  # GRAY2
OBSTACLE = (255, 255, 0)  # GRAY2
LOCAL_GRID = (0, 0, 80)  # BLUE

colors = {
    0: UNOCCUPIED,
    1: GOAL,
    255: OBSTACLE
}


class Animation:
    def __init__(self,
                 title="Project 5 - Namala & Srivastava",
                 width=10,
                 height=10,
                 margin=0,
                 x_dim=100,
                 y_dim=50,
                 start=(0, 0),
                 goal=(50, 50),
                 start2=(0, 0),
                 goal2=(50, 50),
                 viewing_range=3):

        self.width = width
        self.height = height
        self.margin = margin
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.start = start
        self.current = start
        self.start2 = start2
        self.current2 = start2
        self.observation = {"pos": None, "type": None}
        self.observation2 = {"pos": None, "type": None}
        self.goal = goal
        self.goal2 = goal2
        self.viewing_range = viewing_range

        pygame.init()

        # Set the 'width' and 'height' of the screen
        window_size = [(width + margin) * y_dim + margin,
                       (height + margin) * x_dim + margin]

        self.screen = pygame.display.set_mode(window_size)

        self.world = OccupancyGridMap(x_dim=x_dim,
                                      y_dim=y_dim,
                                      exploration_setting='8N')

        # Set title of screen
        pygame.display.set_caption(title)

        # set font
        pygame.font.SysFont('Comic Sans MS', 36)

        # Loop until the user clicks the close button
        self.done = False

        # used to manage how fast the screen updates
        self.clock = pygame.time.Clock()

    def get_position(self):
        return self.current, self.current2

    def set_position(self, pos: (int, int), pos2: (int, int)):
        self.current = pos
        self.current2 = pos2

    def get_goal(self):
        return self.goal

    def set_goal(self, goal: (int, int)):
        self.goal = goal

    def set_start(self, start: (int, int)):
        self.start = start

    def display_path(self, path=None):
        if path is not None:
            for step in path:
                # draw a moving robot, based on current coordinates
                step_center = [round(step[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                               round(step[0] * (self.height + self.margin) + self.height / 2) + self.margin]

                # draw robot position as red circle
                pygame.draw.circle(self.screen, START, step_center, round(self.width / 2) - 2)

    def display_path2(self, path2=None):
        if path2 is not None:
            for step in path2:
                # draw a moving robot, based on current coordinates
                step_center = [round(step[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                               round(step[0] * (self.height + self.margin) + self.height / 2) + self.margin]

                # draw robot position as red circle
                pygame.draw.circle(self.screen, START2, step_center, round(self.width / 2) - 2)

    def display_obs(self, observations=None):
        if observations is not None:
            for o in observations:
                pygame.draw.rect(self.screen, GRAY1, [(self.margin + self.width) * o[1] + self.margin,
                                                      (self.margin + self.height) * o[0] + self.margin,
                                                      self.width,
                                                      self.height])

    def collision_detector(self, path, path2):
        for i in range(-5 + path[0][0], 5 + path[0][0]):
            for j in range(-5 + path[0][1], 5 + path[0][1]):
                if (i, j) in path2[:5]:
                    return True
        return False

    def run_game(self, path=None, path2=None):
        for i in range(self.x_dim):
            for j in range(self.y_dim):
                if obstacle(i, j):
                    grid_cell = (i, j)
                    self.world.set_obstacle(grid_cell)
        ##### Put the collision checker
        if self.collision_detector(path=path, path2=path2):
            print("Collision Detected")
            f = open("position.txt", "r")
            m = int(f.read())
            plt.figure()
            x_dim = m + len(path)
            y_dim = m + len(path2)
            obstacle_list = list()

            coordination_map = OccupancyGridMap(x_dim, y_dim)
            for i in range(-5 + m, 5 + m):
                for j in range(-5 + m, 5 + m):
                    coordination_map.set_obstacle((i, j))
                    obstacle_list.append((i, j))
            obstacle_list = np.array(obstacle_list)
            slam = SLAM(map=coordination_map,
                        view_range=max(x_dim, y_dim))

            dstar = DStarLite(map=coordination_map, s_start=(0, 0), s_goal=(x_dim - 1, y_dim - 1))
            coordinationPath, g, rhs = dstar.move_and_replan(robot_position=(0, 0))
            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=(0, 0))
            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map
            coordinationPath1, g, rhs = dstar.move_and_replan(robot_position=(0, 0))

            coordinationPath = np.array(coordinationPath1)
            print("Coordination Path", coordinationPath)
            print("Obstacle LIST", obstacle_list)

            plt.plot(coordinationPath[:, 0], coordinationPath[:, 1], '-b')
            plt.plot(obstacle_list[:, 0], obstacle_list[:, 1], '.r')

            plt.xlabel('Robot 1 Path Steps')
            plt.ylabel('Robot 2 Path Steps')
            plt.show()

            robot1_movements = coordinationPath[:, 0]
            robot2_movements = coordinationPath[:, 1]

            occurrences1 = list()
            occurrences2 = list()
            for i in range(min(robot1_movements), max(robot1_movements)):
                occurrences1.append(np.count_nonzero(robot1_movements == i))
            for i in range(min(robot2_movements), max(robot2_movements)):
                occurrences2.append(np.count_nonzero(robot2_movements == i))

            robot1_stops = list()
            for a in range(0,len(occurrences1)):
                if occurrences1[a]>1:
                    robot1_stops.append((a, occurrences1[a]))

            robot2_stops = list()
            for a in range(0, len(occurrences2)):
                if occurrences2[a] > 1:
                    robot2_stops.append((a, occurrences2[a]))
            print(robot1_stops)
            print(robot2_stops)
            # The remaining path of robot 1 and robot 2
            f = open("R1_Path.txt", "a")
            for each in path:
                f.write(str(each)+"!")
            f.close()
            f = open("R2_Path.txt", "a")
            for each in path2:
                f.write(str(each)+"!")
            f.close()
            f = open("R1_Path.txt", 'r')
            r1_path = f.read()
            f.close()
            f = open("R2_Path.txt", 'r')
            r2_path = f.read()
            f.close()
            r1_path_list = r1_path.split('!')
            r2_path_list = r2_path.split('!')
            r1_plot_list = list()
            r2_plot_list = list()
            for each in r1_path_list:
                try:
                    temp = each[1:-1].split(', ')
                    r1_plot_list.append((int(temp[0]),int(temp[1])))
                except:
                    pass
            for each in r2_path_list:
                try:
                    temp = each[1:-1].split(', ')
                    r2_plot_list.append((int(temp[0]),int(temp[1])))
                except:
                    pass
            main_path_r1 = list()
            main_path_r2 = list()
            if len(robot1_stops)>0:
                for each in robot1_stops:
                    index = each[0]
                    iterations = each[1]
                    counter = -1
                    for anothereach in r1_plot_list:
                        counter = counter + 1
                        main_path_r1.append(anothereach)
                        if counter == index:
                            for i in range(iterations - 1):
                                main_path_r1.append(anothereach)
                            break
            if len(robot2_stops) == 0:
                main_path_r1 = r1_plot_list
            if len(robot2_stops) > 0:
                index_list_r2 = list()
                iteration_list_r2 =list()
                for each in robot2_stops:
                    index_list_r2.append(each[0])
                    iteration_list_r2.append(each[1])
                counter = -1
                for anothereach in r2_plot_list:
                    counter = counter + 1
                    main_path_r2.append(anothereach)
                    if counter in index_list_r2:
                        for i in range(0, iteration_list_r2[index_list_r2.index(counter)]-1):
                            main_path_r2.append(anothereach)


            if len(robot2_stops) == 0:
                main_path_r2 = r2_plot_list

            print("######")
            print(r1_plot_list)
            print("######")
            print(r2_plot_list)
            print("MAIN1\n")
            print(main_path_r1)
            print("MAIN2\n")
            print(main_path_r2)



            raise SystemExit()
            # # x_dim = len(path)+ len(FILE)
            # # y_dim = len(path2)+ len(FILE2)
            # #### D STAR AGAIN :
            #         '''
            #         coordination_map = OccupancyGridMap(x_dim, y_dim)
            #         dstar = DStarLite(map=coordination_map,
            #         s_start=(0,0),
            #         s_goal=(xdim-1, ydim-1))
            #         path, g, rhs = dstar.move_and_replan(robot_position=s_start)
            #       '''
        #### X_DIM = length of R1
        ##### Y_DIM = length of R2
        pygame.font.init()  # you have to call this at the start,
        # if you want to use this module.
        myfont = pygame.font.SysFont('Times New Roman', 30)
        textsurface = myfont.render('Group 7: Alkesh, Prannoy', False, (119, 136, 153))

        if path is None:
            path = []
        if path2 is None:
            path2 = []

        grid_cell = None
        self.cont = False
        for event in pygame.event.get():

            if event.type == pygame.QUIT:  # if user clicked close
                print("quit")
                self.done = True  # flag that we are done so we can exit loop

            elif (event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE) or self.cont:

                f = open("R1_Path.txt", "a")
                f.write(str(path[0]) + ",")
                f.close()
                f = open("R2_Path.txt", "a")
                f.write(str(path2[0]) + ",")
                f.close()
                # space bar pressed. call next action
                if path:
                    if len(path) > 1:
                        (x, y) = path[1]
                    else:
                        (x, y) = path[0]
                if not path:
                    pass
                if path2:
                    if len(path2) > 1:
                        (x2, y2) = path2[1]
                    else:
                        (x2, y2) = path2[0]
                if not path2:
                    pass

                # for each in path: #R1
                #     if each not in path2 and not obstacle(each[0], each[1]): #R2
                #         for i in range(each[0] - 10, each[0] + 10):
                #             for j in range(each[1] - 10, each[1] + 10):
                #                 if i < 100 and j < 100:
                #                     if not self.world.is_unoccupied((i, j)) and not obstacle(i, j):
                #                         self.world.remove_obstacle((i, j))
                # for each in path:
                #     if each in path2:
                #         for i in range(each[0] - 2, each[0] + 2):
                #             for j in range(each[1] - 2, each[1] + 2):
                #                 if i < 100 and j < 100:
                #                     self.world.set_obstacle((i, j))
                # self.world.set_obstacle(each)

                self.set_position((x, y), (x2, y2))

                # for i in range(-1 * int(self.viewing_range), int(self.viewing_range)):
                #     for j in range(-1 * int(self.viewing_range), int(self.viewing_range)):
                #
                #         if obstacle(x + i, y + j):
                #             grid_cell = (x + i, y + j)
                #             self.world.set_obstacle(grid_cell)
                #         if obstacle(x2 + i, y2 + j):
                #             grid_cell = ((x2 + i, y2 + j))
                # self.world.set_obstacle(grid_cell)
            elif event.type == pygame.KEYUP and event.key == pygame.K_SPACE:
                pass

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_BACKSPACE:
                print("backspace automates the press space")
                if not self.cont:
                    self.cont = True
                else:
                    self.cont = False

            # set obstacle by holding left-click
            elif pygame.mouse.get_pressed()[0]:
                # User clicks the mouse. Get the position
                (col, row) = pygame.mouse.get_pos()

                # change the x/y screen coordinates to grid coordinates
                x = row // (self.height + self.margin)
                y = col // (self.width + self.margin)

                # turn pos into cell
                grid_cell = (x, y)

                # set the location in the grid map
                if self.world.is_unoccupied(grid_cell):
                    self.world.set_obstacle(grid_cell)
                    self.observation = {"pos": grid_cell, "type": OBSTACLE}

            # remove obstacle by holding right-click
            elif pygame.mouse.get_pressed()[2]:
                # User clicks the mouse. Get the position
                (col, row) = pygame.mouse.get_pos()

                # change the x/y screen coordinates to grid coordinates
                x = row // (self.height + self.margin)
                y = col // (self.width + self.margin)

                # turn pos into cell
                grid_cell = (x, y)

                # set the location in the grid map
                if not self.world.is_unoccupied(grid_cell):
                    print("grid cell: ".format(grid_cell))
                    self.world.remove_obstacle(grid_cell)
                    self.observation = {"pos": grid_cell, "type": UNOCCUPIED}

        # set the screen background
        self.screen.fill(BLACK)

        # draw the grid
        for row in range(self.x_dim):
            for column in range(self.y_dim):
                # color the cells
                pygame.draw.rect(self.screen, colors[self.world.occupancy_grid_map[row][column]],
                                 [(self.margin + self.width) * column + self.margin,
                                  (self.margin + self.height) * row + self.margin,
                                  self.width,
                                  self.height])

        self.display_path(path=path)
        self.display_path2(path2=path2)
        # fill in the goal cell with green
        pygame.draw.rect(self.screen, GOAL, [(self.margin + self.width) * self.goal[1] + self.margin,
                                             (self.margin + self.height) * self.goal[0] + self.margin,
                                             self.width,
                                             self.height])
        # fill in the goal cell with green
        pygame.draw.rect(self.screen, GOAL, [(self.margin + self.width) * self.goal[1] + self.margin,
                                             (self.margin + self.height) * self.goal[0] + self.margin,
                                             self.width,
                                             self.height])
        pygame.draw.rect(self.screen, GOAL2, [(self.margin + self.width) * self.goal2[1] + self.margin,
                                              (self.margin + self.height) * self.goal2[0] + self.margin,
                                              self.width,
                                              self.height])

        # draw a moving robot, based on current coordinates
        robot_center = [round(self.current[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                        round(
                            self.current[0] * (self.height + self.margin) + self.height / 2) + self.margin]
        robot_center2 = [round(self.current2[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                         round(
                             self.current2[0] * (self.height + self.margin) + self.height / 2) + self.margin]

        # draw robot position as red circle
        pygame.draw.circle(self.screen, START, robot_center, round(self.width / 2) - 2)

        # draw robot local grid map (viewing range)
        pygame.draw.rect(self.screen, LOCAL_GRID,
                         [robot_center[0] - self.viewing_range * (self.height + self.margin),
                          robot_center[1] - self.viewing_range * (self.width + self.margin),
                          2 * self.viewing_range * (self.height + self.margin),
                          2 * self.viewing_range * (self.width + self.margin)], 2)
        pygame.draw.rect(self.screen, LOCAL_GRID,
                         [robot_center2[0] - self.viewing_range * (self.height + self.margin),
                          robot_center2[1] - self.viewing_range * (self.width + self.margin),
                          2 * self.viewing_range * (self.height + self.margin),
                          2 * self.viewing_range * (self.width + self.margin)], 2)

        # set game tick
        self.clock.tick(20)
        self.screen.blit(textsurface, (self.screen.get_width() // 2 - 150, 0))

        # go ahead and update screen with that we've drawn
        pygame.display.flip()

    # be 'idle' friendly. If you forget this, the program will hang on exit
    pygame.quit()
