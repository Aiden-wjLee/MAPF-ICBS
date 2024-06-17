import numpy as np
from collections import deque
from a_star_class import A_Star, get_location, get_sum_of_cost, compute_heuristics
from scipy import ndimage


class CollisionMap:
    def __init__(self, my_map, starts, goals, agent_num, sigma):
        """
        my_map: a 2D numpy array where True means there is an obstacle and False means the cell is empty
        starts: a list of tuples (x, y) representing the starting locations of the agents
        goals: a list of tuples (x, y) representing the goal locations of the agents
        """
        self.my_map = np.array(my_map)
        self.starts = starts
        self.goals = goals
        self.agent_num = agent_num
        self.sigma = sigma

        self.height = self.my_map.shape[0]
        self.width = self.my_map.shape[1]
        self.max_depth = max(self.height, self.width)

        # heuristics
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        self.heuristics = np.array(self.heuristics)

        # collision heuristicis
        self.agents_possibility_map = np.zeros(
            (self.agent_num, self.max_depth, self.height, self.width)
        )
        self.collision_map = np.zeros(
            (self.agent_num, self.height, self.width)
        )  # 0: no collision, 1: collision, normalize for each agent

        self.AStar = A_Star

        print("hihi")

    def calculate_collision_map(self, agent_index):
        for t in range(self.max_depth + 1):
            # 모든 에이전트의 현재 시간 t에서의 맵 # (agent_num, height, width)
            all_agents_map = self.agents_possibility_map[:, t, :, :]

            # 현재 에이전트를 제외 # (agent_num-1, height, width)
            all_agents_map_except_current = np.delete(
                all_agents_map, agent_index, axis=0
            )

            # 다른 에이전트의 최대값 # (height, width)
            other_agents_max = np.max(all_agents_map_except_current, axis=0)

            # 최소값 계산 및 누적 합산 # (height, width) 더해주기
            # temp_map += np.minimum(
            #     self.agents_possibility_map[agent_index, t, :, :], other_agents_max
            # )
            self.collision_map[agent_index, :, :] += np.minimum(
                self.agents_possibility_map[agent_index, t, :, :], other_agents_max
            )

    def get_collision_map(self):
        for i in range(self.agent_num):
            path_numpy = self.get_path_from_astar(i)[0]
            max_t_for_path = path_numpy.shape[0]
            if max_t_for_path > self.max_depth:
                self.expand_time_dimension(max_t_for_path)

            # np.arange(path_numpy.shape[0]) : [0, 1, 2, ..., max_t_for_path-1], path_numpy[:, 0] : x, path_numpy[:, 1] : y
            self.agents_possibility_map[
                i, np.arange(path_numpy.shape[0]), path_numpy[:, 0], path_numpy[:, 1]
            ] = 1

        # gaussian filter
        self.agents_possibility_map = self.gaussian_filter(
            self.agents_possibility_map, self.sigma
        )

        for i in range(self.agent_num):
            self.calculate_collision_map(i)

        # np.savetxt("collision_map.txt", self.collision_map[1], fmt="%f", delimiter=" ")
        # for i in range(self.agent_num):
        #     self.collision_map[i] = self.collision_map[i] / np.max(
        #         self.collision_map[i]
        #     )
        self.collision_map = self.collision_map / np.max(self.collision_map)

        return self.collision_map

    def expand_time_dimension(self, new_max_depth):
        if new_max_depth > self.agents_possibility_map.shape[1]:
            expanded_map = np.zeros(
                (self.agent_num, new_max_depth, self.height, self.width)
            )
            expanded_map[:, : self.agents_possibility_map.shape[1], :, :] = (
                self.agents_possibility_map
            )
            self.agents_possibility_map = expanded_map

    def get_path_from_astar(self, agent_index):
        """
        Get the path from start to goal using A* algorithm
        start: a tuple (x, y) representing the starting location
        goal: a tuple (x, y) representing the goal location
        Returns a list of tuples representing the path from start to goal
        """
        astar = self.AStar(
            self.my_map,
            self.starts,
            self.goals,
            self.heuristics,
            agent_index,
            [],
        )
        path = astar.find_paths()
        path_numpy = np.array(path)
        return path_numpy

    def gaussian_filter(self, A, sigma):
        filtered_A = ndimage.gaussian_filter(A, sigma=sigma, mode="constant")
        # 정규화
        # for i in range(A.shape[0]):
        #     filtered_A[i] = filtered_A[i] / np.sum(filtered_A[i])
        return filtered_A
