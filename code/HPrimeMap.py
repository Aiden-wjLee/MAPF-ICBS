from a_star_class import A_Star, get_location, get_sum_of_cost, compute_heuristics
from Voronoi import Voronoi
from GaussianFilter import GaussianFilter
from CollisionMapCalculator import CollisionMapCalculator

import numpy as np
from collections import deque
from scipy import ndimage
import time


class HPrimeMap:
    def __init__(
        self,
        my_map,
        starts,
        goals,
        agent_num,
        sigma,
        approximated=True,
        voronoi_TF=True,
        random_ratio=0.0,
        zero_ratio=0.0,
    ):
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
        self.approximated = approximated
        self.voronoi_TF = voronoi_TF
        self.random_ratio = random_ratio
        self.zero_ratio = zero_ratio
        print("approximated", approximated)
        print("voronoi_TF", voronoi_TF)

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
        self.collision_prob_map = np.zeros(
            (self.agent_num, self.height, self.width)
        )  # 0: no collision, 1: collision, normalize for each agent
        self.voronoi_map = np.zeros((self.agent_num, self.height, self.width))

        self.AStar = A_Star
        self.time1 = np.zeros(5)
        print("hihi")

    def calculate_collision_map(self, agent_index):
        other_agents_max = np.delete(self.agents_possibility_map, agent_index, axis=0)
        other_agents_max = np.max(other_agents_max, axis=0)  # (time, height, width)
        direction = np.array([[0, 1], [0, -1], [1, 0], [-1, 0]])
        direction_2 = np.array(
            [[0, 2], [1, 1], [2, 0], [1, -1], [0, -2], [-1, -1], [-2, 0], [-1, 1]]
        )
        temp_map = np.zeros((self.agent_num, self.max_depth, self.height, self.width))
        for i in range(self.height):
            for j in range(self.width):
                for d in direction:
                    next_i = np.clip(i + d[0], 0, self.height - 1)
                    next_j = np.clip(j + d[1], 0, self.width - 1)

                    temp_map[agent_index, :, i, j] = (
                        self.agents_possibility_map[agent_index, :, i, j]
                        * other_agents_max[:, next_i, next_j]
                    )
                    self.collision_prob_map[agent_index, i, j] += np.sum(
                        temp_map[agent_index, :, i, j]
                        * other_agents_max[:, next_i, next_j]
                    )
                    # self.collision_map[agent_index, i, j] += np.sum(
                    #     self.agents_possibility_map[agent_index, :, i, j]
                    #     * other_agents_max[:, next_i, next_j]
                    # )
                for d in direction_2:
                    next_i = np.clip(i + d[0], 0, self.height - 1)
                    next_j = np.clip(j + d[1], 0, self.width - 1)

                    temp_map[agent_index, :, i, j] = (
                        self.agents_possibility_map[agent_index, :, i, j]
                        * other_agents_max[:, next_i, next_j]
                    )
                    self.collision_prob_map[agent_index, i, j] += np.sum(
                        temp_map[agent_index, :, i, j]
                        * other_agents_max[:, next_i, next_j]
                    )
        # self.collision_map[agent_index] += self.collision_map_wall
        # * np.sum(
        #     self.agents_possibility_map[agent_index], axis=0
        # )

        #         temp_map[agent_index, t, :, :] = (
        #     self.agents_possibility_map[agent_index, t, :, :] * other_agents_max
        # )
        # for t in range(self.max_depth):
        #     # 모든 에이전트의 현재 시간 t에서의 맵 # (agent_num, height, width)
        #     start_time = time.time()
        #     all_agents_map = self.agents_possibility_map[:, t, :, :]
        #     self.time1[0] += time.time() - start_time

        #     start_time = time.time()
        #     # 현재 에이전트를 제외 # (agent_num-1, height, width)
        #     all_agents_map_except_current = np.delete(
        #         all_agents_map, agent_index, axis=0
        #     )
        #     self.time1[1] += time.time() - start_time

        #     start_time = time.time()
        #     # 다른 에이전트의 최대값 # (height, width)
        #     other_agents_max = np.max(all_agents_map_except_current, axis=0)
        #     self.time1[2] += time.time() - start_time

        #     # 최소값 계산 및 누적 합산 # (height, width) 더해주기
        #     # temp_map = np.minimum(
        #     #     self.agents_possibility_map[agent_index, t, :, :], other_agents_max
        #     # )

        #     start_time = time.time()
        #     # self.collision_map[agent_index, :, :] += np.minimum(
        #     #     self.agents_possibility_map[agent_index, t, :, :], other_agents_max
        #     # )
        #     temp_map[agent_index, t, :, :] = (
        #         self.agents_possibility_map[agent_index, t, :, :] * other_agents_max
        #     )
        #     self.collision_map[agent_index, :, :] += temp_map[agent_index, t, :, :]
        #     self.time1[3] += time.time() - start_time
        # # self.collision_map[agent_index] = np.mean(temp_map[agent_index], axis=0)
        filtered_values = temp_map[agent_index][temp_map[agent_index] > 0.0]
        self.collision_prob_map[agent_index] = np.mean(filtered_values)
        # std_dev = np.std(filtered_vlues)
        # self.collision_std_map[agent_index] = np.std(temp_map[agent_index], axis=0)

        # self.collision_std_map[agent_index] = np.std(temp_map[agent_index], axis=0)

    def get_voronoi_map(self):
        v = Voronoi(
            self.starts,
            self.goals,
            self.agent_num,
            len(self.my_map),
            len(self.my_map[0]),
        )
        v.create_voronoi()
        voronoi_map = v.get_voronoi()
        ones_count = v.get_ones_count()

        self.voronoi_map = np.array(voronoi_map).astype(np.float32) / ones_count

    def get_collision_map(self):
        start_time_for_path = time.time()
        paths_numpy = []
        for i in range(self.agent_num):
            path_numpy = self.get_path_from_astar(i)[0]
            max_t_for_path = path_numpy.shape[0]
            paths_numpy.append(path_numpy)
            if max_t_for_path > self.max_depth:
                self.max_depth = max_t_for_path

        if self.approximated == True:
            print("path_numpy.shape", paths_numpy[1].shape)
            # 충돌 맵 초기화
            collision_map = {}

            # 각 에이전트의 경로를 순회하며 충돌 맵 생성
            for agent_id, path in enumerate(paths_numpy):
                for t, (x, y) in enumerate(path):
                    if (x, y) not in collision_map:
                        collision_map[(x, y)] = {}
                    if t not in collision_map[(x, y)]:
                        collision_map[(x, y)][t] = []
                    collision_map[(x, y)][t].append(agent_id)

            # Conflict 감지
            vertex_conflicts = []
            edge_conflicts = []

            # Vertex conflict 감지
            for cell, time_agents in collision_map.items():
                for t, agents in time_agents.items():
                    if len(agents) > 1:
                        vertex_conflicts.append((cell, t, agents))

            # Edge conflict 감지
            for agent_id, path in enumerate(paths_numpy):
                for t in range(len(path) - 1):
                    current_pos = tuple(path[t])
                    next_pos = tuple(path[t + 1])
                    for other_agent_id, other_path in enumerate(paths_numpy):
                        if agent_id != other_agent_id and t < len(other_path) - 1:
                            other_current_pos = tuple(other_path[t])
                            other_next_pos = tuple(other_path[t + 1])
                            if (
                                current_pos == other_next_pos
                                and next_pos == other_current_pos
                            ):
                                edge_conflicts.append(
                                    (agent_id, other_agent_id, current_pos, next_pos, t)
                                )
            # Vertex conflicts를 collision_prob_map에 반영
            for cell, _, agents in vertex_conflicts:
                x, y = cell
                for agent_id in agents:
                    self.collision_prob_map[agent_id, y, x] = 1

            # Edge conflicts를 collision_prob_map에 반영
            for agent_id, other_agent_id, current_pos, next_pos, _ in edge_conflicts:
                self.collision_prob_map[agent_id, current_pos[1], current_pos[0]] = 1
                self.collision_prob_map[agent_id, next_pos[1], next_pos[0]] = 1
                self.collision_prob_map[
                    other_agent_id, current_pos[1], current_pos[0]
                ] = 1
                self.collision_prob_map[other_agent_id, next_pos[1], next_pos[0]] = 1
            for agent_id in range(self.agent_num):
                self.collision_prob_map[agent_id] = self.gaussian_filter(
                    self.collision_prob_map[agent_id], self.sigma
                )
        else:  # if self.approximated == False:
            self.agents_possibility_map = np.zeros(
                (self.agent_num, self.max_depth, self.height, self.width)
            )
            for i in range(self.agent_num):
                path_numpy = paths_numpy[i]
                # np.arange(path_numpy.shape[0]) : [0, 1, 2, ..., max_t_for_path-1], path_numpy[:, 0] : x, path_numpy[:, 1] : y
                self.agents_possibility_map[
                    i,
                    np.arange(path_numpy.shape[0]),
                    path_numpy[:, 0],
                    path_numpy[:, 1],
                ] = 1

            start_time_for_filtering = time.time()
            filter_obj = GaussianFilter(
                self.agent_num, self.sigma, self.agents_possibility_map
            )
            filter_obj.apply_filter_multithreaded()
            print("Time for filtering: ", time.time() - start_time_for_filtering)

            start_time_for_calculation = time.time()
            for i in range(self.agent_num):
                self.calculate_collision_map(i)
                if i == 0:
                    for j in range(self.time1.shape[0]):
                        print(f"time{j+1}: {self.time1[j]}")
            for i in range(self.time1.shape[0]):
                print(f"time{i+1}: {self.time1[i]}")
            print(
                "Time for collision map calculation: ",
                time.time() - start_time_for_calculation,
            )

            # self.collision_map = self.collision_map / np.max(self.collision_map)
            # self.collision_std_map = (self.collision_std_map) / np.max(
            #     self.collision_std_map
            # )
            # self.collision_std_map = np.max(self.collision_std_map) - self.collision_std_map

            # np.savetxt("collision_map.txt", self.collision_map[1], fmt="%f", delimiter=" ")

        for i in range(self.agent_num):
            print("max of collision map", np.max(self.collision_prob_map[i]))
            if np.max(self.collision_prob_map[i] > 0):
                self.collision_prob_map[i] = self.collision_prob_map[i] / np.max(
                    self.collision_prob_map[i]
                )

        for i in range(self.agent_num):
            if (
                np.max(self.collision_prob_map[i]) < 0
                or np.max(self.collision_prob_map[i]) > 1
            ):
                print("Collision map is not valid")
                exit()
            self.collision_prob_map[i, self.goals[i][0], self.goals[i][1]] = 0

        if self.voronoi_TF:
            self.get_voronoi_map()
            if self.collision_prob_map.shape != self.voronoi_map.shape:
                raise BaseException(
                    f"collision_map.shape: {self.collision_prob_map.shape} != voronoi.shape: {self.voronoi_map.shape}"
                )
            self.collision_prob_map += self.voronoi_map

        self.collision_prob_map = self.collision_prob_map / np.max(
            self.collision_prob_map
        )
        if self.random_ratio > 0.0:
            print("random_ratio", self.random_ratio)
            self.collision_prob_map = (
                self.collision_prob_map * (1 - self.random_ratio)
                + np.random.rand(self.agent_num, self.height, self.width)
                * self.random_ratio
            )
        if self.zero_ratio > 0.0:
            print("zero_ratio", self.zero_ratio)
            # zero_ratio에 따라 에이전트 선택
            num_zero_agents = int(self.agent_num * self.zero_ratio)
            zero_agents = np.random.choice(
                self.agent_num, num_zero_agents, replace=False
            )
            # 선택된 에이전트의 collision probability map을 0으로 설정
            for agent in zero_agents:
                self.collision_prob_map[agent] = 0.0

        return self.collision_prob_map

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
