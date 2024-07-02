import threading
import numpy as np


class CollisionMapCalculator:
    def __init__(self, agent_num, agents_possibility_map, collision_map, max_depth):
        self.agent_num = agent_num
        # 충돌 맵 데이터 구조를 초기화합니다. 예를 들어:
        self.collision_maps = [None] * agent_num
        self.threads = []
        self.agents_possibility_map = agents_possibility_map
        self.collision_map = collision_map
        self.max_depth = max_depth

    def calculate_collision_map(self, agent_index):
        for t in range(self.max_depth):
            # 모든 에이전트의 현재 시간 t에서의 맵 # (agent_num, height, width)
            all_agents_map = self.agents_possibility_map[:, t, :, :]

            # 현재 에이전트를 제외 # (agent_num-1, height, width)
            all_agents_map_except_current = np.delete(
                all_agents_map, agent_index, axis=0
            )

            # 다른 에이전트의 최대값 # (height, width)
            other_agents_max = np.max(all_agents_map_except_current, axis=0)

            # 최소값 계산 및 누적 합산 # (height, width) 더해주기
            self.collision_map[agent_index, :, :] += np.minimum(
                self.agents_possibility_map[agent_index, t, :, :], other_agents_max
            )

    def apply_collision_map_multithreaded(self):
        for i in range(self.agent_num):
            thread = threading.Thread(target=self.calculate_collision_map, args=(i,))
            self.threads.append(thread)
            thread.start()

        # 모든 스레드가 종료될 때까지 기다립니다.
        for thread in self.threads:
            thread.join()
