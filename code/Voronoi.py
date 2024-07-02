import numpy as np


class Voronoi:
    def __init__(self, starts, goals, agent_num, width, height):
        self.starts = np.array(starts)
        self.goals = np.array(goals)
        self.agent_num = agent_num
        self.width = width
        self.height = height
        self.voronoi = np.zeros((agent_num, height, width), dtype=int)
        self.ones_count = np.zeros((height, width), dtype=int)

        self.EXIST = 1
        self.NOT_EXIST = 0

    def create_voronoi(self):
        for agent, (start, goal) in enumerate(zip(self.starts, self.goals)):
            if start[0] != goal[0]:
                m = (goal[1] - start[1]) / (goal[0] - start[0])
                b = start[1] - m * start[0]

                for x in range(self.width):
                    y = int(m * x + b)
                    if 0 <= y < self.height:
                        if start[0] < goal[0]:
                            self.voronoi[agent, y:, x] = self.EXIST
                            self.voronoi[agent, :y, x] = self.NOT_EXIST
                        else:
                            self.voronoi[agent, y:, x] = self.NOT_EXIST
                            self.voronoi[agent, :y, x] = self.EXIST
                    elif y < 0:
                        if start[0] < goal[0]:
                            self.voronoi[agent, :, x] = self.EXIST
                        else:
                            self.voronoi[agent, :, x] = self.NOT_EXIST
                    elif y >= self.height:
                        if start[0] < goal[0]:
                            self.voronoi[agent, :, x] = self.NOT_EXIST
                        else:
                            self.voronoi[agent, :, x] = self.EXIST
            else:
                # 수직선의 경우
                x = start[0]
                if start[1] < goal[1]:
                    self.voronoi[agent, :, x:] = self.EXIST
                    self.voronoi[agent, :, :x] = self.NOT_EXIST
                else:
                    self.voronoi[agent, :, x:] = self.NOT_EXIST
                    self.voronoi[agent, :, :x] = self.EXIST
        # 1의 개수 계산
        self.ones_count = np.sum(self.voronoi == self.EXIST, axis=0)

    def get_voronoi(self):
        return self.voronoi

    def get_ones_count(self):
        return self.ones_count


if __name__ == "__main__":
    # 사용 예시
    # starts = [(0, 0), (5, 5)]
    # goals = [(5, 5), (0, 0)]
    agent_num = 20
    starts = np.random.randint(0, 256, (agent_num, 2))
    goals = np.random.randint(0, 256, (agent_num, 2))
    width = 256
    height = 256

    v = Voronoi(starts, goals, agent_num, width, height)
    v.create_voronoi()

    voronoi_map = v.get_voronoi()
    ones_count = v.get_ones_count()

    print("Voronoi Map Shape:", voronoi_map.shape)
    print("Ones Count Shape:", ones_count.shape)

    # 각 에이전트의 Voronoi 맵 출력
    for i in range(agent_num):
        print(f"Agent {i} Voronoi Map:")
        print(voronoi_map[i])
        print()

    print("Ones Count:")
    print(ones_count)
