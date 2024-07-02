import threading
from scipy import ndimage


class GaussianFilter:
    def __init__(self, agent_num, sigma, agents_possibility_map):
        self.agent_num = agent_num
        self.sigma = sigma
        self.agents_possibility_map = agents_possibility_map
        self.threads = []

    def gaussian_filter(self, index):
        self.agents_possibility_map[index] = ndimage.gaussian_filter(
            self.agents_possibility_map[index], sigma=self.sigma, mode="constant"
        )

    def apply_filter_multithreaded(self):
        for i in range(self.agent_num):
            thread = threading.Thread(target=self.gaussian_filter, args=(i,))
            self.threads.append(thread)
            thread.start()

        # 모든 스레드가 종료될 때까지 기다립니다.
        for thread in self.threads:
            thread.join()
