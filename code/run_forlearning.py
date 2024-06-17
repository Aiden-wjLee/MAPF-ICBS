#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs_basic import CBSSolver  # original cbs with standard/disjoint splitting

# cbs with different improvements
from icbs_cardinal_bypass import ICBS_CB_Solver  # only cardinal dectection and bypass
from icbs_complete import ICBS_Solver  # all improvements including MA-CBS
from CollisionMap import CollisionMap

from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

import time


def print_mapf_instance(my_map, starts, goals):
    print("Start locations")
    print_locations(my_map, starts)
    print("Goal locations")
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ""
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + " "
            elif my_map[x][y]:
                to_print += "@ "
            else:
                to_print += ". "
        to_print += "\n"
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, "r")
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(" ")]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == "@" or cell == "T":
                my_map[-1].append(True)
            elif cell == ".":
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(" ")]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


def get_parser():
    parser = argparse.ArgumentParser(description="Runs various MAPF algorithms")
    parser.add_argument(
        "--instance",
        type=str,
        default=INSTANCE,
        help="The name of the instance file(s)",
    )
    parser.add_argument(
        "--batch",
        action="store_true",
        default=False,
        help="Use batch output instead of animation",
    )
    parser.add_argument(
        "--disjoint",
        action="store_true",
        default=False,
        help="Use the disjoint splitting",
    )
    parser.add_argument(
        "--hlsolver",
        type=str,
        default=HLSOLVER,
        help="The solver to use (one of: {CBS,ICBS_CB,ICBS}), defaults to "
        + str(HLSOLVER),
    )
    parser.add_argument(
        "--randomness",
        type=str,
        default="random",
        help="The randomness of the HL solver (one of: {random,biased}), defaults to random",
    )
    # parser.add_argument('--llsolver', type=str, default=LLSOLVER,
    #                     help='The solver to use (one of: {a_star,pea_star,epea_star}), defaults to ' + str(LLSOLVER))
    return parser


def main(args):
    txt_file_name = args.instance.split("/")[-1].split(".")[0]
    result_file = open(
        f"/mnt/Topics/Learning/MAPF/LocalHeuristics/MAPF-ICBS/code/test_results/ \
results_for_{txt_file_name}_{args.randomness}.txt",
        "a",
        buffering=1,
    )
    # result_file = open("results.csv", "w", buffering=1)

    # node_results_file = open("nodes-cleaned.csv", "w", buffering=1)

    nodes_gen_file_name = f"/mnt/Topics/Learning/MAPF/LocalHeuristics/MAPF-ICBS/code/test_results/ \
nodes_gen_for_{txt_file_name}_{args.randomness}.csv"
    nodes_exp_file_name = f"/mnt/Topics/Learning/MAPF/LocalHeuristics/MAPF-ICBS/code/test_results/ \
nodes_exp_for_{txt_file_name}_{args.randomness}.csv"
    nodes_gen_file = open(nodes_gen_file_name, "w", buffering=1)
    nodes_exp_file = open(nodes_exp_file_name, "w", buffering=1)

    # if args.batch:

    #     input_instance = sorted(glob.glob("instances/test*"))
    # else:
    input_instance = sorted(glob.glob(args.instance))
    if len(input_instance) == 0:
        raise BaseException(f"{INSTANCE} does not exist.")
    for file in input_instance:

        print("***Import an instance***")

        print(file)
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        if args.hlsolver == "CBS":
            print("***Run CBS***")
            collision_map = CollisionMap(
                my_map, starts, goals, agent_num, 1.3
            ).get_collision_map()

            cbs = CBSSolver(my_map, starts, goals, args.randomness, collision_map)
            # solution = cbs.find_solution(args.disjoint)

            # if solution is not None:
            #     # print(solution)
            #     paths, nodes_gen, nodes_exp = [solution[i] for i in range(3)]
            #     if paths is None:
            #         raise BaseException('No solutions')
            # else:
            #     raise BaseException('No solutions')
        elif args.hlsolver == "ICBS":
            print("***Run ICBS***")
            cbs = ICBS_Solver(my_map, starts, goals)
        else:
            raise RuntimeError("Unknown solver!")

        solution = cbs.find_solution(args.disjoint)

        if solution is not None:
            # print(solution)
            paths, nodes_gen, nodes_exp = [solution[i] for i in range(3)]
            if paths is None:
                raise BaseException("No solutions")
        else:
            raise BaseException("No solutions")

        cost = get_sum_of_cost(paths)
        result_file.write(
            "file{}, \ncost: {} nodes_gen: {} nodes_exp: {}\n".format(
                file, cost, nodes_gen, nodes_exp
            )
        )

        nodes_gen_file.write("{},{}\n".format(file, nodes_gen))
        nodes_exp_file.write("{},{}\n".format(file, nodes_exp))

        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0)
            animation.show()
            # animation.save('demo/fig.gif', 1)

    result_file.close()


if __name__ == "__main__":
    HLSOLVER = "CBS"
    LLSOLVER = "a_star"

    # map_name = "random-32-32-10"
    # map_name = "random-32-32-20"
    map_name = "random-64-64-10"
    # map_name = "Boston_0_256"
    cost_of_map = 0  # 1906  # 0: normal, <0 : cost of map

    agent_num = 60

    INSTANCE = f"/mnt/Topics/Learning/MAPF/LocalHeuristics/MAPF-ICBS/code/instances/test_a{agent_num}.txt"
    INSTANCE = f"/mnt/Topics/Learning/MAPF/LocalHeuristics/MAPF-ICBS/code/instances/test_{map_name}_a{agent_num}.txt"
    if cost_of_map > 0:
        INSTANCE = f"/mnt/Topics/Learning/MAPF/LocalHeuristics/MAPF-ICBS/code/instances/test_{map_name}_a{agent_num}_c{cost_of_map}.txt"

    parser = get_parser()
    args = parser.parse_args()
    args.batch = True  # simulation True: Simx
    args.randomness = "random"

    start_time = time.time()
    main(args)
    end_time = time.time()
    print(f"{args.randomness}에서 소요 시간:", end_time - start_time, "초")

    txt_file_name = args.instance.split("/")[-1].split(".")[0]
    result_file = open(
        f"/mnt/Topics/Learning/MAPF/LocalHeuristics/MAPF-ICBS/code/test_results/ \
results_for_{txt_file_name}_{args.randomness}.txt",
        "a",
        buffering=1,
    )
    result_file.write("소요 시간: {} 초\n\n".format(end_time - start_time))
