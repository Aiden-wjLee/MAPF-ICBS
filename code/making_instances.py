import random


def read_map(file_name):
    with open(file_name, "r") as file:
        lines = file.readlines()

    # Extract height and width from the input data
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])

    # Extract the map section
    map_start_idx = lines.index("map\n") + 1
    map_data = [line.strip() for line in lines[map_start_idx : map_start_idx + height]]

    return height, width, map_data


def generate_agent_tasks(map_data, num_agents):
    height = len(map_data)
    width = len(map_data[0])
    empty_cells = [
        (x, y) for x in range(height) for y in range(width) if map_data[x][y] == "."
    ]

    if len(empty_cells) < num_agents * 2:
        raise ValueError("Not enough empty cells to place all agents")

    tasks = []
    used_cells = set()

    for _ in range(num_agents):
        start = random.choice(empty_cells)
        empty_cells.remove(start)
        used_cells.add(start)

        goal = random.choice(empty_cells)
        empty_cells.remove(goal)
        used_cells.add(goal)

        tasks.append(f"{start[1]} {start[0]} {goal[1]} {goal[0]}")  # x, y format

    return tasks


def create_output_file(height, width, map_data, tasks, output_file_name):
    agent_count = len(tasks)

    # Create the output data
    output_lines = []
    output_lines.append(f"{height} {width}")
    output_lines.extend([" ".join(line) + " " for line in map_data])

    output_lines.append(f"{agent_count}")
    for i, task in enumerate(tasks):
        output_lines.append(f"{task}")

    # Join the output lines into a single string
    output_data = "\n".join(output_lines)

    # Write the output data to a file
    with open(output_file_name, "w") as file:
        file.write(output_data)


def main(input_file_name, output_file_name, num_agents):
    height, width, map_data = read_map(input_file_name)
    tasks = generate_agent_tasks(map_data, num_agents)
    create_output_file(height, width, map_data, tasks, output_file_name)


if __name__ == "__main__":
    input_file_name = (
        "/mnt/Topics/Learning/MAPF/LocalHeuristics/map/random-32-32-10.map"
    )
    output_file_name = (
        "/mnt/Topics/Learning/MAPF/LocalHeuristics/MAPF-ICBS/code/instances/test1.txt"
    )
    num_agents = 5
    main(input_file_name, output_file_name, num_agents)
