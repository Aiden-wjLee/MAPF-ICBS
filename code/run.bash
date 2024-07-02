#!/bin/bash

# sigma 배열을 설정
# sigma=(1.0 2.0 3.0 4.0 5.0)
sigma=(5.0)
# sigma=1.0

# 반복 횟수를 설정
repeat_count=5
map_name="random-32-32-10"
# map_name="random-32-32-20"
# map_name="random-64-64-10"
map_name="Boston_0_256"
agent_num=40

cost_of_map=7425
# cost_of_map=0

h_prime_TF=true
approximated=true #true: 빠른 실행, false: 정확한 실행 (collision check)
voronoi_TF=true
random_ratio=0.2
zero_ratio=0.5
# cost_of_map=7425 #7425 for Boston_0_256
# c

# for 루프를 사용하여 Python 스크립트를 여러 번 실행
for s in "${sigma[@]}"
do
    for i in $(seq 1 $repeat_count)
    do
        echo "Running iteration $i with sigma $s"
        python run_forlearning.py --sigma $s --map_name $map_name --agent_num $agent_num --h_prime_TF $h_prime_TF --cost_of_map $cost_of_map --approximated "$approximated" --voronoi_TF "$voronoi_TF" --random_ratio $random_ratio --zero_ratio $zero_ratio
    done
done
