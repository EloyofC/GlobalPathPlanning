#!/usr/bin/python

import re
import sys

def get_obs_info(file_name):
    with open(file_name, 'r') as fobj:
        start = None
        end = None
        obstacles = []
        obstacle_index = 0
        for each_line in fobj.readlines():
            match_start = re.match('[sS]tart.*?(\\d+).*?(\\d+)', each_line)
            if match_start:
                start= [int(i) for i in match_start.groups()]
            match_end = re.match('[eE]nd.*?(\\d+).*?(\\d+)', each_line)
            if match_end:
                end = [int(i) for i in match_end.groups()]
            match_obstacle_index = re.match('Obs.*?(\\d+)', each_line)
            if match_obstacle_index:
                new_index = int(match_obstacle_index.groups()[0])
                assert(obstacle_index + 1 == new_index)
                obstacle_index = new_index
                obstacles.append([])
            match_obstacle = re.match('(\\d+).*?(\\d+)', each_line)
            if match_obstacle:
                obstacle_cor = match_obstacle.groups()
                obstacles[obstacle_index - 1].append(obstacle_cor)
        return (start, end, obstacles)

def output_obs_info(start, end, obstacles):
    print "Start : x {0[0]} y {0[1]}".format(start)
    print "End : x {0[0]} y {0[1]}".format(end)
    for i, single_obstacle in enumerate(obstacles):
        obsx = [int(cor[0]) for cor in single_obstacle]
        obsy = [int(cor[1]) for cor in single_obstacle]
        print "X of {0}nd obstacle : {1}".format(i, obsx)
        print "Y of {0}nd obstacle : {1}".format(i, obsy)

if __name__ == '__main__':
    file_name = 'Obstacles.txt'
    start, end, obstacles = get_obs_info(file_name)
    output_obs_info(start, end, obstacles)
