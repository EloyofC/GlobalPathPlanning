#!/usr/bin/python

# need the dependency of matplotlib

import matplotlib.pyplot as plt
import re

def get_env_para(testlog):
    try:
        fobj = open(testlog, 'r')
        obstacles = []
        size = None
        for each_line in fobj.readlines():
            match_size = re.match('PrintEnvironment : .*length (\\d+).*width (\\d+)', each_line)
            if match_size:
                size = tuple([int(i) for i in match_size.groups()])
            match_obstacle = re.match('PrintEnvironmentMember :.*x (\\d+).*y (\\d+).*with obstacle$', each_line)
            if match_obstacle:
                nums = match_obstacle.groups()
                obstacles.append(nums)
            fobj.close()
    except IOError, error_msg:
        print 'File not found :', error_msg
    return (size, obstacles)

def plot_env(size, obstacles):
    env_scatter = plt.subplot(1,1,1)
    length, width = size
    env_scatter.set_xlim([0, length])
    env_scatter.set_ylim([0, width])
    x = [single_obstacle[0] for single_obstacle in obstacles]
    y = [single_obstacle[1] for single_obstacle in obstacles]
    env_scatter.scatter(x, y, c = 'red')
    plt.show()

if __name__ == '__main__':
    size, obstacles = get_env_para('test.log')
    plot_env(size, obstacles)
