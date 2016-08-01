#!/usr/bin/python

# need the dependency of matplotlib

import re
import sys
import matplotlib.pyplot as plt


def get_env_para(testlog):
    try:
        fobj = open(testlog, 'r')
    except IOError, error_msg:
        print 'File not found :', error_msg
    else:
        obstacles = []
        env_size = None
        for each_line in fobj.readlines():
            match_size = re.match('InitialEnvWithCell : .*x (\\d+).*y (\\d+)', each_line)
            if match_size:
                env_size = tuple([int(i) for i in match_size.groups()])
            match_obstacle = re.match('PrintEnvironmentMember :.*x (\\d+).*y (\\d+).*with obstacle$', each_line)

            if match_obstacle:
                nums = match_obstacle.groups()
                obstacles.append([int(i) for i in nums])
        fobj.close()
    return (env_size, obstacles)


def get_env_path(testlog):
    try:
        fobj = open(testlog, 'r')
    except IOError, error_msg:
        print 'File not found :', error_msg
    else:
        env_points = []
        for each_line in fobj.readlines():
            match_points = re.match('The env pass point : .*x (\\d+).*y (\\d+)', each_line)
            if match_points:
                point = match_points.groups()
                env_points.append([int(i) for i in point])
        fobj.close()
    return env_points


def plot_env(env_size, obstacles, path_points):
    env_scatter = plt.subplot(1, 1, 1)
    env_scatter.set_title("Obstacles In Environment")

    length, width = env_size
    env_scatter.set_xlabel("Length %d" % (length))
    env_scatter.set_ylabel("Height %d " % (width))
    env_scatter.set_xlim([0, length])
    env_scatter.set_ylim([0, width])

    obsx = [single_obstacle[0] for single_obstacle in obstacles]
    obsy = [single_obstacle[1] for single_obstacle in obstacles]
    env_scatter.scatter(obsx, obsy, c='red')

    pointx = [single_point[0] for single_point in path_points]
    pointy = [single_point[1] for single_point in path_points]
    env_scatter.scatter(pointx, pointy, c='blue')
    plt.plot( pointx, pointy, '-og')
    plt.show()


def main():
    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    else:
        file_name = 'test.log'
    parameters = get_env_para(file_name)
    env_size, obstacles = parameters
    if env_size is not None:
        path_points = get_env_path(file_name)
        plot_env(env_size, obstacles, path_points)
    else:
        print "Debug Info is not right"


if __name__ == '__main__':
    main()
