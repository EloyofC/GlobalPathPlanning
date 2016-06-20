#!/usr/bin/python

import matplotlib.pyplot as plt
import re

def getEnvPara(testlog):
    obstacles = []
    size = None
    try :
        f = open(testlog, 'r')
        for eachLine in f.readlines():
            matchSize = re.match('PrintEnvironment : .*length (\d+).*width (\d+)', eachLine)
            if matchSize:
                size = tuple([int(i) for i in matchSize.groups()])
            matchObstacle = re.match('PrintEnvironmentMember :.*x (\d+).*y (\d+).*with obstacle$', eachLine)
            if matchObstacle:
                nums = matchObstacle.groups()
                obstacles.append(nums)
            f.close()
    except IOError, e:
        print 'File not found :', e
    return (size, obstacles)

def plotEnv(size, obstacles):
    env_scatter = plt.subplot(1,1,1)
    length, width = size
    env_scatter.set_xlim([0, length])
    env_scatter.set_ylim([0, width])
    x = [single_obstacle[0] for single_obstacle in obstacles]
    y = [single_obstacle[1] for single_obstacle in obstacles]
    env_scatter.scatter(x, y, c = 'red')
    plt.show()

if __name__ == '__main__':
    size, obstacles = getEnvPara('test.log')
    plotEnv(size, obstacles)
