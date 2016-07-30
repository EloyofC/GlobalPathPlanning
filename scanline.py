#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt


class Edge(object):
    """ This object is to create the attribute for the edge table
    Caution: The y1 can not be equal with y2
    >>> edges = Edge(2, 1, 8, 1)
    Traceback (most recent call last):
    ...
    AssertionError
    >>> Edge.cal_rev_ratio(16, 6, 8, 1)
    1.6
    >>> Edge.cal_x_ypositivemin(1, 2, 3, 4)
    1.0
    >>> Edge.cal_x_ypositivemin(-1, -3, 2, 3)
    0.5
    >>> Edge.cal_ymax(3, 1)
    3
    >>> Edge.cal_ymin_positive(1, 3)
    1
    >>> Edge.cal_ymin_positive(-2, 9)
    0
    >>> Edge.cal_ymin_positive(2, -9)
    0
    """
    def __init__(self, x1, y1, x2, y2):
        assert(y1 != y2)
        self._ymax = self.cal_ymax(y1, y2)
        self._ymin = self.cal_ymin_positive(y1, y2)
        self._xcurrent = self.cal_x_ypositivemin(x1, y1, x2, y2)           # it is a float number ,initial the x with the xmin and update the value in AET
        self._ratio = self.cal_rev_ratio(x1, y1, x2, y2)

    def update_x(self):
        self._xcurrent += self._ratio

    def get_ymax(self):
        return self._ymax

    def get_x(self):
        return self._xcurrent

    def get_ymin(self):
        return self._ymin

    @staticmethod
    def cal_rev_ratio(x1, y1, x2, y2):
        if y1 < y2:
            return 1.0 * (x2-x1) / (y2-y1)
        else:
            return 1.0 * (x1-x2) / (y1-y2)

    @staticmethod
    def cal_x_ypositivemin(x1, y1, x2, y2):
        """
        This function return the x cor with the ymin
        and if y < 0 it will be left to zero
        """
        if y1 < 0 or y2 < 0:
            xshift = -1.0 * (x2-x1) / (y2-y1) * y2
            xcal = x2 + xshift
            return xcal
        elif y1 < y2:
            return x1 * 1.0
        else:
            return x2 * 1.0

    @staticmethod
    def cal_ymax(y1, y2):
        if y1 < y2:
            return y2
        else:
            return y1

    @staticmethod
    def cal_ymin_positive(y1, y2):
        if y1 < y2:
            if y1 < 0:
                return 0
            else:
                return y1
        else:
            if y2 < 0:
                return 0
            else:
                return y2


class EdgeTable(object):
    """
    >>> edge_table = EdgeTable(5)
    >>> edge1 = Edge(1, 1, 3, 7)
    >>> edge_table.insert_edge(edge1)
    >>> (edge_table.get_yedges_table(1))[0] == edge1
    True
    >>> len(edge_table.get_yedges_table(0))
    0
    >>> edge2 = Edge(-1, -3, 2, 6)
    >>> edge_table.insert_edge(edge2)
    >>> len(edge_table.get_yedges_table(0))
    1
    """
    def __init__(self, height):
        self._height = height
        self._table = [[] for i in range(height)]

    def insert_edge(self, edge):
        self._table[edge.get_ymin()].append(edge)

    def get_table_height(self):
        return self._height

    def get_yedges_table(self, index):
        return self._table[index]


class ActiveEdgeTable(object):
    def __init__(self, edge_table):
        """
        The initial state of the active edge table should be empty until
        get_new_edge method is used with the argument of zero
        """
        self._edge_table = edge_table
        self._active_table = []

    def get_new_edge(self, ycurrent):
        for i in self._edge_table.get_yedges_table(ycurrent):
            self._active_table.append(i)

    def remove_died_edge(self, ycurrent):
        self._active_table = [edge for edge in self._active_table if edge.get_ymax() != ycurrent]

    def get_xofedges(self):
        return [edge.get_x() for edge in self._active_table]

    def update_allx(self):
        for edge in self._active_table:
            edge.update_x()


def isvalid_edge(y1, y2, height):
    """ check whether the edge is a horizontal line or the egde is all
    below the zero line or above the top line
    >>> isvalid_edge(1, 2, 5)
    True
    >>> isvalid_edge(1, 1, 5)
    False
    >>> isvalid_edge(-1, 0, 5)
    True
    >>> isvalid_edge(-2, -4, 5)
    False
    >>> isvalid_edge(6, 3, 5)
    True
    >>> isvalid_edge(8, 19, 5)
    False
    """
    if ((y1 == y2) or
        (y1 < 0 and y2 < 0) or
        (y1 >= height and y2 >= height)):
        return False
    else:
        return True

class EnvMap(object):
    """
    >>> envmap = EnvMap(9, 5)
    >>> envmap.set_envmap_obstacle(2,3)
    >>> envmap.get_envmap_byheight(3)
    [0, 0, 1, 0, 0, 0, 0, 0, 0]
    >>> envmap.IsVertexInEnv(1,3)
    True
    >>> envmap.IsVertexInEnv(6, 8)
    False
    >>> envmap.IsVertexInEnv(-1, 3)
    False
    >>> envmap.fill_scanline((5, 7, 1, 3), 2)
    >>> envmap.get_envmap_byheight(2)
    [0, 1, 1, 0, 0, 1, 1, 0, 0]
    """

    def __init__(self, length, height):
        self._length = length
        self._height = height
        self._envmap = [[0 for i in range(length)] for j in range(height)]

    def set_envmap_obstacle(self, length, height):
        assert(length < self._length)
        assert(height < self._height)
        print "set envmap obstacle length %d height %d" % (length, height)
        self._envmap[height][length] = 1

    def get_envmap_byheight(self, height):
        assert(0 <= height < self._height)
        return self._envmap[height]

    def get_height(self):
        return self._height

    def get_length(self):
        return self._length

    def IsVertexInEnv(self, i, j):
        return 0 <= i < self._length and 0 <= j < self._height

    def fill_scanline(self, xlist, ycurrent):
        """
        the fill rule is include the left point and ignore the right
        the precondition is already exclude the horizontal line
        """
        if xlist is None or len(xlist) < 2:
            return
        xsorted = list(xlist)
        xsorted.sort()
        xsorted = [int(round(i)) for i in xsorted]
        xpairs = []
        for i in range(len(xsorted) - 1):
            if i % 2 == 0:
                xpairs.append(xsorted[i : i+2])
        for xpair in xpairs:
            for i in range(xpair[0], xpair[1]):
                if self.IsVertexInEnv(i, ycurrent):
                    self.set_envmap_obstacle(i, ycurrent)


def plot_env(envmap, row=1, col=1, index=1):
    env_scatter = plt.subplot(row, col, index)
    env_scatter.set_title("Obstacles In Environment")
    length = envmap.get_length()
    width = envmap.get_height()
    env_scatter.set_xlabel("Length %d" % (length))
    env_scatter.set_ylabel("Width %d " % (width))
    env_scatter.set_xlim([0, length])
    env_scatter.set_ylim([0, width])
    obsx = []
    obsy = []
    for i in range(width):
        env_length_map = envmap.get_envmap_byheight(i)
        for j, envmem in enumerate(env_length_map):
            if envmem == 1:
                obsx.append(j)      # x cor to length
                obsy.append(i)
    env_scatter.scatter(obsx, obsy, c = 'red')
    plt.show()


def get_envedges(height, edges):
    """
    >>> get_envedges(100, ([10, 10], [20, 10], [20, 20], [10, 20]))
    [[20, 10, 20, 20], [10, 20, 10, 10]]
    """
    if edges is None or len(edges) < 2:
        return []
    envedge = []
    prev_vetex = None
    modified_edges = list(edges)
    modified_edges.append(modified_edges[0])
    for i in modified_edges:
        if prev_vetex is not None and isvalid_edge(prev_vetex[1], i[1], height):
            envedge.append(prev_vetex + i)
        prev_vetex = i
    return envedge


def draw_envmap(length, height, obstacles, row_plot=1, col_plot=1, index_plot=1):
    envmap = EnvMap(length, height)
    set_obstacles_env(length, height, obstacles, envmap)
    plot_env(envmap, row_plot, col_plot, index_plot)


def set_obstacles_env(length, height, obstacles, envmap):
    for obstacle in obstacles:
        set_singleobstacle_env(length, height, obstacle, envmap)


def set_singleobstacle_env(length, height, single_obstacle, envmap):
    env_edges = get_envedges(height, single_obstacle)
    env_edge_table = EdgeTable(height)
    for i in env_edges:
        edge = Edge(*i)
        env_edge_table.insert_edge(edge)
    active_edge_table = ActiveEdgeTable(env_edge_table)
    for i in range(height):
        active_edge_table.get_new_edge(i)
        active_edge_table.remove_died_edge(i)
        xlist = active_edge_table.get_xofedges()
        envmap.fill_scanline(xlist, i)
        active_edge_table.update_allx()


def main():
    import doctest
    doctest.testmod()
    length1 = 50
    height1 = 50
    edges1 = [
        [40, 20], [40, -20], [60, -20], [60, 20]
    ]
    edges2 = [
        [-5, 30], [-3, 8], [8, -3], [15, 10]
    ]
    edges3 = [
        [30, 30], [30, 90], [90, 90], [90, 30]
    ]
    edges = [
        edges1, edges2, edges3
    ]
    obstacles2 = []
    for i in edges:
        obstacles2.append(i)
    draw_envmap(length1, height1, obstacles2)


if __name__ == '__main__':
    main()
