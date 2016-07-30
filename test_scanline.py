#!/usr/bin/env python


import unittest
from scanline import *


class TestEdge(unittest.TestCase):
    def test_edgemethod(self):
        edge1 = Edge(-1, -3, 2, 6)
        self.assertEqual(edge1.get_ymax(), 6)
        self.assertEqual(edge1.get_ymin(), 0)
        self.assertEqual(edge1.get_x(), 0.0)
        edge1.update_x()
        self.assertAlmostEqual(edge1.get_x(), 0.3333333)

class TestEdgeTable(unittest.TestCase):
    def setUp(self):
        self.my_edge_table = EdgeTable(5)

    def tearDown(self):
        self.my_edge_table = None

    def test_initial(self):
        self.assertEqual(self.my_edge_table.get_table_height(), 5)

    def test_insert(self):
        self.assertEqual(len(self.my_edge_table.get_yedges_table(0)), 0)
        edge1 = Edge(-1, -3, 2, 6)
        self.my_edge_table.insert_edge(edge1)
        self.assertEqual(len(self.my_edge_table.get_yedges_table(0)), 1)
        self.assertEqual(len(self.my_edge_table.get_yedges_table(1)), 0)
        edge2 = Edge(1, 1, 3, 7)
        self.my_edge_table.insert_edge(edge2)
        self.assertEqual(len(self.my_edge_table.get_yedges_table(1)), 1)
        self.assertEqual(len(self.my_edge_table.get_yedges_table(2)), 0)
        edge3 = Edge(2, 2, 4, 4)
        self.my_edge_table.insert_edge(edge3)
        self.assertEqual(len(self.my_edge_table.get_yedges_table(2)), 1)
        edge4 = Edge(1, 2, 3, 4)
        self.my_edge_table.insert_edge(edge4)
        self.assertEqual(len(self.my_edge_table.get_yedges_table(2)), 2)


class TestActiveEdgeTable(unittest.TestCase):
    def setUp(self):
        self.edge1 = Edge(-1, -3, 2, 3)
        self.edge2 = Edge(1, 1, 3, 3)
        self.edge3 = Edge(0, 0, 4, 4)
        self.edge4 = Edge(1, 2, 3, 4)
        self.edges = [self.edge1, self.edge2, self.edge3, self.edge4]
        self.edge_table = EdgeTable(5)
        for i in self.edges:
            self.edge_table.insert_edge(i)
        self.active_edge_table = ActiveEdgeTable(self.edge_table)

    def tearDown(self):
        self.edge1 = None
        self.edge2 = None
        self.edge3 = None
        self.edge4 = None
        self.edge_table = None
        self.active_edge_table = None

    def test_update(self):
        self.assertItemsEqual(self.active_edge_table.get_xofedges(), [])
        self.active_edge_table.get_new_edge(0)
        self.assertItemsEqual(self.active_edge_table.get_xofedges(), [0, 0.5])
        self.active_edge_table.update_allx()
        self.active_edge_table.get_new_edge(1)
        self.assertItemsEqual(self.active_edge_table.get_xofedges(), [1, 1, 1])


def suite():
    test_class_to_run = [
        'TestEdge', 'TestEdgeTable', 'TestActiveEdgeTable'
    ]
    loader = unittest.TestLoader()
    suite_list = []
    for test_class in test_class_to_run:
        new_suite = loader.loadTestsFromTestCase(test_class)
        suite_list.append(new_suite)
    return suite_list


if __name__ == '__main__':
    unittest.main()
