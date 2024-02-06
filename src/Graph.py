from enum import Enum
from Arc import Arc
from Node import Node


class Color(Enum):
    GRAY = 1
    WHITE = 2
    BLACK = 3


class Graph:
    def __init__(self, name):
        self.name = name
        self.nodes = {}
        self.arcs = []
        self.time = 0

    def add_node(self, node):
        if node not in self.nodes:
            self.nodes[node] = Node(node)

    def get_node(self, nodo):
        return self.nodes.get(nodo, None)

    def add_arc(self, origin, destiny, cost):
        if self.get_node(origin) is not None and self.get_node(destiny) is not None:
            arco = Arc(self.nodes[origin], self.nodes[destiny], float(cost))
            self.nodes[origin].add_adjacent(self.nodes[destiny])
            self.arcs.append(arco)

    def get_arc(self, origin, destiny):
        for arc in self.arcs:
            if arc.origin.name == origin and arc.destiny.name == destiny:
                return arc

        return None

    def __str__(self):
        result = ''

        for node in self.nodes.values():
            result += str(node) + "\n"

        return result

    def breadth_first_search(self, start_node):
        for node in self.nodes.values():
            node.color = Color.WHITE
            node.distance = float('Inf')
            node.parent = None

        start_node = self.get_node(start_node)
        start_node.distance = 0
        queue = [start_node]

        while len(queue) > 0:
            node = queue.pop(0)

            for adjacent in node.adjacent:
                if adjacent.color == Color.WHITE:
                    adjacent.color = Color.GRAY
                    adjacent.distance = node.distance + 1
                    adjacent.parent = node
                    queue.append(adjacent)

            node.color = Color.BLACK

    def depth_first_search(self):
        for node in self.nodes.values():
            node.color = Color.WHITE
            node.parent = None

        time = 0

        for node in self.nodes.values():
            if node.color == Color.WHITE:
                time = self.depth_first_search_visit(node, time)

    def depth_first_search_visit(self, node, time):
        node.color = Color.GRAY
        time += 1
        node.distance = time

        for adjacent in node.adjacent:
            if adjacent.color == Color.WHITE:
                adjacent.parent = node
                time = self.depth_first_search_visit(adjacent, time)

        node.color = Color.BLACK
        time += 1
        node.finalization = time

        return time

    def get_transposed(self):
        transposed_graph = Graph(self.name + '_transposed')

        for node in self.nodes.values():
            transposed_graph.add_node(node.name)

        for arc in self.arcs:
            new_origin = arc.destiny.name
            new_destiny = arc.origin.name
            cost = arc.cost
            transposed_graph.add_arc(new_origin, new_destiny, cost)

        return transposed_graph

    # return list of nodes

    def sort_nodes_by_finalization_desc(self):
        nodes_list = []
        desc_nodes = sorted(self.nodes.values(), key=lambda key_node: key_node.finalization, reverse=True)

        for node in desc_nodes:
            nodes_list.append(node.name)

        return nodes_list

    def strongly_connected_componentes(self):
        self.depth_first_search()
        transposed_graph = self.get_transposed()
        sorted_nodes = self.sort_nodes_by_finalization_desc()

        for node in transposed_graph.nodes.values():
            node.color = Color.WHITE
            node.parent = None

        time = 0
        forest = []

        for sorted_node in sorted_nodes:
            node = transposed_graph.get_node(sorted_node)

            if node.color == Color.WHITE:
                tree = []
                time = transposed_graph.strongly_connected_componentes_visit(node, time, tree)
                forest.append(forest)

        return forest

    def strongly_connected_componentes_visit(self, node, time, tree):
        tree.append(node)
        node.color = Color.GRAY
        time += 1
        node.distance = time

        for adjacent in node.adjacent:
            if adjacent.color == Color.WHITE:
                adjacent.parent = node
                time = self.depth_first_search_visit(adjacent, time)

        node.color = Color.BLACK
        time += 1
        node.finalization = time

        return time

    def mst_kruskal(self):
        new_graph = Graph(f'{self.name}_mst')
        union_find = {}

        for iteration, node in enumerate(self.nodes.values()):
            union_find[node.name] = iteration

        self.sort_nodes_by_cost_asc()

        '''
        A = new_graph
        u = origin
        v = destiny
        e = 
        '''

    # No sé si esté bien este método
    def sort_nodes_by_cost_asc(self):
        nodes_list = []
        desc_nodes = sorted(self.nodes.values(), key=lambda key_node: key_node.cost, reverse=False)

        for node in desc_nodes:
            nodes_list.append(node.name)

        return nodes_list
