import heapq
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

    def find(self, parent, i):
        while parent[i] != i:
            i = parent[i]
        return i

    def union(self, parent, root_range, x, y):
        x_root = self.find(parent, x)
        y_root = self.find(parent, y)

        if root_range[x_root] < root_range[y_root]:
            parent[x_root] = y_root
        elif root_range[x_root] > root_range[y_root]:
            parent[y_root] = x_root
        else:
            parent[y_root] = x_root
            root_range[x_root] += 1

    def kruskal(self):
        minimum_spanning_tree = []
        self.arcs.sort(key=lambda sort_arc: sort_arc.cost)
        parent = {}
        rank = {}

        for node in self.nodes:
            parent[node] = node
            rank[node] = 0

        for arc in self.arcs:
            root_origin = self.find(parent, arc.origin.name)
            root_destination = self.find(parent, arc.destiny.name)

            if root_origin != root_destination:
                minimum_spanning_tree.append(arc)
                self.union(parent, rank, root_origin, root_destination)

        kruskal_graph = Graph(f'{self.name}_kruskal')

        for arc in minimum_spanning_tree:
            origin = arc.origin.name
            destiny = arc.destiny.name
            cost = arc.cost

            kruskal_graph.add_node(origin)
            kruskal_graph.add_node(destiny)
            kruskal_graph.add_arc(origin, destiny, cost)
            kruskal_graph.add_arc(destiny, origin, cost)

        return kruskal_graph

    def mst_prim(self, start_node):
        prim_graph = Graph(f'{self.name}_prim')
        nodes_list = []

        for node in self.nodes.values():
            node.distance = float('Inf')
            node.parent = None
            nodes_list.append(node)

        start_node = self.get_node(start_node)
        start_node.distance = 0

        while len(nodes_list) > 0:
            node = min(nodes_list, key=lambda node: node.distance)
            nodes_list.remove(node)
            prim_graph.add_node(node.name)
            prim_graph.get_node(node.name).distance = node.distance
            prim_graph.get_node(node.name).parent = node.parent

            if node.parent is not None:
                prim_graph.add_arc(node.parent.name, node.name, node.distance - node.parent.distance)
                prim_graph.add_arc(node.name, node.parent.name, node.distance - node.parent.distance)

            for adjacent_node in node.adjacent:
                cost = self.get_arc(node, adjacent_node)
                if cost == float('Inf'):
                    continue
                if adjacent_node in nodes_list and cost + node.distance < adjacent_node.distance:
                    adjacent_node.distance = cost + node.distance
                    adjacent_node.parent = node

        return prim_graph

    def old_mst_prim(self):

        # Creamos una lista para almacenar los nodos visitados y una
        # lista para almacenar los nodos que faltan por visitar
        visited = set()
        remaining = set(self.nodes)

        # Se agarra un nodo aleatorio como el profe le hizo, en su caso el empezo en s pero pues es la misma xd
        start_node = next(iter(remaining))
        visited.add(start_node)
        remaining.remove(start_node)

        # Creamos un montículo para almacenar las aristas disponibles
        edges = [(weight, start_node, neighbor) for neighbor, weight in self[start_node]]
        heapq.heapify(edges)

        # Inicializamos el árbol de expansión mínima
        mst = []

        # Mientras queden nodos por visitar
        while remaining:
            # Tomamos la arista más corta disponible
            # Como ven plebes, se importo esa madre que se llama heapq, sirve para tomar la arista mas corta sin
            # necesidad de hacer un puto metodo
            weight, source, destination = heapq.heappop(edges)

            # Si el nodo de destino no ha sido visitado aún, lo agregamos al árbol de expansión mínima
            if destination in remaining:
                visited.add(destination)
                remaining.remove(destination)
                mst.append((source, destination, weight))

                # Agregamos las aristas del nuevo nodo a las aristas disponibles
                for neighbor, weight in self[destination]:
                    if neighbor in remaining:
                        heapq.heappush(edges, (weight, destination, neighbor))

        return mst

        # Pueden moverle con gusto plebes, yo consegui este algoritmo desde chatgpt pero le cambie unas cosas,
        # la verdad siento el error que da es porque estoy llamando mal los atributos del grafo, espero lo sepan adaptar
        # al nuestro, de igual forma les dejare el codigo que me dio chatgpt para que vean que es lo que me dio. Lo
        # pondre en un txt para que lo puedan leer
