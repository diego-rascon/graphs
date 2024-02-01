from Graph import Graph

file = open('../graphs/kruskal1.txt')
lines = file.readlines()

graph = Graph('graph')

for row, line in enumerate(lines):
    line = line.rstrip()
    line = line.lower()
    elements = line.split(' ')
    if len(elements) == 2 or len(elements) > 3:
        print(f'Sintaxis error on line {row} ({elements}).')
        exit(1)
    elif len(elements) == 3:
        origin_node = elements[0]
        destination_node = elements[1]
        arc_cost = elements[2]

        graph.add_node(origin_node)
        graph.add_node(destination_node)

        graph.add_arc(origin_node, destination_node, arc_cost)
        # hehehe

graph.depth_first_search()

print('Grafo Original\n')
print(graph)

print('Grafo Transpuesto\n')
print(graph.get_transposed())

print('Grafo Ordenado por Tiempo de Finalización\n')
print(graph.sort_nodes_by_finalization_desc())

mst = graph.mst_kruskal()

print(mst)


