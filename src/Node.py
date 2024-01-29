class Node:
    def __init__(self, name):
        self.name = name
        self.adjacent = []
        self.distance = float('Inf')
        self.finalization = 0
        self.parent = None
        self.color = None
        self.id = 0
        self.time = 0

    def add_adjacent(self, new_node):
        if new_node not in self.adjacent:
            self.adjacent.append(new_node)

    def __str__(self):
        adjacent = []

        for node in self.adjacent:
            adjacent.append(node.name)

        if len(adjacent) > 0:
            adjacent = ', '.join(adjacent)
        else:
            adjacent = 'no adjacent'

        if self.parent is None:
            parent = 'no parent'
        else:
            parent = self.parent.name

        return (f'node: {self.name},'
                f' d({self.distance}),'
                f' f({self.finalization}),'
                f' parent: {parent},'
                f' color: {self.color},'
                f' id: {self.id}:'
                f' adjacent: {adjacent}')
