class Arc:
    def __init__(self, origin, destiny, cost):
        self.origin = origin
        self.destiny = destiny
        self.cost = cost

    def __str__(self):
        return self.origin.name + ' - ' + self.destiny.name + ' : ' + str(self.cost)
