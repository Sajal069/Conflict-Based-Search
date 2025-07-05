import AStar

class Node:
    def __init__(self, constraint, cost):
        self.constraint = constraint
        self.cost = cost
        self.children = []

    def add_child(self, node):
        self.children.append(node)

class CBS:
    def __init__(self, agents, graph):
        self.agents = agents
        self.graph = graph

    def find_path(self, agent, constraints):
        # This function should implement a pathfinding algorithm (like A*) that finds a path for the agent in the graph
        # while respecting the constraints.
        return AStar.A_star(agent.start,agent.goal,AStar.heuristic,self.graph)

    def search(self):
        root = Node({}, 0)
        open_list = [root]

        while open_list:
            node = open_list.pop(0)
            paths = {}

            for agent in self.agents:
                if agent in node.constraint:
                    path = self.find_path(agent, node.constraint[agent])
                else:
                    path = self.find_path(agent, [])

                if path is None:
                    break

                paths[agent] = path

            if len(paths) == len(self.agents):
                return paths

            for agent in self.agents:
                if agent in node.constraint:
                    continue

                for i in range(len(paths[agent]) - 1):
                    constraint = {agent: [(paths[agent][i], paths[agent][i + 1])]}
                    child = Node(constraint, node.cost + 1)
                    node.add_child(child)
                    open_list.append(child)

        return None
