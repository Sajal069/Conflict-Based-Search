# A basic implementation of the Conflict-Based Search (CBS) for the Multi-Agent
# Path Finding (MAPF) Problem. The problem is to find a path for multiple
# robots such that they could go through the shared pathways without having
# a conflict with each other.
#


from __future__ import annotations
from itertools import zip_longest
from itertools import combinations
from math import fabs
from copy import deepcopy



class Constraints:
    """
    Used to add the constraints to the top-level CBS tree.
    """
    def __init__(
            self) -> None:
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(
            self,
            other: Constraints) -> None:
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(
            self) -> str:
        return "VC: " + str([str(vc) for vc in self.vertex_constraints]) + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])


class HighLevelNode:
    """
    Root node for CBS
    """
    def __init__(self) -> None:
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(
            self,
            other: HighLevelNode) -> bool:
        if not isinstance(other, type(self)):
            return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(
            self) -> hash:
        """Added to make this class hashable"""
        return hash(self.cost)

    def __lt__(       # Less than
            self,
            other: HighLevelNode):
        return self.cost < other.cost


class Conflict:
    """
    The Conflict class defines what kind of conflicts can occur for two agents.
    There are many kinds of conflicts that can occur, including vertex, edge,
    cycle and swapping conflicts.

    We only consider two kinds of conflicts in the current implementation, a
    vertex conflict and an edge conflict.

    A vertex conflict is a conflict where agents plan to occupy the same vertex
    at the same time stamp.

    An edge conflict occurs when agents plan to traverse the same edge at the
    same time stamp.
    """
    VERTEX = 1
    EDGE = 2

    def __init__(
            self) -> None:
        self.time = -1
        self.conflict_type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.position_1 = Position()
        self.position_2 = Position()

    def __str__(self):
        return '[' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
               ', (' + str(self.position_1) + ', ' + str(self.position_2) + ')' + ']'


class AStar:
    """
    Define the A-star class for low-level shortest path computation
    """

    def __init__(
            self,
            env) -> None:
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors

    @staticmethod
    def reconstruct_path(
            came_from,
            current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name):
        """
        low level search
        """
        initial_state = self.agent_dict[agent_name]["start"]
        step_cost = 1

        closed_set = set()
        # Open_set is the set of nodes that need to be expanded / re-expanded
        open_set = {initial_state}

        # came_from is the node/state that immediately precedes the current
        # node/state on the shortest path from the start to the currently known
        # node/state
        came_from = {}

        # g_score is the cost of the cheapest path from start node/state to
        # current node/state
        g_score = {initial_state: 0}

        # For node n, fScore[n] := gScore[n] + h(n), where h(n) represents
        # the admissible heuristic. It is our best guess of the cost of the
        # path if it goes through n
        f_score = {initial_state: self.admissible_heuristic(initial_state,
                                                            agent_name)}

        while open_set:
            # Initialize the temp dict
            temp_dict = {
                open_item: f_score.setdefault(open_item, float("inf")) for
                open_item in open_set}
            current = min(temp_dict, key=temp_dict.get)

            if self.is_at_goal(current, agent_name):
                return self.reconstruct_path(came_from, current)

            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score.setdefault(current, float(
                    "inf")) + step_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor,
                                                             float("inf")):
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[
                                        neighbor] + self.admissible_heuristic(
                    neighbor, agent_name)
        return False
    
class CBS:
    """
    Define the CBS class
    """
    def __init__(
            self,
            environment) -> None:
        self.env = environment
        self.open_set = set()
        self.closed_set = set()

    def search(self):
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = self.env.get_first_conflict(P.solution)
            if not conflict_dict:
                # print("A-star solution found")
                return self.generate_plan(P.solution)

            constraint_dict = self.env.create_constraints_from_conflict(
                conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(
                    constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                # else:
                #     print(new_node.solution)
                #     return self.generate_plan(new_node.solution)
                new_node.cost = self.env.compute_solution_cost(
                    new_node.solution)
                # print(new_node.cost)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    @staticmethod
    def generate_plan(solution):
        plan = {}
        for agent, path in solution.items():
            # path_dict_list = [{'t': state.time, 'x': state.position.x,
            #                    'y': state.position.y} for state in path]
            path_list = [(state.time, (state.position.x, state.position.y))
                         for state in path]
            plan[agent] = path_list
        return plan

class Position:
    """
    The Position class defines a specific position (x,y) in the environment.
    """
    def __init__(
            self,
            x: int = -1,
            y: int = -1) -> None:
        self.x = x
        self.y = y

    def __eq__(
            self,
            other: Position) -> bool:
        return self.x == other.x and self.y == other.y

    def __str__(
            self) -> str:
        return str((self.x, self.y))

class State:
    """
    The State class defines the state of an agent at a specific time. This
    includes both the position and time for that agent
    """

    def __init__(
            self,
            time: int,
            position: Position) -> None:
        self.position = position
        self.time = time

    def __eq__(
            self,
            other: State) -> bool:
        return self.time == other.time and self.position == other.position

    def is_equal_except_time(
            self,
            other: State) -> bool:
        return self.position == other.position

    def __hash__(
            self) -> hash:
        """Added to make the State class hashable"""
        return hash(str(self.time)+str(self.position.x) + str(self.position.y))

    def __str__(
            self) -> str:
        return str((self.time, self.position.x, self.position.y))


class VertexConstraint:
    """
    A vertex constraint is constraint which defines a vertex to be added in the
    top-level CBS tree.
    """
    def __init__(
            self,
            time: int,
            position: Position) -> None:
        self.time = time
        self.position = position

    def __eq__(
            self,
            other: VertexConstraint) -> bool:
        return self.time == other.time and self.position == other.position

    def __hash__(
            self) -> hash:
        """Added to make this class hashable"""
        return hash(str(self.time)+str(self.position.x) + str(self.position.y))

    def __str__(
            self) -> str:
        return '[' + str(self.time) + ', ' + str(self.position) + ']'

class EdgeConstraint:
    """
    An edge constraint is constraint which defines an edge to be added in the
    top-level CBS tree.
    """
    def __init__(
            self,
            time: int,
            position_1: Position,
            position_2: Position):
        self.time = time
        self.position_1 = position_1
        self.position_2 = position_2

    def __eq__(
            self,
            other: EdgeConstraint) -> bool:
        return self.time == other.time and self.position_1 == other.position_1 \
            and self.position_2 == other.position_2

    def __hash__(
            self) -> hash:
        """Added to make this class hashable"""
        return hash(str(self.time)+str(self.position_1) + str(self.position_2))

    def __str__(
            self) -> str:
        return '[' + str(self.time) + ', (' + str(self.position_1) + ', ' + \
            str(self.position_2) + ')]'


class Environment:
    """
    Orchestration of CBS happens here.
    """

    def __init__(self, grid: list, agents: list) -> None:
        self.grid = grid
        self.agents = agents
        self.agent_dict = {}
        self.make_agent_dict()
        self.constraints = Constraints()
        self.constraint_dict = {}
        self.a_star = AStar(self)

    def get_neighbors(self, state: State) -> list:
        """
        Extract neighbours valid for the next move. The valid moves include
        wait, up, down, left and right

        :param state: The current state of the agent
        :returns: A list of neighbours valid for the next move
        """

        neighbors = []
        directions = [[-1, 0], [1, 0], [0, -1], [0, 1],[1,1],[1,-1],[-1,1],[-1,-1]]  
        for dx, dy in directions:
            new_pos = Position(state.position.x + dx, state.position.y + dy)
            new_state = State(state.time + 1, new_pos)
            if self.is_state_valid(new_state) and self.is_transition_valid(state, new_state):
                neighbors.append(new_state)
        return neighbors

    def get_first_conflict(self, solution) -> Conflict:
        """
        Extract the first conflict that exists in the plans

        :param solution: Complete solution containing plans for each agent
        :returns: An object of type Conflict, containing the first conflict found
        """
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()

        for t in range(max_t):
            # Identify a vertex conflict
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.conflict_type = Conflict.VERTEX
                    result.position_1 = state_1.position
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            # Identify an edge conflict
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t + 1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t + 1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.conflict_type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.position_1 = state_1a.position
                    result.position_2 = state_1b.position
                    return result
        return None

    def create_constraints_from_conflict(self, conflict: Conflict) -> dict:
        """
        Creates the constraints that need to be added to the CBS tree

        :param conflict: The conflict that has been identified
        :returns: A dict containing constraints
        """
        constraint_dict = {}
        if conflict.conflict_type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.position_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.conflict_type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.position_1, conflict.position_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.position_2, conflict.position_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def is_state_valid(self, state: State) -> bool:
        """
        Checks if a state is valid or not.
        :param state: State to be checked
        :returns: bool, whether the state is valid or not
        """
        return state.position.x >= 0 and state.position.x < len(self.grid) \
               and state.position.y >= 0 and state.position.y < len(self.grid[0]) \
               and VertexConstraint(state.time, state.position) not in self.constraints.vertex_constraints \
               and self.grid[state.position.x][state.position.y] != 1

    def is_transition_valid(self, state_1: State, state_2: State) -> bool:
        """
        Checks if a transition between two states is valid or not.
        :param state_1: Starting state
        :param state_2: Ending state
        :returns: bool, whether the transition is valid or not
        """
        return EdgeConstraint(state_1.time, state_1.position, state_2.position) not in self.constraints.edge_constraints

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Position(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Position(agent['goal'][0], agent['goal'][1]))
            self.agent_dict.update({agent['name']: {'start': start_state, 'goal': goal_state}})

    def compute_solution(self):
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent: local_solution})
        return solution

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.position.x - goal.position.x) + fabs(state.position.y - goal.position.y)

    def is_at_goal(self, state: State, agent_name) -> bool:
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Position(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Position(agent['goal'][0], agent['goal'][1]))
            self.agent_dict.update({agent['name']: {'start': start_state, 'goal': goal_state}})

    def compute_solution(self):
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent: local_solution})
        return solution

    def compute_solution_cost(
            self,
            solution):
        return sum([len(path) for path in solution.values()])
    

def extract_moves_from_solution(solution):
    # Actions
    WAIT = 0
    LEFT = 1
    RIGHT = 2
    DOWN = 3
    UP = 4
    UPLEFT = 5
    UPRIGHT = 6
    DOWNLEFT = 7
    DOWNRIGHT = 8

    moves = []
    single_move = []
    for r_ID, sol in solution.items():
        for s1, s2 in zip(sol[:-1], sol[1:]):
            _, q1 = s1
            x1, y1 = q1
            _, q2 = s2
            x2, y2 = q2

            dx = x2 - x1
            dy = y2 - y1

            if dx == 0 and dy == 0:
                single_move.append(WAIT)
            elif dx == 0 and dy == 1:
                single_move.append(UP)
            elif dx == 0 and dy == -1:
                single_move.append(DOWN)
            elif dx == 1 and dy == 0:
                single_move.append(RIGHT)
            elif dx == -1 and dy == 0:
                single_move.append(LEFT)
            elif dx == 1 and dy == 1:
                single_move.append(UPRIGHT)
            elif dx == 1 and dy == -1:
                single_move.append(DOWNRIGHT)
            elif dx == -1 and dy == 1:
                single_move.append(UPLEFT)
            elif dx == -1 and dy == -1:
                single_move.append(DOWNLEFT)
            else:
                raise ValueError("Wrong Move detected!")
        moves.append(single_move)
        single_move = []
    # Make them step moves for all agents at one step
    moves2 = [list(i) for i in zip_longest(*moves, fillvalue=0)]
    return moves2


def convert_move_to_direction(move):
    directions = {
        0: "WAIT",
        1: "LEFT",
        2: "RIGHT",
        3: "DOWN",
        4: "UP",
        5: "UPLEFT",
        6: "UPRIGHT",
        7: "DOWNLEFT",
        8: "DOWNRIGHT"
    }
    return directions[move]


def main():
    # Input the dimension of the maze
    dimension_of_maze = int(input("Enter the dimension of the maze: "))

    # Initialize the grid map with zeros
    grid = [[0 for _ in range(dimension_of_maze)] for _ in range(dimension_of_maze)]

    # Input the number of agents
    num_agents = int(input("Enter the count of agents: "))
    agents = []

    # Input the positions of agents
    for i in range(num_agents):
        agent_name = f'agent{i + 1}'
        start_row = int(input(f"Enter {agent_name}'s start row coordinate: "))
        start_column = int(input(f"Enter {agent_name}'s start column coordinate: "))
        goal_row = int(input(f"Enter {agent_name}'s goal row coordinate: "))
        goal_column = int(input(f"Enter {agent_name}'s goal column coordinate: "))
        agents.append({'name': agent_name, 'start': (start_row, start_column), 'goal': (goal_row, goal_column)})

    # Input the number of obstacles
    num_obstacles = int(input("Enter the number of obstacles: "))

    # Place obstacles on the grid map
    for _ in range(num_obstacles):
        obstacle_row = int(input("Enter obstacle's row coordinate: "))
        obstacle_column = int(input("Enter obstacle's column coordinate: "))
        grid[obstacle_row][obstacle_column] = 1

    # Create an instance of the Environment class
    env = Environment(grid, agents)

    # Create an instance of the CBS class
    cbs = CBS(env)

    # Search for a solution
    solution = cbs.search()

    if not solution:
        print("Solution not found")
    else:
        print("Solution found:")
        print(solution)
        '''
        moves = extract_moves_from_solution(solution)
        for i, agent_moves in enumerate(moves, start=1):
            print(f"Agent {i} moves:")
            for move in agent_moves:
                print(convert_move_to_direction(move))
        '''

if __name__ == "__main__":
    main()

