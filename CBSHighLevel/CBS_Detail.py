class Node:       
    def __init__(self, parent, position,t):
        self.parent = parent
        self.position = position
        self.t=t
        self.g = 0
        self.h = 0
        self.f = 0

class ConstraintTreeNode():     
    def __init__(self, solution,parent):
        self.parent=parent
        constraint= []
        self.constraint = constraint
        self.solution = solution
        self.cost= 0    

        
def AStar(grid, agent,constraints):
    start = Node(None, agent.start,0)
    open_ = []
    #open list consists of the nodes which are left for exploration in te A star algorithm and are possible the nodes with the lowest f costs
    open_.append(start)
    while open_:
        currNode = open_[0]
        currIndex = 0
        for i in range(1, len(open_)):
            if open_[i].f < currNode.f:
                currNode = open_[i]
                currIndex = i
        open_.pop(currIndex)
        if currNode.position == agent.end:
            path = []
            while currNode is not None:
                path.append(currNode.position)
                currNode = currNode.parent
            return path[::-1]
        dirs = ((0, 1), (1, 0), (0, -1), (-1, 0),(1,-1),(1,1),(-1,-1),(-1,1))
        for poss_neigh in dirs:  
            position = [currNode.position[0] + poss_neigh[0], currNode.position[1] + poss_neigh[1]]

            if position[0] < 0 or position[0] >= len(grid) or position[1] < 0 or position[1] >= len(grid):
                continue
            if grid[position[0]][position[1]] == 1:
                continue
            cNode = Node(currNode, position,currNode.t+1)  
            cNode.g = currNode.g + 1
            cNode.h = abs(cNode.position[0] - agent.end[0]) + abs(cNode.position[1] - agent.end[1])**2
            cNode.f = cNode.g + cNode.h
            if [cNode.position,cNode.t] in constraints:
                continue

            if any(node.position == cNode.position and cNode.g >= node.g for node in open_):
                continue
            open_.append(cNode)

def low_level(agents,grid,constraints):        # the low level part of the cbs algorithm
    solution=[]
    for i in range(len(agents)):
        solution.append(AStar(grid,agents[i],constraints[i]))
    return solution

def SIC(solution):
    cost=0
    for path in solution:
        cost+=len(path)-1
    return cost

def isValid(solution):
    for i in range(len(solution)):
        for j in range(i+1,len(solution)):
            for k in range(min(len(solution[i]),len(solution[j]))):
                if solution[i][k]==solution[j][k]:
                    return [i,j,solution[i][k],k]
    return 1

def CBS(grid,agents):    # THE CBS ALGORITHM
    root=ConstraintTreeNode([],None)
    p=[]
    for i in range(len(agents)):
        p.append([])
    root.solution=low_level(agents,grid,p)
    root.cost=SIC(root.solution)
    open_list=[]
    open_list.append(root)
    while open is not None:
        x=0
        min=float("inf")
        for i in range(len(open_list)):
            if open_list[i].cost<min:
                min=open_list[i].cost
                x=i
        n=open_list.pop(x)
        a=isValid(n.solution)
        if a==1: return n.solution
        conflict=a
        for i in [0, 1]:
            next_node=ConstraintTreeNode(n.solution,n)
            next_node.constraint=[conflict[i],conflict[2],conflict[3]]
            con=next_node
            constraints=[]
            for j in range(len(agents)):
                constraints.append([])
            while con.constraint!= []:
                constraint=con.constraint
                constraints[constraint[0]].append([constraint[1],constraint[2]])
                con=con.parent
            next_node.solution=low_level(agents,grid,constraints)
            open_list.append(next_node)
        
def main():    
    class Agent:     
        def __init__(self, start, end):
            self.start = start
            self.end = end

    dimension_of_grid = int(input("Enter the dimension of the grid: "))

    # Initialize the grid map with zeros
    grid = [[0 for _ in range(dimension_of_grid)] for _ in range(dimension_of_grid)]

    NAgents = int(input("Enter the count of agents: "))
    agents = []

    NObstacles = int(input("Enter the number of obstacles: "))
    for _ in range(NObstacles):
        obstacleRow = int(input("Enter obstacle's row coordinate: "))
        obstacleColumn = int(input("Enter obstacle's column coordinate: "))
        grid[obstacleRow][obstacleColumn] = 1

    for _ in range(NAgents):
        agentStartRow = int(input("Enter agent's starting row coordinate: "))  
        agentStartColumn = int(input("Enter agent's starting column coordinate: "))
        agentStopRow = int(input("Enter agent's stopping row coordinate: "))
        agentStopColumn = int(input("Enter agent's stopping column coordinate: "))
        agents.append(Agent([agentStartRow, agentStartColumn], [agentStopRow, agentStopColumn]))
    for i in CBS(grid, agents):
            print(i)

if __name__ == "__main__":
    main()
