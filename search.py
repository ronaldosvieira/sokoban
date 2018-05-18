from collections import deque, defaultdict
import heapq

class Node:
    def __init__(self, state, pred = None, cost = 0, depth = 0):
        self.state = state
        self.pred = pred
        self.cost = cost
        self.depth = depth
        
    def __lt__(self, other):
        return self.cost < other.cost
        
    def __gt__(self, other):
        return self.cost > other.cost
        
    def __str__(self):
        return "<%s %g>" % (str(self.state), self.cost)

class Solution:
    def __init__(self, goal, fringe, visited):
        self.steps = []
        self.info = {}
        
        self.info["nodes_generated"] = fringe.nodes()
        self.info["nodes_expanded"] = visited
        self.info["depth"] = goal.depth
        self.info["cost"] = goal.cost
        i = goal
        
        while i:
            self.steps.append(i)
            i = i.pred
            
        self.steps.reverse()
        
    def __getitem__(self, key):
        return self.steps[key]
            
    def __str__(self):
        return " ".join(list(map(lambda s: "<%d, %d, %g>" % (s.state.label[0], s.state.label[1], s.cost), self.steps)))

class SolutionNotFoundError(Exception):
    def __init__(self, fringe):
        self.fringe = fringe

class InvalidGoalError(Exception):
    def __init__(self, message):
        super().__init__(message)

class ManhattanDistanceHeuristic:
    def __init__(self, goal):
        self.goal = goal
    
    def get(self, node):
        dx = abs(node[0] - self.goal[0])
        dy = abs(node[1] - self.goal[1])
        
        return dx + dy

class OctileDistanceHeuristic:
    def __init__(self, goal):
        self.goal = goal
        
    def get(self, node):
        dx = abs(node[0] - self.goal[0])
        dy = abs(node[1] - self.goal[1])
        
        return max(dx, dy) + 0.5 * min(dx, dy)

class Fringe(object):
    def __init__(self):
        self.nodes_generated = set()
        self.visited = set()
        
        self.best_cost = defaultdict(lambda: float("inf"))
        
    def init(self, nodes):
        self.nodes_generated.update(set(nodes))
        
    def extend(self, nodes):
        for node in nodes:
            self.nodes_generated.add(node)
        
    def nodes(self):
        return self.nodes_generated

class BreadthFirstFringe(Fringe):
    def __init__(self):
        super().__init__()
        self.open_list = deque([])
        
    def __str__(self):
        return str(list(map(str, self.open_list)))
        
    def __len__(self):
        return len(self.open_list)
        
    def init(self, nodes):
        super().init(nodes)
        self.open_list = deque(list(nodes))
        
    def pop(self):
        return self.open_list.popleft()
        
    def extend(self, nodes):
        super().extend(nodes)
        self.open_list.extend(nodes)
        
class UniformCostFringe(Fringe):
    def __init__(self):
        super().__init__()
        self.open_list = []
            
    def __str__(self):
        return str(list(map(str, map(lambda n: n[1], self.open_list))))
        
    def __len__(self):
        return len(self.open_list)
        
    def init(self, nodes):
        super().init(nodes)
        self.open_list = []
        
        for node in nodes:
            heapq.heappush(self.open_list, (node.cost, node))
        
    def pop(self):
        return heapq.heappop(self.open_list)[1]
        
    def extend(self, nodes):
        super().extend(nodes)
        
        for node in nodes:
            heapq.heappush(self.open_list, (node.cost, node))

class LimitedDepthFirstFringe(Fringe):
    def __init__(self, limit):
        super().__init__()
        
        self.limit = limit
        
        self.open_list = []
        self.filtered_out = []
        
        self.initialized = False
        
    def __str__(self):
        return str(list(map(str, self.open_list)))
        
    def __len__(self):
        return len(self.open_list)
        
    def init(self, nodes):
        if not self.initialized:
            within_limit = lambda n: n.cost <= self.limit
            
            self.filtered_out = [] + list(reversed(list(filter(lambda n: not within_limit(n), nodes))))
            nodes = filter(within_limit, nodes)
            nodes = list(nodes)
            
            super().init(nodes)
            self.open_list = [] + nodes
            
            self.initialized = True
        
    def pop(self):
        return self.open_list.pop()
        
    def extend(self, nodes):
        within_limit = lambda n: n.cost <= self.limit
        
        self.filtered_out.extend(list(reversed(list(filter(lambda n: not within_limit(n), nodes)))))
        nodes = filter(within_limit, nodes)
        nodes = list(nodes)
        
        super().extend(nodes)
        self.open_list.extend(nodes)

class BestFirstFringe(Fringe):
    def __init__(self, heuristic):
        super().__init__()
        self.open_list = []
        self.heuristic = heuristic
            
    def __str__(self):
        return str(list(map(str, map(lambda n: n[1], self.open_list))))
        
    def __len__(self):
        return len(self.open_list)
        
    def init(self, nodes):
        super().init(nodes)
        self.open_list = []
        
        for node in nodes:
            heapq.heappush(self.open_list, (self.heuristic.get(node), node))
        
    def pop(self):
        return heapq.heappop(self.open_list)[1]
        
    def extend(self, nodes):
        super().extend(nodes)
        
        for node in nodes:
            heapq.heappush(self.open_list, (self.heuristic.get(node), node))

class AStarFringe(Fringe):
    def __init__(self, heuristic):
        super().__init__()
        self.open_list = []
        self.heuristic = heuristic
            
    def __str__(self):
        return str(list(map(str, map(lambda n: n[1], self.open_list))))
        
    def __len__(self):
        return len(self.open_list)
        
    def init(self, nodes):
        super().init(nodes)
        self.open_list = []
        
        for node in nodes:
            heapq.heappush(self.open_list, (node.cost + self.heuristic.get(node), node))
        
    def pop(self):
        return heapq.heappop(self.open_list)[1]
        
    def extend(self, nodes):
        super().extend(nodes)
        
        for node in nodes:
            heapq.heappush(self.open_list, (node.cost + self.heuristic.get(node), node))

def search(instance, start, fringe):
    fringe.init([Node(instance.start)])
    
    while fringe:
        current = fringe.pop()
        
        print("current:", current)
        
        if instance.is_goal(current.state):
            return Solution(current, fringe, fringe.visited)
            
        if current.state not in fringe.visited or current.cost <= fringe.best_cost[current.state]:
            fringe.visited.add(current.state)
            
            fringe.best_cost[current.state] = current.cost
            
            successors = map(lambda s: Node(s[0], current, current.cost + s[1], current.depth + 1), 
                                current.state.get_neighbors())
            successors = filter(lambda n: n.cost < fringe.best_cost[n.state], successors)
            successors = list(successors)
            
            print("successors:", list(map(str, successors)))
            print()
            
            for node in successors:
                fringe.best_cost[node.state] = min(fringe.best_cost[node.state], node.cost)
            
            fringe.extend(successors)
    
    raise SolutionNotFoundError(fringe)