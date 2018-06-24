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
    def __init__(self, goal, generated, visited):
        self.steps = []
        self.info = {}
        
        self.info["nodes_generated"] = generated
        self.info["nodes_expanded"] = visited
        self.info["depth"] = goal.depth
        self.info["cost"] = goal.cost
        i = goal
        
        while i:
            self.steps.append(i)
            i = i.pred
            
        self.steps.reverse()
        
    def __len__(self):
        return len(self.steps)
        
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

class Fringe(object):
    def __init__(self):
        self.nodes_generated = set()
        self.visited = set()
        
        self.best_cost = defaultdict(lambda: float("inf"))
        self.best_node = dict()
        
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

class DepthFirstFringe(Fringe):
    def __init__(self):
        super().__init__()
        self.open_list = []
        
    def __str__(self):
        return str(list(map(str, self.open_list)))
        
    def __len__(self):
        return len(self.open_list)
        
    def init(self, nodes):
        super().init(nodes)
        self.open_list = [] + nodes
        
    def pop(self):
        return self.open_list.pop()
        
    def extend(self, nodes):
        super().extend(nodes)
        self.open_list.extend(nodes)

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
            h = self.heuristic.get(node)
            heapq.heappush(self.open_list, ((node.cost + h, h), node))
        
    def pop(self):
        return heapq.heappop(self.open_list)[1]
        
    def extend(self, nodes):
        super().extend(nodes)
        
        for node in nodes:
            h = self.heuristic.get(node)
            heapq.heappush(self.open_list, ((node.cost + h, h), node))

def grid_search(instance, start, fringe, debug = False):
    k = -1
    
    fringe.init([Node(start)])
    
    while fringe:
        current = fringe.pop()
        
        if debug:
            if current.cost > k:
                k = current.cost
                print(k)
        
        if instance.is_goal(current.state):
            return Solution(current, fringe, fringe.visited)
            
        if current.state not in fringe.visited or current.cost <= fringe.best_cost[current.state]:
            fringe.visited.add(current.state)
            
            fringe.best_cost[current.state] = current.cost
            
            try:
                instance.last_state = current.pred.state
            except:
                instance.last_state = None
            
            successors = map(lambda s: Node(s[0], current, current.cost + s[1], current.depth + 1), 
                                current.state.get_neighbors())
            successors = filter(lambda n: n.cost < fringe.best_cost[n.state], successors)
            successors = list(successors)
            
            for node in successors:
                fringe.best_cost[node.state] = min(fringe.best_cost[node.state], node.cost)
            
            fringe.extend(successors)
    
    raise SolutionNotFoundError(fringe)

def search(instance, start, fringe, debug = False):
    k = -1
    
    fringe.init([Node(start)])
    
    while fringe:
        current = fringe.pop()
        
        if debug:
            if current.cost > k:
                k = current.cost
                print(k)
        
        if instance.is_goal(current.state):
            return Solution(current, fringe.nodes_generated, fringe.visited)
            
        if current not in fringe.visited or current.cost <= fringe.best_node[current].cost:
            fringe.visited.add(current)
            fringe.best_node.add(current)
            
            try:
                instance.last_state = current.pred.state
            except:
                instance.last_state = None
            
            successors = map(lambda s: Node(s[0], current, current.cost + s[1], current.depth + 1), 
                                current.state.get_neighbors())
            successors = filter(lambda n: n not in fringe.best_node or n.cost < fringe.best_node[n].cost, successors)
            successors = list(successors)
            
            for node in successors:
                fringe.best_node.add(node)
            
            fringe.extend(successors)
    
    raise SolutionNotFoundError(fringe)

def build_bidirectional_solution(solutions, fringes):
    left, right = solutions
    
    print("chosen:", left, right)
    
    right = right.pred
        
    while right:
        neighbor = next(filter(lambda s: s[0] == right.state, left.state.get_neighbors()))
        left = Node(neighbor[0], left, left.cost + neighbor[1], left.depth + 1)
        
        right = right.pred
    
    return Solution(left, fringes[0].nodes_generated.union(fringes[1].nodes_generated), fringes[0].visited.union(fringes[1].visited))

def combine_cost(result):
    return result[0] + result[1].cost

def bidirectional_search(instances, starts, fringes, debug = False):
    k = [0, 0]
    last_cost = [0, 0]
    direction = 0
    
    shortest = float("inf")
    sol = (None, None)
    
    for i in [0, 1]:
        fringes[i].init([Node(starts[i])])
    
    while all(fringes):
        instance, fringe = instances[direction], fringes[direction]
        
        current = fringe.pop()
        
        last_cost[direction] = max(last_cost[direction], current.cost)
        
        if debug:
            if current.cost > k[direction]:
                k[direction] = current.cost
                print(sum(k))
                
        if sum(last_cost) >= shortest:
            return build_bidirectional_solution(sol, fringes)
        
        if current not in fringe.visited or current.cost <= combine_cost(fringe.best_node[current]):
            fringe.visited.add(current)
            fringe.best_node.add(current)
            
            if current in fringes[1 - direction].visited:
                n1 = fringes[0].best_node[current][1]
                n2 = fringes[1].best_node[current][1]
                
                combined_cost = n1.cost + instance.dist(n1.state, n2.state) + n2.cost
                
                if combined_cost < shortest:
                    shortest = combined_cost
                    sol = (n1, n2)
            
            instance.last_state = current.pred.state if current.pred else None
            
            successors = map(lambda s: Node(s[0], 
                                    current, 
                                    current.cost + s[1], 
                                    current.depth + 1),
                                current.state.get_neighbors())
            successors = list(filter(lambda n: n not in fringe.best_node or n.cost < combine_cost(fringe.best_node[n]), successors))
            successors = list(successors)
            
            for node in successors:
                fringe.best_node.add(node)
                
            fringe.extend(successors)
            
        direction = 1 - direction
        
    if sol[0] is not None and sol[1] is not None:
        return build_bidirectional_solution(sol, fringes)
    
    raise SolutionNotFoundError(fringe)