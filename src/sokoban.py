import sys, search, itertools, time
from operator import itemgetter
from collections import defaultdict
from munkres import Munkres

BreadthFirstFringe = search.BreadthFirstFringe
UniformCostFringe = search.UniformCostFringe
AStarFringe = search.AStarFringe

class GridSearchState:
    def __init__(self, instance, x, y):
        self.instance = instance
        self.x, self.y = x, y
        
        self.hash_val = None
        self.neighbors = None
        
    def get_neighbors(self):
        if self.neighbors is None:
            self.neighbors = self.instance.generate_neighbors(self)
            
        return self.neighbors
        
    def __str__(self):
        return "(%d, %d)" % (self.x, self.y)
        
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
        
    def __hash__(self):
        if self.hash_val is None:
            self.hash_val = self.x + self.y * len(self.instance.grid[0])
            
        return self.hash_val

class GridSearchInstance:
    def __init__(self, grid, goal):
        self.grid = grid
        self.states = {}
        self.goal = GridSearchState(self, goal[0], goal[1])
        
    def _is_blocked(self, x, y):
        try:
            return self.grid[y][x] in ['@', 'b', 'B']
        except IndexError:
            return True
        
    def generate_neighbors(self, state):
        neighbors = []
        x, y = state.x, state.y
        
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            if not self._is_blocked(x + dx, y + dy):
                neighbor = GridSearchState(self, x + dx, y + dy)
                
                try:
                    neighbor = self.states[neighbor]
                except:
                    self.states[neighbor] = neighbor
                    
                neighbors.append((neighbor, 1))
                
        return neighbors
        
    def is_goal(self, state):
        return state == self.goal

class ManhattanDistanceHeuristic:
    def __init__(self, goal):
        self.goal = goal
    
    def get(self, node):
        dx = abs(node.state.x - self.goal[0])
        dy = abs(node.state.y - self.goal[1])
        
        return dx + dy
        
class MinMatchingHeuristic:
    def __init__(self, goal):
        self.goal = goal
        self.instance = goal.instance
        
    def _dist(self, first, second):
        dx = abs(first[0] - second[0])
        dy = abs(first[1] - second[1])
        
        return dx + dy
        
    def get(self, node):
        boxes = list(node.state.boxes)
        
        dist_mat = [[self._dist(box, goal) for box in boxes] for goal in self.goal.boxes]
        h_value = 0
        
        m = Munkres()
        indexes = m.compute(dist_mat)
        
        for row, column in indexes:
            h_value += dist_mat[row][column]
            
        return h_value

class MinMatchingActualPath(MinMatchingHeuristic):
    def _dist(self, first, second):
        try:
            grid = self.instance.empty_grid
            path_search = GridSearchInstance(grid, second)
            path_search_start = GridSearchState(path_search, *first)
            
            path = search.grid_search(path_search, 
                    path_search_start,
                    search.AStarFringe(ManhattanDistanceHeuristic(second)))
                    
            return path.info["cost"]
        except search.SolutionNotFoundError:
            return float("inf")

class MinMatchingPlayer(MinMatchingHeuristic):
    def get(self, node):
        boxes = list(node.state.boxes)
        player = node.state.player
        
        dist_mat = [[self._dist(box, goal) for box in boxes] for goal in self.goal.boxes]
        h_value = 0
        
        m = Munkres()
        indexes = m.compute(dist_mat)
        done_list = [False for _ in boxes]
        
        for goal, box in indexes:
            h_value += dist_mat[goal][box]
            
            if boxes[box] == self.goal.boxes[goal]:
                done_list[box] = True
        
        boxes_aux = []
        
        for box, done in zip(boxes, done_list):
            if not done:
                boxes_aux.append(box)
            
        boxes = list(boxes_aux)
        
        if player is not None:
            current_pos = player
        else:
            try:
                current_pos = boxes[0]
            except:
                pass
        
        while boxes:
            dist_to_boxes = [self._dist(current_pos, box) - 1 for box in boxes]
            next_box = dist_to_boxes.index(min(dist_to_boxes))
            
            h_value += dist_to_boxes[next_box]
            
            current_pos = boxes.pop(next_box)
            
        return h_value

class MinMatchingActualPlayerPath(MinMatchingActualPath, MinMatchingPlayer):
    pass

class XStrategy:
    def __init__(self, instance, y_strategy):
        self.instance = instance
        self.y_strategy = y_strategy
        self.goals = set()
        
        for y in range(0, len(self.instance.reversed_grid)):
            for x in range(0, len(self.instance.reversed_grid[y])):
                if self.instance.reversed_grid[y][x] in set(['.', 'B', 'O']):
                    self.goals.add((x, y))

class YStrategy:
    def __init__(self, goal):
        self.goal = goal

class AfterEachStep(XStrategy):
    def get(self, last_state, state, grid):
        return self.y_strategy.next_box(state, grid)

class UntilPlaced(XStrategy):
    def get(self, last_state, state, grid):
        if last_state is None:
            return self.y_strategy.next_box(state, grid)
        
        i = 0
        
        while i < len(state.boxes):
            if state.boxes[i] not in last_state.boxes:
                break
            
            i += 1
        else:
            return self.y_strategy.next_box(state, grid)
        
        box = state.boxes[i]
        
        if box not in self.goals:
            return [box]
        else:
            return self.y_strategy.next_box(state, grid)

def until_k_steps_away(k):
    class UntilKStepsAway(XStrategy):
        def get(self, last_state, state, grid):
            if last_state is None:
                return self.y_strategy.next_box(state, grid)
            
            i = 0
            
            while i < len(state.boxes):
                if state.boxes[i] not in last_state.boxes:
                    break
                
                i += 1
            else:
                return self.y_strategy.next_box(state, grid)
            
            box = state.boxes[i]
            
            distance = min([abs(box[0] - goal[0]) + abs(box[1] - goal[1]) for goal in self.goals])
            
            if distance > k:
                return [box]
            else:
                return self.y_strategy.next_box(state, grid)
    
    return UntilKStepsAway

class AllBoxes(YStrategy):
    def next_box(self, state, grid):
        return state.boxes
        
class UnplacedBoxes(YStrategy):
    def next_box(self, state, grid):
        return list(filter(lambda b: grid[b[1]][b[0]] != 'B', state.boxes))

class NodeSet:
    def __init__(self):
        self.nodes = defaultdict(list)
        
    def _get_hash(self, node):
        return tuple((x, y) for x, y in sorted(sorted(node.state.boxes, key=itemgetter(1)), key=itemgetter(0)))
    
    def __contains__(self, node):
        try:
            self.__getitem__(node)
            return True
        except IndexError:
            return False
    
    def __len__(self):
        return len(self.nodes)
    
    def add(self, node):
        state_hash = self._get_hash(node)
        
        if state_hash in self.nodes and node.state.player is not None:
            for similar in self.nodes[state_hash]:
                if similar.state.player is None: return None
                
                try:
                    grid = node.state.instance.get_grid_from_state(similar.state)
                    path_search = GridSearchInstance(grid, similar.state.player)
                    path_search_start = GridSearchState(path_search, *node.state.player)
                    
                    path = search.grid_search(path_search, 
                            path_search_start,
                            search.AStarFringe(ManhattanDistanceHeuristic(similar.state.player)))
                    
                    '''if node.cost < similar.cost:
                        self.nodes[state_hash].remove(similar)
                        break'''
                except search.SolutionNotFoundError:
                    continue
                
        return self.nodes[state_hash].append(node)

    def update(self, node_list):
        for node in node_list:
            self.add(node)

    def __setitem__(self, state, node):
        self.add(node)

    def __getitem__(self, node):
        state_hash = self._get_hash(node)
    
        if state_hash in self.nodes:
            if node.state.player is None: return self.nodes[state_hash][0]
            
            found = None
            
            for similar in self.nodes[state_hash]:
                if similar.state.player is None: return similar
                
                try:
                    grid = node.state.instance.get_grid_from_state(similar.state)
                    path_search = GridSearchInstance(grid, similar.state.player)
                    path_search_start = GridSearchState(path_search, *node.state.player)
                    
                    path = search.grid_search(path_search, 
                            path_search_start,
                            search.AStarFringe(ManhattanDistanceHeuristic(similar.state.player)))
                    #return similar
                    if not found or similar.cost + path.info["cost"] < found.cost:
                        found = similar
                except search.SolutionNotFoundError:
                    continue
                
            if found: return found
        
        raise IndexError()

    def union(self, other):
        return set(self.nodes.keys()).union(set(other.nodes.keys()))

class BreadthFirstSokobanFringe(BreadthFirstFringe):
    def __init__(self):
        super().__init__()
        self.visited = NodeSet()
        self.best_node = NodeSet()
        self.nodes_generated = NodeSet()

class UniformCostSokobanFringe(UniformCostFringe):
    def __init__(self):
        super().__init__()
        self.visited = NodeSet()
        self.best_node = NodeSet()
        self.nodes_generated = NodeSet()

class AStarSokobanFringe(AStarFringe):
    def __init__(self, heuristic):
        super().__init__(heuristic)
        self.visited = NodeSet()
        self.best_node = NodeSet()
        self.nodes_generated = NodeSet()

class GameState:
    def __init__(self, instance, boxes, player):
        self.instance = instance
        self.boxes = boxes
        self.player = player
        
        self.hash_val = None
        self.neighbors = None
    
    def get_neighbors(self):
        if self.neighbors is None:
            self.neighbors = self.instance.generate_neighbors(self)
            
        return self.neighbors
        
    def __eq__(self, other):
        for box in self.boxes:
            if not box in other.boxes:
                return False
        
        if self.player and other.player:
            current_grid = self.instance.get_grid_from_state(self)
            
            path_search = GridSearchInstance(current_grid, self.player)
            path_search_start = GridSearchState(path_search, other.player[0], other.player[1])
            
            try:
                search.grid_search(path_search, path_search_start, search.AStarFringe(ManhattanDistanceHeuristic(self.player)))
            except search.SolutionNotFoundError:
                return False
        
        return True
        
    def __str__(self):
        return "{b: %s, p: %s}" % (list(map(str, self.boxes)), self.player)
        
    def __hash__(self):
        if self.hash_val is None:
            #self.hash_val = sum(x + y * self.instance.width for x, y in self.boxes)
            self.hash_val = hash((((x, y) for x, y in sorted(sorted(self.boxes, key=itemgetter(1)), key=itemgetter(0))), self.player))
            
        return self.hash_val
        
    def diff(self, other):
        action = None
        
        for i, box in enumerate(self.boxes):
            if not box in other.boxes:
                move = (box, tuple(t1 - t2 for t1, t2 in zip(self.boxes[i], other.boxes[i])))
                break
            
        return move

class GameInstance:
    def __init__(self, width, height, grid, x_strategy = AfterEachStep, y_strategy = AllBoxes):
        self.grid = list(map(list, grid))
        self.width, self.height = width, height
        
        self.empty_grid = list(map(list, grid))
        self.reversed_grid = list(map(list, grid))
        
        swap = {'.': 'b', 'b': '.', 'O': 'b', 'o': ' '}
        
        for y in range(0, height):
            for x in range(0, width):
                self.empty_grid[y][x] = ' ' if self.empty_grid[y][x] != '@' else '@'
                
                try:
                    self.reversed_grid[y][x] = swap[self.reversed_grid[y][x]]
                except:
                    pass
        
        self.states = {}
        self.grids = {}
        
        self.start = self._get_state_from_grid(self.grid)
        self.goal = self._get_state_from_grid(self.reversed_grid)
        
        self.last_state = None
        self.boxes_to_consider = x_strategy(self, y_strategy(self.goal))
    
    def _get_state_from_grid(self, grid):
        boxes = []
        player = None
        
        for y in range(0, self.height):
            for x in range(0, self.width):
                if grid[y][x] in ['b', 'B']:
                    boxes.append((x, y))
                elif grid[y][x] in ['o', 'O']:
                    player = (x, y)
                    
        state = GameState(self, boxes, player)
        
        self.states[state] = state
        
        return state
    
    def get_grid_from_state(self, state):
        try:
            return self.grids[state]
        except:
            new_grid = list(map(list, self.empty_grid))
            
            swap_box = {'.': 'B', ' ': 'b', 'o': 'b', 'O': 'B', 'B': 'B', 'b': 'b'}
            swap_player = {'.': 'O', ' ': 'o', 'o': 'o', 'O': 'O'}
            
            for x, y in state.boxes:
                try:
                    new_grid[y][x] = swap_box[new_grid[y][x]]
                except:
                    raise ValueError("invalid state")
                    
            if state.player:
                x, y = state.player
                try:
                    new_grid[y][x] = swap_player[new_grid[y][x]]
                except:
                    raise ValueError("invalid state")
                    
            self.grids[state] = new_grid
            
            return self.grids[state]
    
    def _is_blocked(self, grid, x, y):
        try:
            return grid[y][x] in ['@', 'b', 'B']
        except IndexError:
            return True
    
    def generate_neighbors(self, state):
        neighbors = []
        
        current_grid = self.get_grid_from_state(state)
        available_boxes = self.boxes_to_consider.get(self.last_state, state, current_grid)
        last_action = self.last_state.diff(state) if self.last_state else None
        
        for x, y in available_boxes:
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                if ((x, y), (dx, dy)) == last_action:
                    continue
                
                old_player_pos = (x - dx, y - dy)
                new_player_pos = (x, y)
                box_pos = (x + dx, y + dy)
                
                player_blocked = self._is_blocked(current_grid, *old_player_pos)
                box_blocked = self._is_blocked(current_grid, *box_pos)
                
                if player_blocked or box_blocked:
                    continue
                
                new_boxes = [(x_j, y_j) if (x_j, y_j) != (x, y) else (x_j + dx, y_j + dy) for x_j, y_j in state.boxes]
                
                neighbor = GameState(self, new_boxes, new_player_pos)
                
                cost = 1
                
                if state.player and state.player != old_player_pos:
                    neighbor_grid = self.get_grid_from_state(state)
                    path_search = GridSearchInstance(neighbor_grid, state.player)
                    path_search_start = GridSearchState(path_search, *old_player_pos)
                    
                    try:
                        path = search.grid_search(path_search, 
                                path_search_start,
                                search.AStarFringe(ManhattanDistanceHeuristic(state.player)))
                        cost = path.info["cost"] + 1
                    except search.SolutionNotFoundError:
                        continue
                
                try:
                    neighbor = self.states[neighbor]
                except:
                    self.states[neighbor] = neighbor
                
                neighbors.append((neighbor, cost))
                
        return neighbors
    
    def dist(self, first, second):
        if first.player is None or second.player is None:
            return 0
        
        try:
            grid = first.instance.get_grid_from_state(first)
            path_search = GridSearchInstance(grid, second.player)
            path_search_start = GridSearchState(path_search, *first.player)
            
            path = search.grid_search(path_search, 
                    path_search_start,
                    search.AStarFringe(ManhattanDistanceHeuristic(second.player)))
                    
            return path.info["cost"]
        except search.SolutionNotFoundError:
            return float("inf")
    
    def is_goal(self, state):
        return state == self.goal

class ReversedGameInstance(GameInstance):
    def __init__(self, width, height, grid, x_strategy = AfterEachStep, y_strategy = AllBoxes):
        super().__init__(width, height, grid, x_strategy, y_strategy)
        
        self.grid, self.reversed_grid = self.reversed_grid, self.grid
        self.start, self.goal = self.goal, self.start
        
        self.boxes_to_consider = x_strategy(self, y_strategy(self.goal))
        
    def generate_neighbors(self, state):
        neighbors = []
        
        current_grid = self.get_grid_from_state(state)
        available_boxes = self.boxes_to_consider.get(self.last_state, state, current_grid)
        last_action = self.last_state.diff(state) if self.last_state else None
        
        for x, y in available_boxes:
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                if ((x, y), (dx, dy)) == last_action:
                    continue
                
                player_pos = (x + (2 * dx), y + (2 * dy))
                box_pos = (x + dx, y + dy)
                
                player_blocked = self._is_blocked(current_grid, *player_pos)
                box_blocked = self._is_blocked(current_grid, *box_pos)
                
                if player_blocked or box_blocked:
                    continue
                
                new_boxes = [(x_j, y_j) if (x_j, y_j) != (x, y) else (x_j + dx, y_j + dy) for x_j, y_j in state.boxes]
                    
                neighbor = GameState(self, new_boxes, player_pos)

                cost = 1
                
                if state.player and state.player != box_pos:
                    neighbor_grid = self.get_grid_from_state(state)
                    path_search = GridSearchInstance(neighbor_grid, state.player)
                    path_search_start = GridSearchState(path_search, *box_pos)
                    
                    try:
                        path = search.grid_search(path_search, 
                                path_search_start, 
                                search.AStarFringe(ManhattanDistanceHeuristic(state.player)))
                        cost = path.info["cost"] + 1
                    except search.SolutionNotFoundError:
                        continue
                
                try:
                    neighbor = self.states[neighbor]
                except:
                    self.states[neighbor] = neighbor
                    
                neighbors.append((neighbor, cost))
        
        return neighbors

def solve_uni(instance, search_strategy):
    solution = search.search(instance, instance.start, search_strategy, True)
    
    for i in range(0, len(solution)):
        print(solution[i].state, solution[i].cost)
    
    current_grid = instance.get_grid_from_state(instance.goal)
    
    path_search = GridSearchInstance(current_grid, solution[-1].state.player)
    path_search_start = GridSearchState(path_search, *instance.goal.player)
    
    path_to_start = search.grid_search(path_search, 
            path_search_start, 
            search.AStarFringe(ManhattanDistanceHeuristic(solution[-1].state.player)))
            
    solution.info["cost"] += path_to_start.info["cost"]
    
    return solution

def solve_bid(instance, search_strategy, instance2, search_strategy2):
    solution = search.bidirectional_search([instance, instance2], 
                [instance.start, instance2.start], 
                [search_strategy, search_strategy2], True)
    
    dist = MinMatchingActualPlayerPath(instance.goal)
    
    for i in range(0, len(solution)):
        print(solution[i].state, solution[i].cost, dist.get(solution[i]))
        
    return solution

def main():
    raw_data = sys.argv
    
    if len(raw_data) not in (4, 5):
        print("usage: %s instance algorithm strategy parameter?" % raw_data[0])
        sys.exit(1)
    
    instance, algorithm, strategy = raw_data[1:4]
    
    try:
        parameter = raw_data[4]
    except:
        parameter = None
    
    with open(instance, 'r') as file:
        data = file.readlines()
        
    width, height = list(map(int, data[0].split()))
    grid = list(map(lambda l: list(l.rstrip('\n')), data[1:]))
    
    if strategy == 'x1y1':
        instance = GameInstance(width, height, grid, AfterEachStep, AllBoxes)
        instance2 = ReversedGameInstance(width, height, grid, AfterEachStep, AllBoxes)
    elif strategy == 'x3y2':
        instance = GameInstance(width, height, grid, UntilPlaced, UnplacedBoxes)
        instance2 = ReversedGameInstance(width, height, grid, UntilPlaced, UnplacedBoxes)
    elif strategy == 'x4y2':
        if parameter:
            instance = GameInstance(width, height, grid, until_k_steps_away(int(parameter)), UnplacedBoxes)
            instance2 = ReversedGameInstance(width, height, grid, until_k_steps_away(int(parameter)), UnplacedBoxes)
        else:
            print("invalid parameter for x4y2")
            sys.exit(1)
    else:
        print("invalid strategy")
        sys.exit(1)
        
    try:
        start_time = time.time()
        
        if algorithm == 'baseline':
            solution = solve_uni(instance2, BreadthFirstSokobanFringe())
        elif algorithm == 'proposed':
            solution = solution = solve_bid(instance, 
                AStarSokobanFringe(MinMatchingActualPlayerPath(instance.goal)), 
                instance2, 
                AStarSokobanFringe(MinMatchingActualPlayerPath(instance.goal)))
        else:
            print("invalid algorithm")
            sys.exit(1)
            
        end_time = time.time()
    
        print("solution length:", len(solution))
        print("solution cost:", solution.info["cost"])
        print(len(solution.info["nodes_expanded"]), "nodes expanded")
        print(len(solution.info["nodes_generated"]), "nodes generated")
        print("time elapsed: %g" % (end_time - start_time))
    except search.SolutionNotFoundError as e:
        print("no solution")
        print(len(e.fringe.nodes_generated), "nodes generated")

if __name__ == '__main__':
    main()