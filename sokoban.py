import sys, search

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
        
    def __is_blocked(self, x, y):
        try:
            return self.grid[y][x] in ['@', 'b', 'B']
        except IndexError:
            return True
        
    def generate_neighbors(self, state):
        neighbors = []
        x, y = state.x, state.y
        
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            if not self.__is_blocked(x + dx, y + dy):
                try:
                    neighbor = self.states[(x + dx, y + dy)]
                except:
                    neighbor = GridSearchState(self, x + dx, y + dy)
                    self.states[(x + dx, y + dx)] = neighbor
                    
                neighbors.append((neighbor, 1))
                
        return neighbors
        
    def is_goal(self, state):
        return state == self.goal

class ManhattanDistanceHeuristic:
    def __init__(self, goal):
        self.goal = goal
    
    def get(self, node):
        dx = abs(node.state.x - self.goal.x)
        dy = abs(node.state.y - self.goal.y)
        
        return dx + dy

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
        
        #if player and search(instance, self, BestFirstFringe(ManhattanDistanceHeuristic(other))):
        
        return True
        
    def __str__(self):
        return "{b: %s, p: %s}" % (list(map(str, self.boxes)), self.player)
        
    def __hash__(self):
        if self.hash_val is None:
            self.hash_val = sum(x + y * self.instance.width for x, y in self.boxes)
            
        return self.hash_val

class GameInstance:
    def __init__(self, width, height, grid):
        self.grid = list(map(list, grid))
        self.width, self.height = width, height
        
        swap = {'.': 'b', 'b': '.', 'O': 'b', 'o': ' ',}
        self.reversed_grid = list(map(list, grid))
        self.empty_grid = list(map(list, grid))
        
        for y in range(0, height):
            for x in range(0, width):
                try:
                    self.reversed_grid[y][x] = swap[self.reversed_grid[y][x]]
                except:
                    pass
                
                self.empty_grid[y][x] = ' ' if self.empty_grid[y][x] != '@' else '@'
                
        self.states = {}
                
        self.start = self.__get_state_from_grid(self.reversed_grid)
        self.goal = self.__get_state_from_grid(self.grid)

    def __get_state_from_grid(self, grid):
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
        
        return new_grid
        
    def __is_blocked(self, grid, x, y):
        try:
            return grid[y][x] in ['@', 'b', 'B']
        except IndexError:
            return True
        
    def generate_neighbors(self, state):
        neighbors = []
        
        current_grid = self.get_grid_from_state(state)
        
        for i, (x, y) in enumerate(state.boxes):
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                player_blocked = self.__is_blocked(current_grid, x + (2 * dx), y + (2 * dy))
                player_blocked = player_blocked or (x + (2 * dx), y + (2 * dy)) in state.boxes
                
                if state.player:
                    path_search = GridSearchInstance(current_grid, state.player)
                    path_search_start = GridSearchState(path_search, x + (2 * dx), y + (2 * dy))
                    
                    try:
                        search.search(path_search, path_search_start, search.AStarFringe(ManhattanDistanceHeuristic(path_search_start)))
                    except search.SolutionNotFoundError:
                        player_blocked = True
                
                box_blocked = self.__is_blocked(current_grid, x + dx, y + dy)
                box_blocked = box_blocked or (x + dx, y + dy) in state.boxes
                
                if not player_blocked and not box_blocked:
                    new_boxes = [(x_j, y_j) if j != i else (x_j + dx, y_j + dy) for j, (x_j, y_j) in enumerate(state.boxes)]
                    
                    try:
                        neighbor = self.states[sum((x + dx) + (y + dy) * self.width for x, y in new_boxes)]
                    except:
                        neighbor = GameState(self, new_boxes, (x + (2 * dx), y + (2 * dy)))
                        self.states[neighbor] = neighbor
                        
                    # unitary cost
                    neighbors.append((neighbor, 1))
                    
        return neighbors
        
    def is_goal(self, state):
        return state == self.goal

def main():
    raw_data = sys.argv
    
    if len(raw_data) != 2:
        print("usage: %s instance" % raw_data[0])
        sys.exit(1)
    
    instance = raw_data[1]
    
    with open(instance, 'r') as file:
        data = file.readlines()
        
    width, height = list(map(int, data[0].split()))
    grid = list(map(lambda l: list(l.rstrip('\n')), data[1:]))

    instance = GameInstance(width, height, grid)
    
    try:
        solution = search.search(instance, instance.start, search.BreadthFirstFringe())
    
        for step in solution:
            print(step.state)
            
        print("solution length:", solution[-1].depth)
        print(len(solution.info["nodes_generated"]), "nodes generated")
    except search.SolutionNotFoundError as e:
        print("no solution")
        print(len(e.fringe.nodes_generated), "nodes generated")

if __name__ == '__main__':
    main()