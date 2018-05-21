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
                search.search(path_search, path_search_start, search.AStarFringe(ManhattanDistanceHeuristic(self.player)))
            except search.SolutionNotFoundError:
                return False
        
        return True
        
    def __str__(self):
        return "{b: %s, p: %s}" % (list(map(str, self.boxes)), self.player)
        
    def __hash__(self):
        if self.hash_val is None:
            #self.hash_val = sum(x + y * self.instance.width for x, y in self.boxes)
            self.hash_val = hash((((x, y) for x, y in self.boxes), self.player))
            
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
                player_pos = (x + (2 * dx), y + (2 * dy))
                box_pos = (x + dx, y + dy)
                
                player_blocked = self.__is_blocked(current_grid, *player_pos)
                box_blocked = self.__is_blocked(current_grid, *box_pos)
                
                if player_blocked or box_blocked:
                    continue
                
                new_boxes = [(x_j, y_j) if j != i else (x_j + dx, y_j + dy) for j, (x_j, y_j) in enumerate(state.boxes)]
                    
                neighbor = GameState(self, new_boxes, player_pos)

                cost = 1
                
                if state.player:
                    neighbor_grid = self.get_grid_from_state(state)
                    path_search = GridSearchInstance(neighbor_grid, state.player)
                    path_search_start = GridSearchState(path_search, *box_pos)
                    
                    try:
                        path = search.search(path_search, 
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
        solution = search.search(instance, instance.start, search.UniformCostFringe())
        
        for i in range(0, len(solution)):
            print(solution[i].state, solution[i].cost)
            
            if i < len(solution) - 1:
                box = list(set(solution[i + 1].state.boxes) - set(solution[i].state.boxes))[0]
                interm_state = GameState(instance, solution[i].state.boxes, box)
                
                #print("\n".join(map(lambda l: "".join(l), instance.get_grid_from_state(interm_state))))
                print(interm_state)
                #print("\n".join(map(lambda l: "".join(l), instance.get_grid_from_state(solution[i + 1].state))))
        
        current_grid = instance.get_grid_from_state(instance.goal)
        
        path_search = GridSearchInstance(current_grid, solution[-1].state.player)
        path_search_start = GridSearchState(path_search, *instance.goal.player)
        
        path_to_start = search.search(path_search, 
                path_search_start, 
                search.AStarFringe(ManhattanDistanceHeuristic(solution[-1].state.player)))
                
        cost = solution.info["cost"] + path_to_start.info["cost"]
            
        print("solution length:", len(solution))
        print("solution cost:", cost)
        print(len(solution.info["nodes_generated"]), "nodes generated")
    except search.SolutionNotFoundError as e:
        print("no solution")
        print(len(e.fringe.nodes_generated), "nodes generated")

if __name__ == '__main__':
    main()