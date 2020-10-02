# Giới thiệu
- Họ và tên : Chu Văn Hưng
- Mã SV : 18020020

# Pacman Search
## Question 1 (3 points): Finding a Fixed Food Dot using Depth First Search
### search.py
```py
def depthFirstSearch(problem):
    stores = util.Stack()
    start = problem.getStartState()
    visited = []
    stores.push((start, []))
    while not stores.isEmpty():
        top, direction = stores.pop()
        if problem.isGoalState(top):
            return direction
        if top not in visited:
            visited.append(top)
            for node, move, cost in problem.getSuccessors(top):
                newDirection = direction + [move]
                stores.push((node, newDirection))
```
Sử dụng stack để lưu lại các toạ độ các điểm và cách đi. Khởi tạo ban đầu trong stack là điểm bắt đầu của Pacman,và cách đi là rỗng. Tại mỗi điểm đầu của stack, ta đánh dấu điểm đó đã được đến, và duyệt tất cả các điểm có thể đi tiếp bằng hàm getSuccessors. Với các điểm có thể đi tiếp đó, ta lưu lại vị trí và cách đi và stack, trong đó cách đi bằng cách đi của điểm đầu stack + cách đi từ điểm đầu stack đến điểm đang xét.

## Question 2 (3 points): Breadth First Search
### search.py
```py
def breadthFirstSearch(problem):
    stores = util.Queue()
    start = problem.getStartState()
    visited = []
    stores.push((start, []))
    while not stores.isEmpty():
        top, direction = stores.pop()
        if problem.isGoalState(top):
            return direction
        if top not in visited:
            visited.append(top)
            for node, move, cost in problem.getSuccessors(top):
                newDirection = direction + [move]
                stores.push((node, newDirection))
```
Tương tự như Question 1, thay vì dùng Stack thì ta dùng Queue để lưu lại vị trí và cách đi tại mỗi điểm có thể đến của pacman.

## Question 3 (3 points): Varying the Cost Function
### search.py
```py
def uniformCostSearch(problem):
    stores = util.PriorityQueue()
    start = problem.getStartState()
    visited = []
    stores.push((start, [], 0), 0)
    while not stores.isEmpty():
        top, direction, costs = stores.pop()
        if problem.isGoalState(top):
            return direction
        if top not in visited:
            visited.append(top)
            for node, move, cost in problem.getSuccessors(top):
                newDirection = direction + [move]
                newCost = costs + cost
                stores.push((node, newDirection, newCost), newCost)
```
Tương tự như Question 1, nhưng thay vì dùng Stack thì ta dùng PriorityQueue để lưu lại vị trí, cách đi của pacman và tổng quãng đường đi đến điểm đó. Khi gặp điểm mới, quãng đường sẽ bằng quãng đường đã đi từ vị trí bắt đầu đến vị trí top queue + quãng đường tự top queue đến điểm đang xét. Và độ ưu tiên của priority queue chính là độ dài quãng đường đó.

## Question 4 (3 points): A* search
### search.py
```py
def aStarSearch(problem, heuristic=nullHeuristic):
    stores = util.PriorityQueue()
    start = problem.getStartState()
    if problem.isGoalState(start):
        return []
    visited = []
    stores.push((start, [], 0), 0)
    while not stores.isEmpty():
        top, direction, costs = stores.pop()
        if problem.isGoalState(top):
            return direction
        if top not in visited:
            visited.append(top)
            for node, move, cost in problem.getSuccessors(top):
                newDirection = direction + [move]
                newCost = costs + cost
                newHeuristicCost = newCost + heuristic(node, problem)
                stores.push((node, newDirection, newCost), newHeuristicCost)
```
Tương tự như Question 3, nhưng ở đây , độ ưu tiên được tính bằng quãng đường đến một điểm + heuristic(node, problem) với heuristic là hàm trả về giá trị dự đoán nếu đi từ điểm đó đến đích.

## Question 5 (3 points): Finding All the Corners
### searchAgents.py
```py
def getStartState(self):
    return (self.startingPosition, [])
```
Hàm trả về trạng thái bắt đầu, trong đó startingPosition là vị trí bắt đầu

```py
def isGoalState(self, state):
  pos = state[0]
  visited = state[1]
  if pos in self.corners:
      if pos not in visited:
          visited.append(pos)
      return len(visited)==4
  else:
      return False 
```
Nếu các góc của state đã được thăm = 4 thì trả về True

```py
def getSuccessors(self, state):
  x,y = state[0]
  visited = state[1]
  successors = []
  for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
      dx, dy = Actions.directionToVector(action)
      nextx, nexty = int(x + dx), int(y + dy)
      next_node = (nextx, nexty)
      hitsWall = self.walls[nextx][nexty]
      if not hitsWall:
          sucVCorners = list(visited) 
          if next_node in self.corners:
              if next_node not in sucVCorners:
                  sucVCorners.append( next_node )
          successor = ((next_node, sucVCorners), action, 1)
          successors.append(successor)
  self._expanded += 1 # DO NOT CHANGE
  return successors
```
Trong hàm này, ta duyệt theo 4 hướng của điểm đang xét. Với mỗi hướng, ta lấy ra điểm tiếp theo và kiểm tra xem có phải là tường không. Nếu điểm đó không phải tường và là góc chưa có trong list thì thêm vào list

## Question 6 (3 points): Corners Problem: Heuristic
### searchAgents.py
```py
def cornersHeuristic(state, problem):
    corners = problem.corners # These are the corner coordinates
    walls = problem.walls # These are the walls of the maze, as a Grid (game.py)

    node = state[0]
    visited = state[1]
    cost = 0
    unVisited = []
    for i in range(4):
        if corners[i] not in visited:
            unVisited.append(corners[i])

    currentPos = node
    while(len(unVisited)!=0):
        distance, corner = min( [(util.manhattanDistance(currentPos ,corner),corner) for corner in unVisited] )
        cost = cost + distance
        currentPos = corner
        unVisited.remove(corner) 

    return cost
```
Ban đầu, kiểm tra 4 corners, nếu chưa được thăm thì cho vào list. Với list corners, ta tìm điểm có khoảng cách nhỏ nhất với điểm đang xét, sau đó cộng chi phí với khoảng cách đó rồi đặt điểm đang xét thành điểm đó và xoá điểm đó khỏi list. Lặp lại quá trình đó đến khi không còn điểm nào trong list. 

## Question 7 (4 points): Eating All The Dots
### searchAgents.py
```py
def foodHeuristic(state, problem):
    position, foodGrid = state
    foodList = foodGrid.asList()
    cost = -1
    maxPos = position
    for food in foodList:
        if cost < util.manhattanDistance(position, food):
            cost = util.manhattanDistance(position, food)
            maxPos = food
    dim = position[0] - maxPos[0]
    foodCount=0
    for food in foodGrid.asList():
        if dim>0:
            if (position[0]-food[0])<0 :
                foodCount+=1
        elif dim<0:
            if (position[0]-food[0])>0 :
                foodCount+=1
        else:
            if (position[0]-food[0])!=0 :
                foodCount+=1
    if cost < 0:
        cost = 0
    return cost + foodCount
    ```
    Ý tưởng: Từ vị trí đang xét, ta tìm điểm có thức ăn xa nhất. Nếu điểm thức ăn xa nhất ở bên trên điểm đang xét , ta cộng khoảng cách xa nhất đó với số điểm có thức ăn ở dưới điểm đang xét và ngược lại. Trong trường hợp điểm xa nhất cùng hàng với điểm đang xét , ta cộng khoảng cách với tất cả các điểm có thức ăn nằm ngoài hàng đó.
    
    
## Question 8 (3 points): Suboptimal Search
### searchAgents.py
```py
def findPathToClosestDot(self, gameState):
    startPosition = gameState.getPacmanPosition()
    food = gameState.getFood()
    walls = gameState.getWalls()
    problem = AnyFoodSearchProblem(gameState)

    return search.bfs(problem)
```
Sử dụng lại thuật toán bfs ở Question 2 để tìm đường đi

# Các câu lệnh
### Question 1
```
python pacman.py -l tinyMaze -p SearchAgent -a fn=tinyMazeSearch
python pacman.py -l tinyMaze -p SearchAgent
python pacman.py -l mediumMaze -p SearchAgent
python pacman.py -l bigMaze -z .5 -p SearchAgent
```
### Question 2
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5
python eightpuzzle.py
```
### Question 3
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs
python pacman.py -l mediumDottedMaze -p StayEastSearchAgent
python pacman.py -l mediumScaryMaze -p StayWestSearchAgent
```
### Question 4
```
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
```
### Question 5
```
python pacman.py -l tinyCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
```
### Question 6
```
python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5
```
### Question 7
```
python pacman.py -l testSearch -p AStarFoodSearchAgent
python pacman.py -l trickySearch -p AStarFoodSearchAgent
```
### Question 8
```
python pacman.py -l bigSearch -p ClosestDotSearchAgent -z .5 
```
