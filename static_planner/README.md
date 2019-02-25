In this project I implemented a static planner that navigates the vehicle down multiple lane with static obstacles.

The project can be decomposed into the following parts:

1. Motion primitives. Three motion primitives were developed, namely, changing lane to the left, changing lane to the right and proceeding straight. The former two 
were constructed using Bezier curve and the script is placed under the directory picutres. Each motion primitve contains the following information: start state, end 
state, length, cost and intermediate points. 

2. Graph configuration. I wrote four callback functions that subscribed to start, pose, map and obstacle topics respectively. The map is discretized into stations
with a length of 10 meters and a width of 3.7 meters and stored as a 2D boolean vector. Obstacle information is integrated into the map by marking those stations as 
occupied. 

3. Planner function. I used A*/Weighted A* algorithm for planning because the number of states that need to be expanded will be limited with the map configuration and my design 
of motion primitives. The planner function will be called after all information necessary for planning is collected and the corresponding flags were set. Each state 
is a node including position, f value, g value and motion id by which is node is generated. The heuristic function used manhattan distance with penalization on lane 
changing action. 

4. Collision checking. A safety margin of 1.8 meters is applied to avoid marginal collision. If the distance between centers of the ego and the obstacle is smaller 
than the sum of outer radius, a double check will be done to confirm there will be no collision along the current motion primitive. The generate successors function 
returns a vector of nodes that can be reached from current state without collision.

5. Publish trajectory. After planning is done, the path is constructed recursively from the goal position and published to planner trajectory topic. 