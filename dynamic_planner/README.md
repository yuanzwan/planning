In this project I implemented a dynamic planner that navigates the vehicle down multiple lane with dynamic obstacles.

The project is similar to the static one except for the following parts:

1. Graph configuration. Since obstacles are dynamic, we cannot integrate them into the map by simply marking their positions. If we add a time 
dimension to the map variable, it would require lots of memory space, which is limited in embedded systems . Therefore, the map is still stored 
as a 2D boolean vector while the the obstacle posisitons are stored separately.

2. Planner function. A state of time is added to the node structure. A* algorithm is used as a starting point for planning and it managed to 
finish planning in ~100ms. The planner function will be called after all callback flags were set. The heuristic function used manhattan distance 
with penalization on lane changing actions. The comparator for the open list now prioritized nodes with less time in case of identical positions. 

3. Collision checking. A safety margin of 1.8 meters was applied to avoid marginal collision. For each point on the motion primitive, the positions
of obstacles were interpolated to determine if there is any intersection of the bounding boxes, in other words, collsion. The generate successors 
function returns a vector of nodes that can be reached from current state without collision.