ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

ArrayList<Integer> planPath(
  Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  int startID = numNodes - 2; 
  int goalID = numNodes - 1;
  nodePos[startID] = startPos;
  nodePos[goalID] = goalPos;
  path = runAStar(nodePos, numNodes + 2, startID, goalID);
  return path;
}

float g(int pointID) {
  int prevNode = parent[pointID];
  float pathLen = 0;
  while (prevNode >= 0){
    float g = nodePos[pointID].distanceTo(nodePos[prevNode]);
    pathLen = pathLen + g;
    prevNode = parent[prevNode];
  }
  return pathLen;
}

float h(int point) {
  float h = nodePos[point].distanceTo(goalPos);
  return h;
}

ArrayList<Integer> runAStar(Vec2[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe //<>//
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
    gscore[i] = 0;
  }
  
  visited[startID] = true;
  fringe.add(startID);
  
  int fringeIndex = -1;
  while (fringe.size() > 0) {
    // find node in fringe with the lowest score
    float cost = 9999999;
    for (int i=0; i < fringe.size(); i++) {
      int nodeID = fringe.get(i);
      float tempScore = gscore[nodeID] + h(nodeID);
      if (tempScore <= cost) {
        cost = tempScore;
        fringeIndex = i;
      }
    }
    
    // remove node from fringe list and change to visited
    int currentNode = fringe.get(fringeIndex);
    fringe.remove(fringeIndex);
    visited[currentNode] = true;
    
    // if the currentNode is goalID, then we are at the end
    if (currentNode == goalID){
      break;
    }

    // for each neighbor of this node
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]) {  // if it hasn't been visited. use it
        float tempGScore = g(currentNode) + nodePos[neighborNode].distanceTo(nodePos[currentNode]);
        boolean betterPath = false;
        if (fringe.contains(neighborNode)) {// if it is in the fringe list
          if (tempGScore < gscore[neighborNode]) {
            gscore[neighborNode] = tempGScore;
            betterPath = true;
          }
        } else {  
          gscore[neighborNode] = tempGScore;
          fringe.add(neighborNode);
          betterPath = true;
        }
        if (betterPath) {
          parent[neighborNode] = currentNode;
        }
      }
    }
  }
    
  if (fringe.size() == 0){
    println("No Path");
    path.add(0,-1);
    return path;
  }
  
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  return path;
}
