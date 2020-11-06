/*
TODO:
50 - single agent navigation - Done
10 - 3d rendering and camera - render in 3D - Done, need to use camera - Done, use texturing and lighting - Done
10 - improved agent and scene rendering - texture the agent - Done, make all obstacles 3D - Done
10 - User scenario and editing+ - let the user change the set up scene only when it's paused - Done
10 - real time user interaction - can drag agent off the course - Done
10 - project report and video - Will do
*/

/*
plan of attack:
First work on Single Agent Navigation
-first generate a field of obstacles and random points - Done
-then find the shortest path through the obstacles from start to goal - Done
-return path of node numbers - Done
-set this as current path for agent - Done
-show agent moving from start to goal - Done
-prevent agent from collidig with objects - Done

The rest of the points will be easy to implement as long as I have
the single agent navigation working
*/

int numObstacles = 100;
int numNodes  = 100;
  
//A list of circle obstacles
static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii

Vec2 startPos = new Vec2(100,500);
Vec2 goalPos = new Vec2(500,200);
// add extra nodes to account for start and goal positions
int startNode = numNodes; 
int goalNode = numNodes + 1;

static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];
// keep track of g metric for PRM
float[] gscore = new float[maxNumNodes];

ArrayList<Integer> curPath;
int pathLen;
Vec2[] path; // Take returned integer indexes from curPath and create array of Vec2
int curPathIdx; //Which node in the path we are going towards, based on path array

//The tennis ball we are controlling
float agentRad = 16;
Vec2 agentPos;
Vec2 agentVel = new Vec2(0,0);
float goalSpeed = 100;

PShape agent; // sphere to texture with tennis ball image
PImage img; // tennis ball image
PImage bg; // background image

Camera camera;  // used previous camera library made by Liam Tyler for CSCI 5611

//////////////////////////////////
// functions for setup() 

void testPRM(){
  placeRandomObstacles(numObstacles);
  
  startPos = sampleFreePos();
  goalPos = sampleFreePos();
  nodePos[startNode] = startPos;
  nodePos[goalNode] = goalPos;

  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes + 2);  // add start and goal nodes
  
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes + 2);
}

Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos);
  while (insideAnyCircle){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos);
  }
  return randPos;
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (20+40*pow(random(1),3));
  }
  circleRad[0] = 30; //Make the first obstacle big
}

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
    }
    nodePos[i] = randPos;
  }
}

/////////////////////////////////////////
// setup

void setup(){
  size(1024,768, P3D);
  bg = loadImage("tennis_court_resized.jpg");
  img = loadImage("TennisBallColorMap.jpg");
  agent = createShape(SPHERE, agentRad);
  agent.setStroke(false);
  agent.setTexture(img);
  
  camera = new Camera();
  
  testPRM();
  // if we don't find a path, redo testPRM until we do
  while (curPath.get(0) == -1){
    testPRM();
  }
  pathLen = curPath.size();
  path = new Vec2 [pathLen];
  for (int i=0; i < pathLen; i++){
    path[i] = nodePos[curPath.get(i)];
  }
  curPathIdx = 0;
  agentPos = new Vec2 (nodePos[startNode].x, nodePos[startNode].y);
}

/////////////////////////////////////////////////////////
// functions for draw() 

Vec2 computeAgentVel(){
  Vec2 vel = path[curPathIdx].minus(agentPos);
  if (curPathIdx > pathLen-1) {  // if we ever get larger than path length
    return new Vec2(0,0);
  }
  else if (curPathIdx == pathLen -1) { // goal node is curPathIdx
    if (vel.length() < 1) return new Vec2(0,0);
    else {
      collisionInCircleList(circlePos, circleRad, numObstacles, agentPos, agentRad+1);
      vel.setToLength(goalSpeed);
      return vel;
    }
  }
  else {
    // check for collision in predicted path of agent
    // and change agentPos in that function
    collisionInCircleList(circlePos, circleRad, numObstacles, agentPos, agentRad+1);
    
    // if the agent is close to the node, set the current path Id to the next one
    if(vel.length() < agentRad) {
        curPathIdx++;
        vel = path[curPathIdx].minus(agentPos);
        vel.setToLength(goalSpeed);
        return vel;
    }
    // check if next node is visible
    Vec2 a_dir = path[curPathIdx+1].minus(agentPos);
    Vec2 check = a_dir.normalized();
    hitInfo circleListCheck = rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPos, check, a_dir.length());
    // if not, advance curPathIdx and recalculate velocity
    if(!circleListCheck.hit) {
      curPathIdx = curPathIdx++;  // set path Id to the next Id
      vel.setToLength(goalSpeed);
      return vel;
    }
    vel.setToLength(goalSpeed);
    return vel;
  }
}

void moveAgent(float dt){
  agentVel = computeAgentVel();
  agentPos.add(agentVel.times(dt));
}

void drawPRM() {
  //Draw the circle obstacles
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    pushMatrix();
    noStroke();
    translate(c.x,c.y);
    sphere(r);
    popMatrix();
  }
}

///////////////////////////////////////////////////////////////
// Draw
boolean paused = true;
void draw(){
  directionalLight(140, 140, 140, -1.5, -1, -1);
  ambientLight(140, 140, 140);
  strokeWeight(1);
  background(bg); // image is 2525 x 1285
  stroke(0,0,0);
  fill(255,255,255);

  // update camera
  // print(camera.position);
  camera.Update(1/frameRate);

  drawPRM();
  
  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
  
  //Draw black circles at nodes
  stroke(0,0,0);
  strokeWeight(3);
  fill(0,0,0);
  for (Vec2 p : path){
    circle(p.x,p.y,10);
  }
  
  //Draw edges between nodes
  for (int i = 0; i < pathLen-1; i++){
    line(path[i].x, path[i].y, path[i+1].x, path[i+1].y);
  }
  
  //Draw the agent
  fill(20,200,150);
  pushMatrix();
  noStroke();
  translate(agentPos.x, agentPos.y);
  shape(agent);
  popMatrix();
  
  if (exactPointer){
    fill(255,255,0, 255);
    arc(mouseX, mouseY, 30, 30, 0, 2*PI);
  }
}

/////////////////////////////////////
// Interactive Functions

boolean selected = false;
boolean dragObs = false;
int ObsInd;
boolean exactPointer = false;
boolean shiftDown = false;

void keyPressed(){
  if (key == ' ') paused = !paused;
  else if (key == 'r'){
    testPRM();
    // if we don't find a path, redo testPRM until we do
    while (curPath.get(0) == -1){
      testPRM();
    }
    pathLen = curPath.size();
    path = new Vec2 [pathLen];
    for (int i=0; i < pathLen; i++){
      path[i] = nodePos[curPath.get(i)];
    }
    curPathIdx = 0;
    agentPos = new Vec2 (nodePos[startNode].x, nodePos[startNode].y);
    paused = true;
  }
  else{
    camera.HandleKeyPressed();
  }

  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes + 2);
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes + 2);
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
  camera.HandleKeyReleased();
}

// Change bools depending on if we are clicking the agent or an obstacle
void mousePressed(){
  exactPointer = true;
  Vec2 mousePos = new Vec2(mouseX, mouseY);
  if (mousePos.distanceTo(agentPos) < agentRad){
    selected = true;
  }
  else if (paused) {
    selected = false;
    for (int i=0; i < numObstacles; i++){
      if (mousePos.distanceTo(circlePos[i]) < circleRad[i]) {
        dragObs = true;
        ObsInd = i;
      }
    }
  }
}

// Check if we are moving the agent or an obstacle
void mouseDragged(){
  if (selected){
    agentPos = new Vec2(mouseX, mouseY);
  }
  else if (dragObs) {
    circlePos[ObsInd] = new Vec2(mouseX, mouseY);
  }
}

// reset bools 
void mouseReleased(){
  selected = false;
  dragObs = false;
  exactPointer = false;
}
