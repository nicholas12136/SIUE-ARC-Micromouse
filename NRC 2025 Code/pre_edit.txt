#include <Arduino.h>
#include <Streaming.h>
#include <Vector.h>

//node class
class Node {
public:
  char* name;
  int cell[2];          // x, y grid position
  bool walls[4];        // [0]=N, [1]=E, [2]=S, [3]=W
  bool exits[4];        // which directions we've actually traveled
  bool exitedAll;       // all exits previously taken
  bool isHall;          // cell is a 'hallway' configuration. two open walls opposite each other
  bool isTurn;          // cell is a turn. two open walls adjacent to each other
  bool isIntersection;  // cell is an intersection. more than two open walls
  bool isDeadEnd;       // cell is a dead-end. only one open wall
  uint8_t exit;         // bitmask for traveled directions. four LSBs [0]=N, [1]=E, [2]=S, [3]=W. 0 is not traveled yet. 1 is an exit already taken. 0bXXXX0000-0bXXXX1111. four MSBs are dont-cares.
  uint8_t wall;         // bitmask for walls. four LSBs [0]=N...etc.
  Node* nextNode[4];    // next node in direction. hallways ignored for defining adjacent nodes.
  int nextCoord[4][2];  // adjacenct cell coords (x,y)

  Node() {
    name = nullptr;
    cell[0] = 0;
    cell[1] = 0;
    for (int i = 0; i < 4; i++) {
      walls[i] = false;
      exits[i] = false;
      nextNode[i] = nullptr;
      //nextCoord[i][0] = -1;
      //nextCoord[i][1] = -1;
    }
    exitedAll = false;
    isHall = false;
    isTurn = false;
    isIntersection = false;
    isDeadEnd = false;
    exit = 0;
    wall = 0;
  }
};

int facing = 0;     //keeps track of facing direction
int startFlag = 1;  //for correctly inititializing first node in maze

Node* getOrCreateNode(int x, int y);  //function declarations
void setWallsForNode(Node* n, int facingDir);
int checkNodeWalls(Node* n);
void setType(Node* n);
void exitCheck(Node* n);
void setExit(Node* n, int direction);
bool nodeHasExits(Node* n);
int findOpenExit(Node* n);

// Movement & turning
void forwardStep();
void leftTurn();
void rightTurn();
void turnTo(int dir);


static const int MAZE_SIZE = 16;
Node* nodeGrid[MAZE_SIZE][MAZE_SIZE] = { nullptr };  //initialize an instance of class Node for each cell in the maze as nullpointer

int positionXY[2] = { 0, 0 };  // current x,y
bool start = true;             // example "start" marker


static const int STACK_CAPACITY = 256;   //stack capacity
Node* nodeStackStorage[STACK_CAPACITY];  //
typedef Vector<Node*> NodeVector;        //
NodeVector nodeStack(nodeStackStorage);  //

Node* last_node = nullptr;  // optional pointer to parent node

//function to return node or create new node if it does not exist yet. arguments are x and y coordinates of the node.
Node* getOrCreateNode(int x, int y) {
  if (nodeGrid[x][y] != nullptr) {
    return nodeGrid[x][y];
  }
  Node* newNode = new Node();
  newNode->cell[0] = x;
  newNode->cell[1] = y;
  nodeGrid[x][y] = newNode;
  return newNode;
}

// Mark the node's walls from the robot's perspective
void setWallsForNode(Node* n, int facingDir) {

  log("setting walls");
  bool wf = wallFront();
  log("front wall");
  n->walls[facingDir] = wf;
  bitWrite(n->wall, facingDir, wf);

  // back (often deduce from adjacency, but let's set false for now)
  n->walls[(facingDir + 2) % 4] = false;
  log("back wall");
  bitWrite(n->wall, (facingDir + 2) % 4, false);

  // right
  bool wr = wallRight();
  log("right wall");
  n->walls[(facingDir + 1) % 4] = wr;
  bitWrite(n->wall, (facingDir + 1) % 4, wr);

  // left
  bool wl = wallLeft();
  log("wall left");
  n->walls[(facingDir + 3) % 4] = wl;
  bitWrite(n->wall, (facingDir + 3) % 4, wl);
}

// Return how many sides are walled
int checkNodeWalls(Node* n) {
  int count = 0;
  for (int i = 0; i < 4; i++) {
    if (n->walls[i]) count++;
  }
  return count;
}

// Determine if it is a dead-end, hall, turn, intersection, etc.
void setType(Node* n) {
  int wCount = checkNodeWalls(n);
  if (wCount == 3) {
    n->isDeadEnd = true;
  } else if (wCount < 2) {
    // 0 or 1 => intersection
    n->isIntersection = true;
  } else if (wCount == 2) {
    // check if the 2 open sides are opposite or adjacent
    bool open[4];
    for (int i = 0; i < 4; i++) {
      open[i] = !n->walls[i];
    }
    if ((open[0] && open[2]) || (open[1] && open[3])) {
      n->isHall = true;
    } else {
      n->isTurn = true;
    }
  }
}

// If (n->exit | n->wall) == 0b1111 => no untraveled directions remain
void exitCheck(Node* n) {
  if ((n->exit | n->wall) == 0x0F) {
    n->exitedAll = true;
  } else {
    n->exitedAll = false;
  }
}

// Mark a direction as traveled
void setExit(Node* n, int direction) {
  n->exits[direction] = true;
  bitWrite(n->exit, direction, 1);
  exitCheck(n);
}

// Check if node has at least one untraveled, unwalled direction
bool nodeHasExits(Node* n) {
  for (int i = 0; i < 4; i++) {
    if (!n->walls[i] && !n->exits[i]) {
      return true;
    }
  }
  return false;
}

// Return the first open direction or -1 if none
int findOpenExit(Node* n) {
  for (int i = 0; i < 4; i++) {
    if (n->walls[i] == false && n->exits[i] == 0) {
      log("open exit: " + String(i));
      return i;
    }
  }
  n->exitedAll = true;
  return -1;
}

int findNeighborExit(Node* n) {
  for (int i = 0; i < 4; i++) {
    if (n->nextNode[i] != nullptr && nodeHasExits(n->nextNode[i]) == true) {
      return i;
    }
  }
  return -1;
}



// -------------------------------------------------------------------
// Movement helpers
// -------------------------------------------------------------------
//facing is always 0-3.
void rightTurn() {
  log("Right turn.");
  turnRight();
  facing = (facing + 1) % 4;
  log("Facing: " + String(facing));
}

void leftTurn() {
  log("Left turn.");
  turnLeft();
  facing = (facing - 1) % 4;
  log("Facing: " + String(facing));
}

void turnAround() {
  log("Turning around.");
  rightTurn();
  rightTurn();
  log("Facing: " + String(facing));
}


void forwardStep() {
  moveForward();
  if (facing == 0) {
    positionXY[1] = positionXY[1] + 1;
  } else if (facing == 1) {
    positionXY[0] = positionXY[0] + 1;
  } else if (facing == 2) {
    positionXY[1] = positionXY[1] - 1;
  } else if (facing == 3) {
    positionXY[0] = positionXY[0] - 1;
  }
}

// Turn to a desired direction (0..3) by repeatedly turning right
void turnTo(int dir) {
  while (facing != dir) {
    rightTurn();
  }
}

//current cell, next cell
void goToNode(Node* n, Node* l) {

  log("go to: " + String(n->cell[0]) + String(n->cell[1]));
  int dx = n->cell[0] - l->cell[1];
  int dy = n->cell[1] - l->cell[1];
  if (dx > 0) {
    turnTo(3);
  } else if (dx < 0) {
    turnTo(1);
  } else if (dy > 0) {
    turnTo(1);
  } else if (dy < 0) {
    turnTo(0);
  }
  setExit(n, facing);
  while (positionXY[0] != l->cell[0] && positionXY[1] != l->cell[1]) {
    forwardStep();
  }
}

void findAdjacentNodes(Node* n){
  int x = n->cell[0];
  int y = n->cell[1];
  while(n->exitedAll == false){
  int d = findOpenExit(n);
  if (d != -1) {
    // Turn physically to d
    turnTo(d);
    // Mark that exit
    setExit(n, d);
    // Move forward

    forwardStep();
    // The new cell => get/create node
    Node* nextN = getOrCreateNode(positionXY[0], positionXY[1]);
  while(nextN->isHall == true){
    nodeStack.push_back(nextN);
    forwardStep();
    Node* nextN = getOrCreateNode(positionXY[0], positionXY[1]);
  }
  if(nextN->isTurn == true || nextN->isIntersection == true){
    n->nextNode[facing] = nextN;
    nextN->nextNode[(facing+2)%4]= n;
  }else if(nextN->isDeadEnd==true){
    
  }
  nodeStack.push_back(nextN);
  goToNode(nextN, n);
  }
}
}

// -------------------------------------------------------------------
// 5) SETUP AND LOOP
// -------------------------------------------------------------------
void setup() {
  Serial.begin(19200);

  nodeStack.clear();  // Clear the stack
}

void loop() {
  // If stack empty, push the start node
  if (nodeStack.size() == 0) {
    Node* startNode = getOrCreateNode(positionXY[0], positionXY[1]);
    nodeStack.push_back(startNode);
    log("Pushed start node onto stack");
  }

  Node* current = nodeStack[nodeStack.size() - 1];
  log("stack size: " + String(nodeStack.size()));
  // Update walls from the robot's current facing
  setWallsForNode(current, facing);
  // Classify
  if (positionXY[0] == 0 && positionXY[1] == 0 && startFlag == 1) {
    for (int i = 0; i < 4; i++) {
      if (current->walls[i] == 0) {
        current->walls[(i + 2) % 4] = 1;
      }
      startFlag = 0;
    }
  }
  setType(current);
  // Mark if fully explored
  exitCheck(current);

  // Debug
  log("Current node: " + String(current->cell[0]) + "," + String(current->cell[1]) + "facing: " + String(facing));
  log("Walls: N=" + String(current->walls[0]) + " E=" + String(current->walls[1]) + " S=" + String(current->walls[2]) + " W=" + String(current->walls[3]));
  log("Exits used: N=" + String(current->exits[0]) + " E=" + String(current->exits[1]) + " S=" + String(current->exits[2]) + " W=" + String(current->exits[3]));
  log("exitedAll=" + String(current->exitedAll));

  // 1) Try to find an open exit
  for(int i=0; i<4; i++){
    if(current->nextNode[i]->isIntersection == true && current->exitedAll == false){
      findAdjacentNodes(current);
    }
  }
  int d = findOpenExit(current);
  if (d != -1) {
    // Turn physically to d
    turnTo(d);
    // Mark that exit
    setExit(current, d);
    // Move forward
    forwardStep();
    // The new cell => get/create node
    Node* nextN = getOrCreateNode(positionXY[0], positionXY[1]);
    
  if(nextN->isIntersection){
    findAdjacentNodes(nextN);
    }else{// Link adjacency in both directions
    current->nextNode[d] = nextN;
    //nextN->nextNode[(d + 2) % 4] = current;
    // Push to stack
    nodeStack.push_back(nextN);
    //last_node = current;
    }
    return; // Let next iteration handle the new cell
  }

  // 2) No open exits => node is fully explored. Backtrack.
  current->exitedAll = true;

  d = findNeighborExit(current);
  if (d != -1) {
    // Turn physically to d
    turnTo(d);
    // Mark that exit
    setExit(current, d);

    goToNode(current, current->nextNode[d]);

    // Move forward

    // The new cell => get/create node
    Node* nextN = getOrCreateNode(positionXY[0], positionXY[1]);
    // Link adjacency in both directions

    nextN->nextNode[(d + 2) % 4] = last_node;
    // Push to stack
    nodeStack.push_back(nextN);
    if (nextN->isIntersection == true || nextN->isTurn == true) {
      last_node->nextNode[facing] = nextN;
      last_node = current;
    }

    return;  // Let next iteration handle the new cell
  }

  if (nodeStack.size() == 1) {
    // This means we are at the start node with no open exits => done

    while (true) { /* done */
    }
  }

  // Pop the current node
  nodeStack.pop_back();
  Node* parentNode = nodeStack[nodeStack.size() - 1];

  // Turn around, move one step to physically go back
  turnAround();
  // or do a function turnAround() if you prefer


  forwardStep();

  // Next iteration sees the parentNode as the top of stack
}
