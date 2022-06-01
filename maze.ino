// Custom Maze game on Adafruit's 32x32 RGB LED matrix:
// http://www.adafruit.com/products/607
// 32x32 MATRICES DO NOT WORK WITH ARDUINO UNO or METRO 328.

// Written by Bryan Luu.
// BSD license, all text above must be included in any redistribution.

#include <RGBmatrixPanel.h>
#include <limits.h>
#include <memory>
#include <vector>
#include <queue>

// Most of the signal pins are configurable, but the CLK pin has some
// special constraints.  On 8-bit AVR boards it must be on PORTB...
// Pin 11 works on the Arduino Mega.  On 32-bit SAMD boards it must be
// on the same PORT as the RGB data pins (D2-D7)...
// Pin 8 works on the Adafruit Metro M0 or Arduino Zero,
// Pin A4 works on the Adafruit Metro M4 (if using the Adafruit RGB
// Matrix Shield, cut trace between CLK pads and run a wire to A4).

#define CLK  8   // USE THIS ON ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE   9
#define LAT 10
#define A   A0
#define B   A1
#define C   A2
#define D   A3

RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, true);

// ########## MAIN CODE ##########

#define MATRIX_WIDTH 31
#define MATRIX_HEIGHT 31
#define MAZE_WIDTH ((MATRIX_WIDTH) - 1)/2
#define MAZE_HEIGHT ((MATRIX_HEIGHT) - 1)/2
#define MAZE_CAPACITY ((MAZE_WIDTH) * (MAZE_HEIGHT))
#define MATRIX(p) (2*(p) + 1) // conversion from maze coordinates to matrix coordinates
#define MATRIX_INTERPOLATE(p, q) ((p) + (q) + 1) // interpolate between maze coordinates in matrix space

// colors
#define B_UNIT 1 // brightness out of 7
#define BLACK (matrix.Color333(0, 0, 0))
#define WHITE (matrix.Color333(B_UNIT, B_UNIT, B_UNIT))
#define RED (matrix.Color333(B_UNIT, 0, 0))
#define GREEN (matrix.Color333(0, B_UNIT, 0))
#define BLUE (matrix.Color333(0, 0, B_UNIT))
#define YELLOW (matrix.Color333(B_UNIT, B_UNIT, 0))
#define WALL_COLOR RED
#define MAZE_COLOR BLACK
#define START_COLOR BLUE
#define FINISH_COLOR GREEN
#define SOLUTION_COLOR YELLOW
#define PLAYER_COLOR WHITE

enum Direction : int
{
  Up,
  Left,
  Down,
  Right,
  None = -1
};

// player parameters
#define UP_CHAR 'U'
#define LEFT_CHAR 'L'
#define DOWN_CHAR 'D'
#define RIGHT_CHAR 'R'

void buildMaze();
void calculateSolution();
Direction readInput();
void movePlayer(Direction dir);
void colorMaze();
void colorEndpoints();
void colorSolution();
void colorPlayer();
void displayMaze();

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));
  matrix.begin();
  buildMaze();
  calculateSolution();
}

Direction inputDir = None;
void loop() {
  // Clear background
  matrix.fillScreen(0);

  inputDir = readInput();
  movePlayer(inputDir);
  colorMaze();
  colorEndpoints();
  colorPlayer();
  displayMaze();
  delay(100);

#if !defined(__AVR__)
  // On non-AVR boards, delay slightly so screen updates aren't too quick.
  delay(20);
#endif

  // Update display
  matrix.swapBuffers(false);
}

// ########## GRAPH CODE ##########

// encodes the position given x and y
#define ENCODE(x, y) ((x) + (MAZE_WIDTH)*(y))
// decodes the x value of position
#define GET_X(p) ((p) % (MAZE_WIDTH))
// decodes the y value of position
#define GET_Y(p) ((p) / (MAZE_WIDTH))
// max number of neighbors this node can have
#define MAX_NEIGHBORS 4
// number of edges
#define N_EDGES (MAZE_CAPACITY - 1)

struct node {
  byte pos = None; // the position in the maze
  int edges[MAX_NEIGHBORS]; // neighboring edges of this node
  int value = INT_MAX; // integer value to keep track of (cheapestEdgeWeight or distance)
  byte id = None; // id of edge or node to keep track of
  bool used = false; // whether the node has been used in the maze

  node ()
  {
    for (byte i = 0; i < MAX_NEIGHBORS; i++)
      edges[i] = None;
  }

  bool operator==(const node &other) const
  {
    return (pos == other.pos);
  }

  /**
   * @brief Find relative position from other
   * 
   * @param other 
   * @return Direction - a direction index
   */
  Direction operator-(const node &other) const
  {
    short dx = GET_X(pos) - GET_X(other.pos);
    short dy = GET_Y(pos) - GET_Y(other.pos);
    if (dx == 0)
    {
      if (dy == 1)
        return Down;
      else if (dy == -1)
        return Up;
    }
    else if (dy == 0)
    {
      if (dx == 1)
        return Right;
      else if (dx == -1)
        return Left;
    }
    return None;
  }

  /**
   * @brief Find the position by relative direction
   * 
   * @param dir 
   * @return byte - the position in the relative direction
   */
  byte pos_relative(char dir)
  {
    short dx, dy;
    dx = 0;
    dy = 0;
    switch (dir)
    {
      case Up:
        dy = -1;
        break;
      case Down:
        dy = 1;
        break;
      case Left:
        dx = -1;
        break;
      case Right:
        dx = 1;
        break;
      default:
        return None;
    }
    byte x, y;
    x = GET_X(pos) + dx;
    y = GET_Y(pos) + dy;
    // if new position is invalid
    if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT)
      return None;

    return ENCODE(x, y);
  }
  
};

bool compare(node * u, node * v);

typedef std::vector<node *> vertex_list;

namespace std {
  template<>
  struct hash<node>
  {
    std::size_t operator()(const node& p) const
    {
      return p.pos;
    }
  };
}

struct graph {
  // hold graph vertices
  node vertices[MAZE_CAPACITY];

  /**
   * Insert a new directed edge with a positive edge weight into the graph.
   * 
   * @param s the encoded position of the source vertex for the edge
   * @param t the encoded position of the target vertex for the edge
   * @param weight the weight for the edge (has to be a positive integer)
   * @return true if the edge could be inserted or its weight updated, false if the edge with the
   *         same weight was already in the graph
   */
  bool insertEdge(byte s, byte t, int weight)
  {
    // source or target vertices invalid
    if (s >= (MAZE_CAPACITY) || t >= (MAZE_CAPACITY))
      return false;

    // invalid negative weight
    if (weight < 0)
      return false;

    node * source = &vertices[s];
    node * target = &vertices[t];
    source->edges[(*target) - (*source)] = weight;
    target->edges[(*source) - (*target)] = weight;
    return true;
  }
};

graph adj_g; // adjacency graph of maze
graph maze_g; // graph of the maze

/**
 * @brief Builds the adjacency graph for the maze
 * 
 */
void buildAdjacencyGraph() {
  for (byte r = 0; r < MAZE_HEIGHT; r++)
  {
    for (byte c = 0; c < MAZE_WIDTH; c++)
    {
      byte p = ENCODE(c, r);
      node * v = &adj_g.vertices[p];
      v->pos = p; // set the pos of the node

      // connect to top node
      if (r > 0)
        adj_g.insertEdge(p, ENCODE(c, r-1), random(N_EDGES));
      // connect to left node
      if (c > 0)
        adj_g.insertEdge(p, ENCODE(c-1, r), random(N_EDGES));
    }
  }
}

/**
 * @brief Comparator used by priority queue to compare two nodes by their value
 * 
 * @param u node 1
 * @param v node 2
 * @return true if node 1's value is greater than node 2's
 * @return false otherwise
 */
bool compare(node * u, node * v)
{
  return u->value > v->value;
}

node * start;
node * finish;
byte playerX, playerY;
/**
 * @brief Set the end points of the maze object
 * 
 */
void setMazeEndpoints()
{
  // set endpoints
  start = &maze_g.vertices[0];
  finish = &maze_g.vertices[MAZE_CAPACITY - 1];
  playerX = MATRIX(GET_X(start->pos));
  playerY = MATRIX(GET_Y(start->pos));
}

/**
 * @brief Builds the maze graph using Prim's algorithm
 * 
 */
void buildMaze() {
  // build the adjacency graph for the edge information
  buildAdjacencyGraph();

  // initialize the queue of vertices not in the maze
  std::priority_queue<node *, vertex_list, decltype(&compare)> pq(&compare);
  for (byte p = 0; p < MAZE_CAPACITY; p++)
    pq.push(&adj_g.vertices[p]);

  while (!pq.empty()) // until the maze has all vertices
  {
    node * v = pq.top(); // take the vertex with the cheapest edge
    pq.pop();
    if (v->used) // if this vertex is already in the maze, skip it
      continue;

    v->used = true; // mark v as a used vertex in the maze
    node * u = &maze_g.vertices[v->pos]; // set v to maze vertex
    u->pos = v->pos; // add the vertex to the maze
    if (v->id != None) // if v touches the maze, add cheapest neighboring edge to maze
      maze_g.insertEdge(v->pos, v->pos_relative(v->id), v->value);
    
    // loop through outgoing edges of vertex
    for (byte i = 0; i < MAX_NEIGHBORS; i++)
    {
      int e = v->edges[i]; // edge cost
      if (e == None) // if edge doesn't exist
        continue;

      byte n = v->pos_relative(i); // get neighbor index
      node * w = &adj_g.vertices[n]; // neighbor node
      if (!w->used && e < w->value) // if a new neighboring cheapest edge is found
      {
        // update neighbor's cheapest edge
        w->value = v->edges[i];
        w->id = ((*v) - (*w));
        pq.push(w);
      }
    }
  }

  setMazeEndpoints();
}

/**
 * @brief Calculates the shortest path from start to finish using Dijkstra's algorithm
 * 
 */
void calculateSolution()
{
  // set finish distance to 0
  finish->value = 0;
  finish->used = true; // mark the finish as visited

  // initialize queue of vertices
  std::priority_queue<node*, vertex_list, decltype(&compare)> pq(&compare);
  pq.push(finish);

  while (!pq.empty()) // until the queue is empty
  {
    node * u = pq.top(); // u is best vertex
    pq.pop();
    if (u == start)
      break; // we've reached the end of the maze!

    // loop through neighbors
    for (byte i =  0; i < MAX_NEIGHBORS; i++)
    {
      int e = u->edges[i]; // edge cost
      if (e == None) // if edge doesn't exist
        continue;

      byte n = u->pos_relative(i); // get neighbor index
      node * v = &maze_g.vertices[n]; // neighbor node
      int alt = u->value + e;
      if (!v->used && alt < v->value)
      {
        v->value = alt;
        v->id = u->pos;
        v->used = true; // mark v as visited
        pq.push(v);
      }
    }
  }
}

uint16_t grid[MATRIX_WIDTH][MATRIX_HEIGHT]; // color of each pixel in matrix

/**
 * @brief Display the maze on the matrix
 * 
 */
void displayMaze() {
  for (byte r = 0; r < MATRIX_HEIGHT; r++)
  {
    for (byte c = 0; c < MATRIX_WIDTH; c++)
    {
      matrix.drawPixel(c, r, grid[r][c]);
    }
  }
}

/**
 * @brief Colors the endpoints of the maze
 * 
 */
void colorEndpoints()
{
  // Color special nodes
  byte x, y;
  x = GET_X(start->pos);
  y = GET_Y(start->pos);
  grid[MATRIX(y)][MATRIX(x)] = START_COLOR;
  x = GET_X(finish->pos);
  y = GET_Y(finish->pos);
  grid[MATRIX(y)][MATRIX(x)] = FINISH_COLOR;
}

/**
 * @brief Color the maze and walls
 * 
 */
void colorMaze()
{
  for (byte r = 0; r < MATRIX_HEIGHT; r++)
  {
    for (byte c = 0; c < MATRIX_WIDTH; c++)
    {
      grid[r][c] = WALL_COLOR;
    }
  }
  byte x, y;
  for (byte p = 0; p < MAZE_CAPACITY; p++)
  {
    x = GET_X(p);
    y = GET_Y(p);
    byte r = MATRIX(y);
    byte c = MATRIX(x);
    grid[r][c] = MAZE_COLOR; // color the vertex node

    // color the edge nodes
    byte x2, y2;
    node * v = &maze_g.vertices[p];
    for (byte i = 0; i < MAX_NEIGHBORS; i++)
    {
      if (v->edges[i] == None) // if edge doesn't exist
        continue;

      node * u = &maze_g.vertices[v->pos_relative(i)];
      x2 = GET_X(u->pos);
      y2 = GET_Y(u->pos);
      r = MATRIX_INTERPOLATE(y, y2);
      c = MATRIX_INTERPOLATE(x, x2);
      grid[r][c] = MAZE_COLOR;
    }
  }
}

/**
 * @brief Color the shortest path from start to finish
 * 
 */
void colorSolution()
{
  node * v = start;
  byte x, y;
  byte lx, ly; // last node pos
  byte r, c;
  while (true) // loop until we reach the finish
  {
    // Update last node position
    lx = x;
    ly = y;
    // Update current node position
    x = GET_X(v->pos);
    y = GET_Y(v->pos);
    // Color current node position
    r = MATRIX(y);
    c = MATRIX(x);
    grid[r][c] = SOLUTION_COLOR;
    if (v != start)
    { // Color the in-between step from last node position
      r = MATRIX_INTERPOLATE(y, ly);
      c = MATRIX_INTERPOLATE(x, lx);
      grid[r][c] = SOLUTION_COLOR;
    }
    if (v == finish) // if we've reached the finish
      break; // stop loop
    
    v = &maze_g.vertices[v->id]; // move to predecessor
  }
}

/**
 * @brief Color the player position
 * 
 */
void colorPlayer()
{
  grid[playerY][playerX] = PLAYER_COLOR;
}

/**
 * @brief Reads the input to direct where to move the player
 * 
 * @return Direction - the input direction to move
 */
Direction readInput()
{
  char c;
  while (Serial.available())
  {
    c = Serial.read();
    switch (c)
    {
    case UP_CHAR:
      return Up;
    case LEFT_CHAR:
      return Left;
    case DOWN_CHAR:
      return Down;
    case RIGHT_CHAR:
      return Right;
      break;
    }
  }
  return None;
}

/**
 * @brief Move the position of the player in the given direction
 * 
 * @param dir 
 */
void movePlayer(Direction dir)
{
  short dx, dy;
  dx = 0;
  dy = 0;
  byte x, y;
  switch (dir)
  {
  case Up:
    dy = -1;
    break;
  case Down:
    dy = 1;
    break;
  case Left:
    dx = -1;
    break;
  case Right:
    dx = 1;
    break;
  default:
    return;
  }
  x = constrain(playerX + dx, 0, MATRIX_WIDTH);
  y = constrain(playerY + dy, 0, MATRIX_HEIGHT);
  if (grid[y][x] != WALL_COLOR)
  {
    playerX = x;
    playerY = y;
    Serial.println(String(playerX) + "," + String(playerY));
  }
}