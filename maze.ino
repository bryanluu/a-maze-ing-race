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

// Similar to F(), but for PROGMEM string pointers rather than literals
#define F2(progmem_ptr) (const __FlashStringHelper *)progmem_ptr
const char congrats[] PROGMEM = "You won, congratulations!!!"; // Congratulations text
#define CONGRATS_PAUSE_TIME 1000 // in ms
#define CONGRATS_SCROLL_DELAY 1 // in ms

// ########## MAIN CODE ##########

#define MATRIX_WIDTH 31
#define MATRIX_HEIGHT 31
#define MAZE_WIDTH ((MATRIX_WIDTH) - 1)/2
#define MAZE_HEIGHT ((MATRIX_HEIGHT) - 1)/2
#define MAZE_CAPACITY ((MAZE_WIDTH) * (MAZE_HEIGHT))
#define MATRIX(p) (2*(p) + 1) // conversion from maze coordinates to matrix coordinates
#define MATRIX_INTERPOLATE(p, q) ((p) + (q) + 1) // interpolate between maze coordinates in matrix space
#define VISIBILITY 1 // in pixels (1 means player can see up to 1 pixel away)
#define SHROUD true // control whether the unseen portions of the maze are shrouded

// colors
#define HUE(deg) ((long) (1536 * ((deg)/360.0))) // conversion to hue value from angle
#define SEEN_BRIGHTNESS 100
#define NEAR_BRIGHTNESS 255
#define BLACK (matrix.Color333(0, 0, 0))
#define WHITE(b) (matrix.ColorHSV(0, 0, b, true)) // white at a given brightness
#define COLOR 255
#define SHADE 0
#define H_RED (HUE(0)) // red hue value
#define H_GREEN (HUE(120)) // green hue value
#define H_BLUE (HUE(240)) // blue hue value
#define H_YELLOW (HUE(60)) // yellow hue value
#define SEEN_WALL_COLOR (matrix.ColorHSV(H_RED, 255, SEEN_BRIGHTNESS, true))
#define NEAR_WALL_COLOR (matrix.ColorHSV(H_RED, 255, NEAR_BRIGHTNESS, true))
#define SEEN_MAZE_COLOR BLACK
#define NEAR_MAZE_COLOR BLACK
#define SEEN_START_COLOR (matrix.ColorHSV(H_BLUE, 255, SEEN_BRIGHTNESS, true))
#define NEAR_START_COLOR (matrix.ColorHSV(H_BLUE, 255, NEAR_BRIGHTNESS, true))
#define SEEN_FINISH_COLOR (matrix.ColorHSV(H_GREEN, 255, SEEN_BRIGHTNESS, true))
#define NEAR_FINISH_COLOR (matrix.ColorHSV(H_GREEN, 255, NEAR_BRIGHTNESS, true))
#define SEEN_SOLUTION_COLOR (matrix.ColorHSV(H_YELLOW, 255, SEEN_BRIGHTNESS, true))
#define NEAR_SOLUTION_COLOR (matrix.ColorHSV(H_YELLOW, 255, NEAR_BRIGHTNESS, true))
#define PLAYER_COLOR (WHITE(NEAR_BRIGHTNESS))

enum Direction : int
{
  None = -1,
  Right,
  Down,
  Left,
  Up
};

// player parameters
#define RIGHT_CHAR 'R'
#define DOWN_CHAR 'D'
#define LEFT_CHAR 'L'
#define UP_CHAR 'U'

void buildMaze();
void calculateSolution();
void readInput();
void movePlayer();
void colorMaze();
void colorEndpoints();
void colorSolution();
void colorPlayer();
void displayMaze();
bool playerHasFinished();
void displayFinishScreen();

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setTextSize(1);
  buildMaze();
  calculateSolution();
}

Direction inputDir = None; // variable to hold direction input state
void loop() {
  // Clear background
  matrix.fillScreen(0);

  if (!playerHasFinished())
  {
    readInput();
    movePlayer();
    colorMaze();
    colorEndpoints();
    colorPlayer();
    displayMaze();
  }
  else
  {
    displayFinishScreen();
  }

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

uint16_t grid[MATRIX_HEIGHT][MATRIX_WIDTH]; // color of each pixel in matrix
bool seen[MATRIX_HEIGHT][MATRIX_WIDTH]; // which pixels the player has seen

/**
 * @brief Display the maze on the matrix
 * 
 */
void displayMaze()
{
  uint16_t color;
  for (byte r = 0; r < MATRIX_HEIGHT; r++)
  {
    for (byte c = 0; c < MATRIX_WIDTH; c++)
    {
      if (!SHROUD || seen[r][c] || grid[r][c] == SEEN_FINISH_COLOR)
        color = grid[r][c]; // show seen pixels
      else
        color = BLACK; // shroud maze sections that haven't been seen
      matrix.drawPixel(c, r, color);
    }
  }
}

/**
 * @brief Check whether given pixel is near player
 * 
 * @param x horizontal coord
 * @param y vertical coord
 * @return true if close to player
 * @return false otherwise
 */
bool isNearPlayer(byte x, byte y)
{
  return (abs(x - playerX) <= VISIBILITY) && (abs(y - playerY) <= VISIBILITY);
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
  grid[MATRIX(y)][MATRIX(x)] = SEEN_START_COLOR;
  x = GET_X(finish->pos);
  y = GET_Y(finish->pos);
  grid[MATRIX(y)][MATRIX(x)] = SEEN_FINISH_COLOR;
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
      grid[r][c] = SEEN_WALL_COLOR; // color wall
    }
  }
  byte x, y;
  for (byte p = 0; p < MAZE_CAPACITY; p++)
  {
    x = GET_X(p);
    y = GET_Y(p);
    byte r = MATRIX(y);
    byte c = MATRIX(x);
    grid[r][c] = SEEN_MAZE_COLOR; // color the vertex node

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
      grid[r][c] = SEEN_MAZE_COLOR;
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
    grid[r][c] = SEEN_SOLUTION_COLOR;
    if (v != start)
    { // Color the in-between step from last node position
      r = MATRIX_INTERPOLATE(y, ly);
      c = MATRIX_INTERPOLATE(x, lx);
      grid[r][c] = SEEN_SOLUTION_COLOR;
    }
    if (v == finish) // if we've reached the finish
      break; // stop loop
    
    v = &maze_g.vertices[v->id]; // move to predecessor
  }
}

/**
 * @brief Brighten pixels around player
 * 
 */
void brightenSurroundings()
{
  for (short x = playerX - VISIBILITY; x <= playerX + VISIBILITY; x++)
  {
    for (short y = playerY - VISIBILITY; y <= playerY + VISIBILITY; y++)
    {
      if (!isNearPlayer(x, y))
        continue;

      uint16_t color = grid[y][x];
      if (color == SEEN_WALL_COLOR)
        color = NEAR_WALL_COLOR;
      else if (color == SEEN_START_COLOR)
        color = NEAR_START_COLOR;
      else if (color == SEEN_FINISH_COLOR)
        color = NEAR_FINISH_COLOR;
      else if (color == SEEN_SOLUTION_COLOR)
        color = NEAR_SOLUTION_COLOR;
      grid[y][x] = color; // update color
      seen[y][x] = true; // mark pixel as seen
    }
  }
}

/**
 * @brief Color the player position
 * 
 */
void colorPlayer()
{
  grid[playerY][playerX] = PLAYER_COLOR;
  brightenSurroundings();
}

/**
 * @brief Reads the input
 */
void readInput()
{
  char c;
  inputDir = None;
  while (Serial.available())
  {
    c = Serial.read();
    switch (c)
    {
    case UP_CHAR:
      inputDir = Up;
      return;
    case LEFT_CHAR:
      inputDir = Left;
      return;
    case DOWN_CHAR:
      inputDir = Down;
      return;
    case RIGHT_CHAR:
      inputDir = Right;
      return;
    }
  }
}

/**
 * @brief Move the position of the player in the given direction
 * 
 */
void movePlayer()
{
  short dx, dy;
  dx = 0;
  dy = 0;
  byte x, y;
  switch (inputDir)
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
  if (grid[y][x] != NEAR_WALL_COLOR)
  {
    playerX = x;
    playerY = y;

    Serial.println(String(playerX) + "," + String(playerY));
  }
}

/**
 * @brief Checks if player has reached the end of the maze
 * 
 * @return true if player's position overlaps with the finish
 * @return false otherwise
 */
bool playerHasFinished()
{
  return playerX == MATRIX(GET_X(finish->pos)) && playerY == MATRIX(GET_Y(finish->pos));
}

int16_t textX = matrix.width();
int16_t textMin = (int16_t)sizeof(congrats) * -6;
int16_t hue = 0;
/**
 * @brief Displays the finishing graphics
 * 
 */
void displayFinishScreen()
{
  matrix.setTextColor(matrix.ColorHSV(hue, 255, 255, true));
  matrix.setCursor(textX, matrix.height()/2 - 4);
  matrix.print(congrats);
  textX--; // move text left
  if (textX < textMin)
  {
    delay(CONGRATS_PAUSE_TIME);
    textX = matrix.width();
  }
  hue += 7;
  if (hue >= 1536)
    hue = 0;
  delay(CONGRATS_SCROLL_DELAY);
}