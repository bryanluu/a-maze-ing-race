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

// colors
#define BLACK (matrix.Color333(0, 0, 0))
#define WHITE (matrix.Color333(7, 7, 7))
#define RED (matrix.Color333(7, 0, 0))
#define GREEN (matrix.Color333(0, 7, 0))
#define BLUE (matrix.Color333(0, 0, 7))
#define WALL_COLOR (matrix.Color333(1, 0, 0))
#define MAZE_COLOR BLACK
#define START_COLOR BLUE
#define FINISH_COLOR GREEN

void buildMaze();
void displayMaze();

void setup() {
  randomSeed(analogRead(0));
  matrix.begin();
  buildMaze();
}

void loop() {
  // Clear background
  matrix.fillScreen(0);

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
// directional indices
#define TOP 3
#define LEFT 2
#define BOTTOM 1
#define RIGHT 0
#define NONE (-1)

struct node {
  byte pos; // the position in the maze
  int edges[MAX_NEIGHBORS]; // neighboring edges of this node
  int cheapestEdgeCost = INT_MAX;
  byte cheapestEdge = NONE;
  bool used = false; // whether the node has been used in the maze

  node ()
  {
    for (byte i = 0; i < MAX_NEIGHBORS; i++)
      edges[i] = NONE;
  }

  bool operator==(const node &other) const
  {
    return (pos == other.pos);
  }

  /**
   * @brief Find relative position from other
   * 
   * @param other 
   * @return byte - a direction index
   */
  byte operator-(const node &other) const
  {
    short dx = GET_X(pos) - GET_X(other.pos);
    short dy = GET_Y(pos) - GET_Y(other.pos);
    if (dx == 0)
    {
      if (dy == 1)
        return BOTTOM;
      else if (dy == -1)
        return TOP;
    }
    else if (dy == 0)
    {
      if (dx == 1)
        return RIGHT;
      else if (dx == -1)
        return LEFT;
    }
    return NONE;
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
      case TOP:
        dy = -1;
        break;
      case BOTTOM:
        dy = 1;
        break;
      case LEFT:
        dx = -1;
        break;
      case RIGHT:
        dx = 1;
        break;
      default:
        return NONE;
    }
    byte x, y;
    x = GET_X(pos) + dx;
    y = GET_Y(pos) + dy;
    // if new position is invalid
    if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT)
      return NONE;

    return ENCODE(x, y);
  }
  
};

bool compare(node * u, node * v);

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
        adj_g.insertEdge(p, ENCODE(c, r-1), rand());
      // connect to left node
      if (c > 0)
        adj_g.insertEdge(p, ENCODE(c-1, r), rand());
    }
  }
}

/**
 * @brief Comparator used by priority queue to compare two nodes by their cheapest edge cost
 * 
 */
bool compare(node * u, node * v)
{
  return u->cheapestEdgeCost > v->cheapestEdgeCost;
}

node * start;
node * finish;
/**
 * @brief Set the end points of the maze object
 * 
 */
void setMazeEndpoints()
{
  // set endpoints
  start = &maze_g.vertices[0];
  finish = &maze_g.vertices[MAZE_CAPACITY - 1];
}

/**
 * @brief Builds the maze graph using Prim's algorithm
 * 
 */
void buildMaze() {
  // build the adjacency graph for the edge information
  buildAdjacencyGraph();

  // initialize the queue of vertices not in the maze
  std::priority_queue<node *, std::vector<node *>, decltype(&compare)> pq(&compare);
  for (byte p = 0; p < MAZE_CAPACITY; p++)
    pq.push(&adj_g.vertices[p]);

  setMazeEndpoints();

  while (!pq.empty()) // until the maze has all vertices
  {
    node * v = pq.top(); // take the vertex with the cheapest edge
    pq.pop();
    if (v->used) // if this vertex is already in the maze, skip it
      continue;

    v->used = true; // mark v as a used vertex in the maze
    node * u = &maze_g.vertices[v->pos]; // set v to maze vertex
    u->pos = v->pos; // add the vertex to the maze
    if (v->cheapestEdge != NONE) // if v touches the maze, add cheapest neighboring edge to maze
      maze_g.insertEdge(v->pos, v->pos_relative(v->cheapestEdge), v->cheapestEdgeCost);
    
    // loop through outgoing edges of vertex
    for (byte i = 0; i < MAX_NEIGHBORS; i++)
    {
      int e = v->edges[i]; // edge cost
      if (e == NONE) // if edge doesn't exist
        continue;

      byte n = v->pos_relative(i); // get neighbor index
      node * w = &adj_g.vertices[n]; // neighbor node
      if (!w->used && e < w->cheapestEdgeCost) // if a new neighboring cheapest edge is found
      {
        // update neighbor's cheapest edge
        w->cheapestEdgeCost = v->edges[i];
        w->cheapestEdge = ((*v) - (*w));
        pq.push(w);
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
      grid[r][c] = WALL_COLOR;
    }
  }
  byte x, y;
  for (byte p = 0; p < MAZE_CAPACITY; p++)
  {
    x = GET_X(p);
    y = GET_Y(p);
    byte r = 2*y + 1;
    byte c = 2*x + 1;
    grid[r][c] = MAZE_COLOR; // color the vertex node

    // color the edge nodes
    byte x2, y2;
    node * v = &maze_g.vertices[p];
    for (byte i = 0; i < MAX_NEIGHBORS; i++)
    {
      if (v->edges[i] == NONE)
        continue;

      node * u = &maze_g.vertices[v->pos_relative(i)];
      x2 = GET_X(u->pos);
      y2 = GET_Y(u->pos);
      r = y + y2 + 1;
      c = x + x2 + 1;
      grid[r][c] = MAZE_COLOR;
    }
  }

  // Color special nodes
  x = GET_X(start->pos);
  y = GET_Y(start->pos);
  grid[2*y + 1][2*x + 1] = START_COLOR;
  x = GET_X(finish->pos);
  y = GET_Y(finish->pos);
  grid[2*y + 1][2*x + 1] = FINISH_COLOR;

  for (byte r = 0; r < MATRIX_HEIGHT; r++)
  {
    for (byte c = 0; c < MATRIX_WIDTH; c++)
    {
      matrix.drawPixel(c, r, grid[r][c]);
    }
  }
}
