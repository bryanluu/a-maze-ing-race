// Custom Maze game on Adafruit's 32x32 RGB LED matrix:
// http://www.adafruit.com/products/607
// 32x32 MATRICES DO NOT WORK WITH ARDUINO UNO or METRO 328.

// Written by Bryan Luu.
// BSD license, all text above must be included in any redistribution.

#include <RGBmatrixPanel.h>
#include <limits.h>
#include <unordered_map>
#include <memory>

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

#define MATRIX_WIDTH 32
#define MATRIX_HEIGHT 32
#define MAZE_WIDTH (MATRIX_WIDTH - 1)/2
#define MAZE_HEIGHT (MATRIX_HEIGHT - 1)/2

void buildMaze();

void setup() {
  randomSeed(analogRead(0));
  matrix.begin();
  buildMaze();
}

void loop() {
  // Clear background
  matrix.fillScreen(0);

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

struct node;
struct edge;

struct node {
  byte pos; // the position in the maze
  edge * edgesLeaving[MAX_NEIGHBORS] = {}; // edges leaving this vertex
  int cheapestEdgeCost = INT_MAX;
  edge * cheapestEdge = NULL;
  unsigned char n_edges = 0;

  bool operator==(const node &other) const
  {
    return (pos == other.pos);
  }
  
};

struct edge {
  node * source;
  node * target;
  int weight;

  edge (node * src, node * tgt, int wt)
  {
    source = src;
    target = tgt;
    weight = wt;
  }
};

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
  node vertices[MAZE_WIDTH * MAZE_HEIGHT];

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

    // handle cases where edge already exists between these points
    bool alreadyExists = false;
    node * source = &vertices[s];
    node * target = &vertices[t];
    edge * e;
    for (byte i = 0; i < MAX_NEIGHBORS; i++)
    {
      e = source->edgesLeaving[i];
      // if an edge to target is found from source
      if (e->target == target)
      {
        if (e->weight == weight)
          return false; // edge already exists
        else
          e->weight = weight; // otherwise update weight of existing edge
        alreadyExists = true;
        break;
      }
      e = target->edgesLeaving[i];
      // if an edge to target is found from source
      if (e->target == source)
      {
        if (e->weight == weight)
          return false; // edge already exists
        else
          e->weight = weight; // otherwise update weight of existing edge
        alreadyExists = true;
        break;
      }
    }
    if (!alreadyExists)
    {
      // otherwise add new edge to source vertex
      source->edgesLeaving[source->n_edges++] = new edge(source, target, weight);
      target->edgesLeaving[target->n_edges++] = new edge(target, source, weight);
    }
    return true;
  }
};

graph maze_g; // graph of the maze

/**
 * @brief Builds the maze graph
 * 
 */
void buildMaze() {
  // build the maze!
  for (byte r = 0; r < MAZE_HEIGHT; r++)
  {
    for (byte c = 0; c < MAZE_WIDTH; c++)
    {
      byte p = ENCODE(c, r);
      maze_g.vertices[p].pos = p; // set the pos of the node
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
      grid[r][c] = matrix.Color333(0, 0, 0);
    }
  }
  byte x, y;
  for (byte p = 0; p < MAZE_CAPACITY; p++)
  {
    x = GET_X(p);
    y = GET_Y(p);
    byte r = 2*y + 1;
    byte c = 2*x + 1;
    grid[r][c] = matrix.Color333(7, 7, 7); // color the vertex node

    // color the edge nodes
    byte x1, y1, x2, y2;
    node * v = &maze_g.vertices[p];
    for (byte i = 0; i < v->n_edges; i++)
    {
      x1 = GET_X(v->edgesLeaving[i]->source->pos);
      y1 = GET_Y(v->edgesLeaving[i]->source->pos);
      x2 = GET_X(v->edgesLeaving[i]->target->pos);
      y2 = GET_Y(v->edgesLeaving[i]->target->pos);
      r = y1 + y2 + 1;
      c = x1 + x2 + 1;
      grid[r][c] = matrix.Color333(7, 7, 7);
    }
  }
  for (byte r = 0; r < MATRIX_HEIGHT; r++)
  {
    for (byte c = 0; c < MATRIX_WIDTH; c++)
    {
      matrix.drawPixel(c, r, grid[r][c]);
    }
  }
}
