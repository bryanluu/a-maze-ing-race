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

// ########## POSITION CODE ##########

struct point {
public:
  byte x; // the horizontal position in the maze
  byte y; // the vertical position in the maze

  bool operator==(const point &other) const
  {
    return (x == other.x) && (y == other.y);
  }
};

namespace std {
  template<>
  struct hash<point>
  {
    std::size_t operator()(const point& p) const
    {
      return p.x + MAZE_WIDTH*(p.y);
    }
  };
}

// ########## GRAPH CODE ##########

#define MAX_NEIGHBORS 4

class Graph {
private:
  class Vertex;
  class Edge;

  typedef std::shared_ptr<Vertex> vertex_ptr;
  
  class Vertex {
  public:
    point * pos; // vertex position
    Edge * edgesLeaving[MAX_NEIGHBORS] = {}; // edges leaving this vertex
    int cheapestEdgeCost = INT_MAX;
    Edge * cheapestEdge = NULL;
    unsigned char n_edges = 0;
  
    Vertex(point * pos)
    {
      this->pos = pos;
    }

    ~Vertex() {
      delete pos;
      for (int i=0; i<4; i++)
        delete(edgesLeaving[i]);
      delete(cheapestEdge);
    }
  };
  
  class Edge {
  public:
    vertex_ptr source;
    vertex_ptr target;
    int weight;
  
    Edge(vertex_ptr source, vertex_ptr target, int weight)
    {
      this->source = source;
      this->target = target;
      this->weight = weight;
    }
  };

public:
  typedef std::unordered_map<point, vertex_ptr> vertex_list;
  
  // hold graph vertices
  vertex_list vertices = vertex_list();

  /**
   * Insert a new vertex into the graph.
   * 
   * @param pos the position stored in the new vertex
   * @return true if the position can be inserted as a new vertex, false if it is already in the graph
   */
  bool insertVertex(point * pos)
  {
    vertex_list::const_iterator got = vertices.find(*pos);
    if (got != vertices.end()) // vertex already exists
      return false;

    vertices[*pos] = vertex_ptr(new Vertex(pos));
    return true;
  }

  /**
   * Insert a new directed edge with a positive edge weight into the graph.
   * 
   * @param source the position of the source vertex for the edge
   * @param target the position of the target vertex for the edge
   * @param weight the weight for the edge (has to be a positive integer)
   * @return true if the edge could be inserted or its weight updated, false if the edge with the
   *         same weight was already in the graph
   */
  bool insertEdge(point *source, point *target, int weight)
  {
    vertex_list::const_iterator sv = vertices.find(*source);
    vertex_list::const_iterator tv = vertices.find(*target);

    // source or target vertices invalid
    if (sv == vertices.end() || tv == vertices.end())
      return false;

    // invalid negative weight
    if (weight < 0)
      return false;

    // handle cases where edge already exists between these points
    bool alreadyExists = false;
    Edge * e;
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
      e = sv->second->edgesLeaving[i];
      // if an edge to target is found from source
      if (e->target == tv->second)
      {
        if (e->weight == weight)
          return false; // edge already exists
        else
          e->weight = weight; // otherwise update weight of existing edge
        alreadyExists = true;
        break;
      }
      e = tv->second->edgesLeaving[i];
      // if an edge to target is found from source
      if (e->target == sv->second)
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
      sv->second->edgesLeaving[sv->second->n_edges++] = new Edge(sv->second, tv->second, weight);
      tv->second->edgesLeaving[tv->second->n_edges++] = new Edge(tv->second, sv->second, weight);
    }
    return true;
  }
};

Graph graph; // graph of the maze
point maze[MAZE_HEIGHT][MAZE_WIDTH]; // points on the maze

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
      if (r > 14 || c > 14)
        break;

      maze[r][c].x = c;
      maze[r][c].y = r;
      graph.insertVertex(&maze[r][c]);

      // if (c > 0) // connect to left neighbor
      //   graph.insertEdge(p, maze[r][c - 1], rand());
      // if (r > 0) // connect to top neighbor
      //   graph.insertEdge(p, maze[r - 1][c], rand());
    }
  }
}

uint16_t grid[MATRIX_WIDTH][MATRIX_HEIGHT]; // color of each pixel in matrix

/**
 * @brief Display the maze on the matrix
 * 
 */
void displayMaze() {
  Serial.println(graph.vertices.size());
  for (byte r = 0; r < MATRIX_HEIGHT; r++)
  {
    for (byte c = 0; c < MATRIX_WIDTH; c++)
    {
      grid[r][c] = matrix.Color333(0, 0, 0);
    }
  }
  byte x, y;
  for (auto it = graph.vertices.begin(); it != graph.vertices.end(); it++)
  {
    x = it->first.x;
    y = it->first.y;
    byte r = 2*y + 1;
    byte c = 2*x + 1;
    grid[r][c] = matrix.Color333(7, 7, 7); // color the vertex node
    Serial.println(String(r) + "," + String(c));

    // color the edge nodes
    byte x1, y1, x2, y2;
    for (byte i = 0; i < it->second->n_edges; i++)
    {
      x1 = it->second->edgesLeaving[i]->source->pos->x;
      y1 = it->second->edgesLeaving[i]->source->pos->y;
      x2 = it->second->edgesLeaving[i]->target->pos->x;
      y2 = it->second->edgesLeaving[i]->target->pos->y;
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
