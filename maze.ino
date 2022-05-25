// Custom Maze game on Adafruit's 32x32 RGB LED matrix:
// http://www.adafruit.com/products/607
// 32x32 MATRICES DO NOT WORK WITH ARDUINO UNO or METRO 328.

// Written by Bryan Luu.
// BSD license, all text above must be included in any redistribution.

#include <RGBmatrixPanel.h>
#include <limits.h>
#include <map>

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

class Point {

private:
  int x; // the horizontal position in the maze
  int y; // the vertical position in the maze

public:
  Point(int x, int y)
  {
    this->x = x;
    this->y = y;
  }

  int getX()
  {
    return x;
  }

  int getY()
  {
    return y;
  }
};

// ########## GRAPH CODE ##########

class Graph {
private:
  class Vertex;
  class Edge;
  
  class Vertex {
  public:
    Point * pos; // vertex position
    Edge * edgesLeaving[4] = {}; // edges leaving this vertex
    int cheapestEdgeCost = INT_MAX;
    Edge * cheapestEdge = NULL;
  
    Vertex(Point * pos)
    {
      this->pos = pos;
    }
  };
  
  class Edge {
  public:
    Vertex * source;
    Vertex * target;
    int weight;
  
    Edge(Vertex * source, Vertex * target, int weight)
    {
      this->source = source;
      this->target = target;
      this->weight = weight;
    }
  };

public:
  
};


void buildMaze() {
  // build the maze!
  Point pos = Point(3, 4);
}
