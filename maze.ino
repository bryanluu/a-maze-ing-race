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
#define CONGRATS_PAUSE_TIME 1000 // in ms
#define CONGRATS_SCROLL_DELAY 1 // in ms

// ########## MAIN CODE ##########

#define MATRIX_WIDTH 31
#define MATRIX_HEIGHT 31
#define MAZE(p) (((p) - 1)/2) // conversion from matrix coordinates to maze coordinates
#define MATRIX(p) (2*(p) + 1) // conversion from maze coordinates to matrix coordinates
#define MATRIX_INTERPOLATE(p, q) ((p) + (q) + 1) // interpolate between maze coordinates in matrix space
#define SHROUD true // control whether the unseen portions of the maze are shrouded (for debug purposes)

// maze dimensions
byte mazeWidth = 5;
byte mazeHeight = 5;

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
#define SETTINGS_COLOR (matrix.ColorHSV(H_BLUE, 255, 100, true))
#define SETTINGS_UNSELECTED_TEXT_COLOR (matrix.ColorHSV(H_YELLOW, 255, 100, true))
#define SETTINGS_SELECTED_TEXT_COLOR WHITE(255)

typedef byte coord; // used for position or direction
struct node; // define a node struct for later
struct graph; // define a graph struct for later

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

// solution parameters
#define HINT_CHAR 'H'
#define HINT_DURATION 3000 // in ms
#define HINTS 3            // number of hints player has

// input parameters
#define HORIZONTAL_PIN A5
#define VERTICAL_PIN A4
#define BUTTON_PIN 0
#define CENTERPOINT 666
#define INPUT_BUFFER 100 // buffer to record a signal
#define HORIZONTAL_INCREASING Right // direction in which the signal increases horizontally
#define VERTICAL_INCREASING Down // direction in which the signal increases vertically
#define INPUT_MAX 1023 // max input value
#define INPUT_MIN 0 // min input value
#define DEFAULT_INPUT_DELAY 500 // default input delay
#define FAST_INPUT_DELAY 300 // input delay at full speed
#define FAST_INPUT_THRESHOLD 100 // how close to max/min should the input be considered fast

unsigned long lastInputTime = 0; // keeps track of last input of joystick
void readInput(bool strobe = true);

// Settings config
#define SMALL_SIZE 5
#define MEDIUM_SIZE 10
#define LARGE_SIZE 15
#define LOW_VISIBILITY 1 // in pixels (1 means player can see up to 1 pixel away)
#define MEDIUM_VISIBILITY 4
#define HIGH_VISIBILITY 10
#define OPTIONS 3

// Start Scene config
#define START_DURATION 5000 // in ms
#define START_DELAY 150 // in ms

// ########## SCENE CODE ##########

/**
 * @brief Describes a scene of the game
 * 
 */
class Scene
{
  public:
    static Scene * currentScene; // which scene is running
    String name;

    Scene()
    {
      name = "_";
    }

    /**
     * @brief Any initializing actions go here
     * 
     */
    virtual void start()
    {
      if (currentScene != this)
        currentScene = this;
    }

    /**
     * @brief Core code during scene goes here
     * 
     */
    virtual void run()
    {
      // do nothing
    }
};
Scene * Scene::currentScene = nullptr;

class SettingsScene : public Scene
{
  private:
    static const char *modeText[] PROGMEM;
    static const char *sizeText[] PROGMEM;
    static const char *shroudText[] PROGMEM;
    static const char *visibilityText[] PROGMEM;
    static const char left[] PROGMEM;
    static const char right[] PROGMEM;
    Direction lastInputDir = None;
    bool lastButtonState = false;
    enum Setting : int
    {
      GameModeSetting,
      SizeSetting,
      ShroudSetting,
      VizSetting,
      Settings
    };
    Setting currentSetting = GameModeSetting;
    static const char **settingsText[] PROGMEM;
    static const int options[];
    static const int defaults[];
    byte choice = 1;
    bool firstTime = true;

    void updateSetting();
    void displaySettings();

  public:
    enum class GameMode : int
    {
      Label,
      Game,
      Custom
    };
    GameMode mode; // mode of the maze game
    enum class Size : int
    {
      Label,
      Small,
      Medium,
      Large
    };
    Size size; // control the size of the maze
    enum class Shroud : int
    {
      Label,
      On,
      Off
    };
    Shroud shroud; // whether the shroud is on
    enum class Visibility : int
    {
      Label,
      Low,
      Medium,
      High
    };
    Visibility visibility; // control whether the unseen portions of the maze are shrouded

    SettingsScene() : Scene()
    {
      name = "Settings";
    }

    void start();
    void run();
};
const char *SettingsScene::modeText[] PROGMEM = {"Mode", "Game\n", "Cstm"};
const char *SettingsScene::sizeText[] PROGMEM = {"Size", "S", "M", "L"};
const char *SettingsScene::shroudText[] PROGMEM = {"Shrd?", "Y", "N"};
const char *SettingsScene::visibilityText[] PROGMEM = {"Viz", "L", "M", "H"};
const char SettingsScene::left[] PROGMEM = "<";
const char SettingsScene::right[] PROGMEM = ">";
const int SettingsScene::options[] = {2, 3, 2, 3};
const int SettingsScene::defaults[] = {1, 3, 1, 1};
const char **SettingsScene::settingsText[] PROGMEM = {SettingsScene::modeText, SettingsScene::sizeText, SettingsScene::shroudText, SettingsScene::visibilityText};
SettingsScene settingsScene = SettingsScene();

class StartScene : public Scene
{
  private:
    static const char textPlayer[] PROGMEM;
    static const char textStart[] PROGMEM;
    static const char textWall[] PROGMEM;
    static const char textFinish[] PROGMEM;
    static const char textHint[] PROGMEM;
    unsigned long startTime = 0;
    int16_t textY;

    void displayStartScreen();

  public:
    StartScene() : Scene()
    {
      name = "Start";
    }

    void start();
    void run();
};
const char StartScene::textPlayer[] PROGMEM = "You";
const char StartScene::textStart[] PROGMEM = "Start";
const char StartScene::textWall[] PROGMEM = "Wall";
const char StartScene::textFinish[] PROGMEM = "End";
const char StartScene::textHint[] PROGMEM = "Hint";
StartScene startScene = StartScene();

class MazeScene : public Scene
{
  private:
    graph * adj_g;  // adjacency graph of maze
    graph * maze_g; // graph of the maze
    uint16_t grid[MATRIX_HEIGHT][MATRIX_WIDTH]; // color of each pixel in matrix
    bool seen[MATRIX_HEIGHT][MATRIX_WIDTH]; // which pixels the player has seen
    unsigned long lastHintTime = -HINT_DURATION;
    byte hints = HINTS; // number of hints
    node * startNode;
    node * endNode;
    byte playerX, playerY;
    int visibility;

    void resetMaze();
    void buildAdjacencyGraph();
    void buildMaze();
    void setMazeEndpoints();
    coord approximatePlayerLocation();
    void calculateSolution();
    bool isNearPlayer(byte x, byte y);
    bool isOnMaze(byte x, byte y);
    void movePlayer();
    void useHint();
    void colorMaze();
    void colorStart();
    void colorFinish();
    void colorSolution();
    void colorPlayer();
    void brightenSurroundings();
    bool playerHasFinished();
    bool isBorder(byte r, byte c);
    void displayMaze();

  public:
    MazeScene() : Scene()
    {
      name = "Maze";
    }

    void start();
    void run();
};
MazeScene mazeScene = MazeScene();

class EndScene : public Scene
{
  private:
    bool displayFinishScreen();
    static const char congrats[] PROGMEM;
    static int16_t textMin;
    int16_t textX = matrix.width();
    int16_t hue = 0;

  public:
    EndScene() : Scene()
    {
      name = "End";
    }

    void start();
    void run();
};
const char EndScene::congrats[] PROGMEM = "You won, congratulations!!!"; // Congratulations text
int16_t EndScene::textMin =  (int16_t)sizeof(congrats) * -6;
EndScene endScene = EndScene();

void setup()
{
  Serial.begin(9600);
  randomSeed(analogRead(0));
  pinMode(0, INPUT_PULLUP);
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setTextSize(1);
  settingsScene.start();
}

Direction inputDir = None; // variable to hold direction input state
bool buttonPressed = false; // variable to hold button state
unsigned long currentTime = 0;
void loop()
{
  // Clear background
  matrix.fillScreen(0);

  currentTime = millis();
  Scene::currentScene->run();

#if !defined(__AVR__)
  // On non-AVR boards, delay slightly so screen updates aren't too quick.
  delay(20);
#endif

  // Update display
  matrix.swapBuffers(false);
}

// ########## SETTINGS CODE ##########

void SettingsScene::start()
{
  Scene::start();
  inputDir = None;
  lastInputDir = None;
  buttonPressed = false;
  lastButtonState = false;
  currentSetting = Setting::GameModeSetting;
  if (firstTime)
    choice = defaults[currentSetting];
  else
    choice = (int) mode;
}

void SettingsScene::run()
{
  Scene::run();
  if (buttonPressed && !lastButtonState)
  {
    updateSetting();
    if (currentSetting == Setting::Settings)
    {
      startScene.start();
      firstTime = false;
      return;
    }
  }
  lastButtonState = buttonPressed;

  readInput(false);
  if (inputDir != lastInputDir || currentTime - lastInputTime >= DEFAULT_INPUT_DELAY)
  {
    if (inputDir == Left)
    {
      choice = constrain(choice - 1, 1, options[currentSetting]);
    } else if (inputDir == Right)
    {
      choice = constrain(choice + 1, 1, options[currentSetting]);
    }
    lastInputTime = currentTime;
    lastInputDir = inputDir;
  }
  displaySettings();
}

void SettingsScene::updateSetting()
{
  switch (currentSetting)
  {
  case GameModeSetting:
    mode = (GameMode) choice;
    if (!firstTime)
      choice = (int) size;
    break;
  case SizeSetting:
    size = (Size) choice;
    if (!firstTime)
      choice = (int) shroud;
    break;
  case ShroudSetting:
    shroud = (Shroud) choice;
    if (!firstTime)
      choice = (int) visibility;
    break;
  case VizSetting:
    visibility = (Visibility) choice;
    break;
  }
  currentSetting = (Setting) (currentSetting + 1);
  if (firstTime)
    choice = defaults[currentSetting];
}

/**
 * @brief Displays the settings
 * 
 */
void SettingsScene::displaySettings()
{
  matrix.setCursor(2, 2);
  matrix.setTextColor(SETTINGS_COLOR);
  matrix.println(settingsText[currentSetting][0]);
  matrix.setCursor(1, 12);
  if (inputDir == Left)
    matrix.setTextColor(SETTINGS_SELECTED_TEXT_COLOR);
  else
    matrix.setTextColor(SETTINGS_UNSELECTED_TEXT_COLOR);
  matrix.print(left);
  for (byte i = 1; i <= options[currentSetting]; i++)
  {
    if (i == choice)
      matrix.setTextColor(SETTINGS_SELECTED_TEXT_COLOR);
    else
      matrix.setTextColor(SETTINGS_UNSELECTED_TEXT_COLOR);
    matrix.print(settingsText[currentSetting][i]);
  }
  if (inputDir == Right)
    matrix.setTextColor(SETTINGS_SELECTED_TEXT_COLOR);
  else
    matrix.setTextColor(SETTINGS_UNSELECTED_TEXT_COLOR);
  matrix.print(right);
}

// ########## START CODE ##########

void StartScene::start()
{
  Scene::start();
  startTime = currentTime;
  textY = 5;
}

void StartScene::run()
{
  Scene::run();
  if (currentTime - startTime >= START_DURATION)
    mazeScene.start();
  displayStartScreen();
  delay(START_DELAY);
}

/**
 * @brief Displays the starting graphics
 */
void StartScene::displayStartScreen()
{
  matrix.setCursor(0, textY--);
  matrix.setTextColor(PLAYER_COLOR);
  matrix.println(textPlayer);
  matrix.setTextColor(NEAR_WALL_COLOR);
  matrix.println(textWall);
  matrix.setTextColor(NEAR_START_COLOR);
  matrix.println(textStart);
  matrix.setTextColor(NEAR_FINISH_COLOR);
  matrix.println(textFinish);
  matrix.setTextColor(NEAR_SOLUTION_COLOR);
  matrix.println(textHint);
}

// ########## MAZE CODE ##########

void MazeScene::start()
{
  Scene::start();
  resetMaze();
  inputDir = None;
  buttonPressed = false;
  hints = HINTS;
  lastHintTime = -HINT_DURATION;
  switch (settingsScene.visibility)
  {
  case SettingsScene::Visibility::Low:
    visibility = LOW_VISIBILITY;
    break;
  case SettingsScene::Visibility::Medium:
    visibility = MEDIUM_VISIBILITY;
    break;
  case SettingsScene::Visibility::High:
    visibility = HIGH_VISIBILITY;
    break;
  default:
    visibility = 0;
    break;
  }
}

void MazeScene::run()
{
  if (playerHasFinished())
    endScene.start();

  Scene::run();
  readInput();
  movePlayer();
  if (buttonPressed)
  {
    calculateSolution();
    useHint();
  }

  colorMaze();
  colorStart();
  if (currentTime - lastHintTime < HINT_DURATION)
    colorSolution();
  colorFinish();
  colorPlayer();
  displayMaze();
}

// encodes the position given x and y
#define ENCODE(x, y) ((x) + (mazeWidth) * (y))
// decodes the x value of position
#define GET_X(p) ((p) % (mazeWidth))
// decodes the y value of position
#define GET_Y(p) ((p) / (mazeWidth))
// max number of neighbors this node can have
#define MAX_NEIGHBORS 4
// number of edges
#define MAX_EDGES ((MAZE(MATRIX_WIDTH) * MAZE(MATRIX_HEIGHT)) - 1)

typedef byte coord; // specify type for position

struct node
{
  coord pos = None;         // the position in the maze
  int weights[MAX_NEIGHBORS]; // neighboring weights of edges of this node
  int value = INT_MAX;      // integer value to keep track of (cheapestEdgeWeight or distance)
  coord id = None;          // id of edge or node to keep track of
  bool used = false;        // whether the node has been used in the maze

  node()
  {
    for (coord i = 0; i < MAX_NEIGHBORS; i++)
      weights[i] = None;
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
   * @return coord - the position in the relative direction
   */
  coord pos_relative(char dir)
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
    if (x < 0 || x >= mazeWidth || y < 0 || y >= mazeHeight)
      return None;

    return ENCODE(x, y);
  }
};

bool compare(node *u, node *v);

typedef std::vector<node *> vertex_list;

namespace std
{
  template<>
  struct hash<node>
  {
    std::size_t operator()(const node& p) const
    {
      return p.pos;
    }
  };
}

struct graph
{
  node * vertices; // array of graph vertices
  byte size; // number of vertices

  /**
   * @brief Construct a new graph object
   * 
   * @param width width of new graph
   * @param height height of new graph
   */
  graph(byte width, byte height)
  {
    size = width * height;
    vertices = new node[size];
  }

  /**
   * Insert a new directed edge with a positive edge weight into the graph.
   * 
   * @param s the encoded position of the source vertex for the edge
   * @param t the encoded position of the target vertex for the edge
   * @param weight the weight for the edge (has to be a positive integer)
   * @return true if the edge could be inserted or its weight updated, false if the edge with the
   *         same weight was already in the graph
   */
  bool insertEdge(coord s, coord t, int weight)
  {
    // source or target vertices invalid
    if (s >= size || t >= size)
      return false;

    // invalid negative weight
    if (weight < 0)
      return false;

    node *source = &vertices[s];
    node *target = &vertices[t];
    source->weights[(*target) - (*source)] = weight;
    target->weights[(*source) - (*target)] = weight;
    return true;
  }

  /**
   * @brief Destroy the graph object
   * 
   */
  ~graph()
  {
    delete[] vertices;
  }
};

/**
 * @brief Resets the maze to a new one
 * 
 */
void MazeScene::resetMaze()
{
  delete adj_g;
  delete maze_g;
  buildMaze();
  for (byte r = 0; r < MATRIX_HEIGHT; r++)
  {
    for (byte c = 0; c < MATRIX_WIDTH; c++)
    {
      seen[r][c] = false;
      grid[r][c] = BLACK;
    }
  }
}

/**
 * @brief Builds the adjacency graph for the maze
 *
 */
void MazeScene::buildAdjacencyGraph()
{
  for (byte r = 0; r < mazeHeight; r++)
  {
    for (byte c = 0; c < mazeWidth; c++)
    {
      coord p = ENCODE(c, r);
      node *v = &adj_g->vertices[p];
      v->pos = p; // set the pos of the node

      // connect to top node
      if (r > 0)
        adj_g->insertEdge(p, ENCODE(c, r - 1), random(MAX_EDGES));
      // connect to left node
      if (c > 0)
        adj_g->insertEdge(p, ENCODE(c - 1, r), random(MAX_EDGES));
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

/**
 * @brief Set the end points of the maze object
 * 
 */
void MazeScene::setMazeEndpoints()
{
  // set endpoints
  startNode = &maze_g->vertices[0];
  endNode = &maze_g->vertices[maze_g->size - 1];
  playerX = MATRIX(GET_X(startNode->pos));
  playerY = MATRIX(GET_Y(startNode->pos));
}

/**
 * @brief Builds the maze graph using Prim's algorithm
 * 
 */
void MazeScene::buildMaze()
{
  byte size;

  switch(settingsScene.size)
  {
    case SettingsScene::Size::Small:
      size = SMALL_SIZE;
      break;
    case SettingsScene::Size::Medium:
      size = MEDIUM_SIZE;
      break;
    case SettingsScene::Size::Large:
      size = LARGE_SIZE;
      break;
    default:
      return;
  }

  mazeWidth = size;
  mazeHeight = size;

  adj_g = new graph(mazeWidth, mazeHeight);
  maze_g = new graph(mazeWidth, mazeHeight);

  // build the adjacency graph for the edge information
  buildAdjacencyGraph();

  // initialize the vertices in the maze
  for (coord p = 0; p < adj_g->size; p++)
    maze_g->vertices[p].pos = p;

  setMazeEndpoints();

  // initialize the queue of vertices not in the maze
  std::priority_queue<node *, vertex_list, decltype(&compare)> pq(&compare);
  pq.push(&adj_g->vertices[startNode->pos]); // build from start node

  while (!pq.empty()) // until the maze has all vertices
  {
    node *v = pq.top(); // take the vertex with the cheapest edge
    pq.pop();

    v->used = true;                     // mark v as a used vertex in the maze
    if (v->id != None)                  // if v touches the maze, add cheapest neighboring edge to maze
      maze_g->insertEdge(v->pos, v->pos_relative(v->id), v->value);

    // loop through outgoing edges of vertex
    for (byte i = 0; i < MAX_NEIGHBORS; i++)
    {
      int e = v->weights[i]; // edge cost
      if (e == None)       // if edge doesn't exist
        continue;

      coord n = v->pos_relative(i);  // get neighbor index
      node *w = &adj_g->vertices[n]; // neighbor node
      if (!w->used && e < w->value) // if a new neighboring cheapest edge is found
      {
        // update neighbor's cheapest edge
        w->value = v->weights[i];
        w->id = ((*v) - (*w));
        pq.push(w);
      }
    }
  }
}

/**
 * @brief Obtain the location of the player on the maze, closest to the finish
 *
 * @return coord position of player on maze closest to finish
 */
coord MazeScene::approximatePlayerLocation()
{
  byte x = MAZE(playerX);
  byte y = MAZE(playerY);
  coord pos = ENCODE(x, y);
  return pos;
}

node * current; // approximate node of player in maze space
/**
 * @brief Calculates the shortest path from player position to finish using Dijkstra's algorithm
 * 
 */
void MazeScene::calculateSolution()
{
  // initialize vertices
  for (coord p = 0; p < maze_g->size; p++)
  {
    node *v = &maze_g->vertices[p];
    v->value = INT_MAX;
    v->id = None;
    v->used = false;
  }

  // set finish distance to 0
  endNode->value = 0;
  endNode->used = true; // mark the finish as visited

  coord playerPos = approximatePlayerLocation();
  current = &maze_g->vertices[playerPos];

  // initialize queue of vertices
  std::priority_queue<node *, vertex_list, decltype(&compare)> pq(&compare);
  pq.push(endNode);

  while (!pq.empty()) // until the queue is empty
  {
    node * u = pq.top(); // u is best vertex
    pq.pop();
    if (u == current)
      break; // we've reached the end of the maze!

    // loop through neighbors
    for (coord i = 0; i < MAX_NEIGHBORS; i++)
    {
      int e = u->weights[i]; // edge cost
      if (e == None)       // if edge doesn't exist
        continue;

      coord n = u->pos_relative(i); // get neighbor index
      node *v = &maze_g->vertices[n]; // neighbor node
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

/**
 * @brief Display the maze on the matrix
 * 
 */
void MazeScene::displayMaze()
{
  uint16_t color;
  byte rowOffset, colOffset; // used for centering
  for (byte r = 0; r < MATRIX(mazeHeight); r++)
  {
    for (byte c = 0; c < MATRIX(mazeWidth); c++)
    {
      if (!SHROUD || settingsScene.shroud == SettingsScene::Shroud::Off || isBorder(r, c) || seen[r][c] || grid[r][c] == SEEN_FINISH_COLOR)
        color = grid[r][c]; // show seen pixels
      else
        color = BLACK; // shroud maze sections that haven't been seen
      rowOffset = (MATRIX_HEIGHT - MATRIX(mazeHeight))/2;
      colOffset = (MATRIX_WIDTH - MATRIX(mazeWidth))/2;
      matrix.drawPixel(c + colOffset, r + rowOffset, color);
    }
  }
}

/**
 * @brief Whether the pixel is part of the border
 * 
 * @param r 
 * @param c 
 * @return true 
 * @return false 
 */
bool MazeScene::isBorder(byte r, byte c)
{
  return r == 0 || r == MATRIX(mazeHeight) - 1 || c == 0 || c == MATRIX(mazeWidth) - 1;
}

/**
 * @brief Check whether given pixel is near player
 * 
 * @param x horizontal coord
 * @param y vertical coord
 * @return true if close to player
 * @return false otherwise
 */
bool MazeScene::isNearPlayer(byte x, byte y)
{
  return (abs(x - playerX) <= visibility) && (abs(y - playerY) <= visibility);
}

/**
 * @brief Check whether given pixel is on maze
 * 
 * @param x horizontal coord
 * @param y vertical coord
 * @return true if on maze
 * @return false otherwise
 */
bool MazeScene::isOnMaze(byte x, byte y)
{
  return (x >= 0) && (x < MATRIX(mazeWidth)) && (y >= 0) && (y < MATRIX(mazeHeight));
}

/**
 * @brief Colors the start of the maze
 * 
 */
void MazeScene::colorStart()
{
  // Color special nodes
  byte x, y;
  x = GET_X(startNode->pos);
  y = GET_Y(startNode->pos);
  grid[MATRIX(y)][MATRIX(x)] = SEEN_START_COLOR;
}

/**
 * @brief Colors the finish of the maze
 * 
 */
void MazeScene::colorFinish()
{
  // Color special nodes
  byte x, y;
  x = GET_X(endNode->pos);
  y = GET_Y(endNode->pos);
  grid[MATRIX(y)][MATRIX(x)] = SEEN_FINISH_COLOR;
}

/**
 * @brief Color the maze and walls
 * 
 */
void MazeScene::colorMaze()
{
  for (byte r = 0; r < MATRIX_HEIGHT; r++)
  {
    for (byte c = 0; c < MATRIX_WIDTH; c++)
    {
      grid[r][c] = SEEN_WALL_COLOR; // color wall
    }
  }
  byte x, y;
  for (coord p = 0; p < maze_g->size; p++)
  {
    x = GET_X(p);
    y = GET_Y(p);
    byte r = MATRIX(y);
    byte c = MATRIX(x);
    grid[r][c] = SEEN_MAZE_COLOR; // color the vertex node

    // color the edge nodes
    byte x2, y2;
    node *v = &maze_g->vertices[p];
    for (coord i = 0; i < MAX_NEIGHBORS; i++)
    {
      if (v->weights[i] == None) // if edge doesn't exist
        continue;

      node *u = &maze_g->vertices[v->pos_relative(i)];
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
void MazeScene::colorSolution()
{
  node *v = current;
  if (playerX % 2 == 0 || playerY % 2 == 0) // player is in-between nodes
  {
    node *a, *b;
    byte p1, p2;
    if (playerX % 2 == 0)
    {
      // in a horizontal corridor
      p1 = ENCODE(MAZE(playerX), MAZE(playerY));
      p2 = ENCODE(MAZE(playerX + 1), MAZE(playerY));
    }
    else
    {
      // in a vertical corridor
      p1 = ENCODE(MAZE(playerX), MAZE(playerY));
      p2 = ENCODE(MAZE(playerX), MAZE(playerY + 1));
    }
    a = &maze_g->vertices[p1];
    b = &maze_g->vertices[p2];
    if (a->value > b->value)
    {
      v = b;
    }
  }

  byte x, y;
  byte lx, ly; // last node pos
  byte r, c;
  bool started = false;
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
    if (started)
    { // Color the in-between step from last node position
      r = MATRIX_INTERPOLATE(y, ly);
      c = MATRIX_INTERPOLATE(x, lx);
      grid[r][c] = SEEN_SOLUTION_COLOR;
    } else
    started = true;
    if (v == endNode) // if we've reached the finish
      break;         // stop loop

    v = &maze_g->vertices[v->id]; // move to predecessor
  }
}

/**
 * @brief Brighten pixels around player
 * 
 */
void MazeScene::brightenSurroundings()
{
  for (short x = playerX - visibility; x <= playerX + visibility; x++)
  {
    for (short y = playerY - visibility; y <= playerY + visibility; y++)
    {
      if (!isOnMaze(x, y) || !isNearPlayer(x, y))
        continue;

      uint16_t color = grid[y][x];
      if (color == SEEN_WALL_COLOR)
        color = NEAR_WALL_COLOR;
      else if (color == SEEN_START_COLOR)
        color = NEAR_START_COLOR;
      else if (color == SEEN_FINISH_COLOR)
        color = NEAR_FINISH_COLOR;
      else if (color == SEEN_SOLUTION_COLOR)
        color = SEEN_SOLUTION_COLOR;
      grid[y][x] = color; // update color
      seen[y][x] = true; // mark pixel as seen
    }
  }
}

/**
 * @brief Color the player position
 * 
 */
void MazeScene::colorPlayer()
{
  grid[playerY][playerX] = PLAYER_COLOR;
  brightenSurroundings();
}

/**
 * @brief Returns the opposing direction
 * 
 * @param dir a given direction
 * @return Direction the opposite direction
 */
Direction opposite(Direction dir)
{
  switch (dir)
  {
  case Up:
    return Down;
  case Down:
    return Up;
  case Left:
    return Right;
  case Right:
    return Left;
  default:
    return None;
  }
}

/**
 * @brief Reads the input
 * 
 * @param strobe whether to "strobe" the analog inputs
 */
void readInput(bool strobe)
{
  int horizontal = analogRead(HORIZONTAL_PIN);
  int vertical = analogRead(VERTICAL_PIN);

  int dx = horizontal - CENTERPOINT;
  int dy = vertical - CENTERPOINT;
  int inputDelay = DEFAULT_INPUT_DELAY;
  if (strobe)
    inputDir = None;
  if (abs(dx) > INPUT_BUFFER || abs(dy) > INPUT_BUFFER)
  {
    if (abs(dx) >= abs(dy)) 
    {
      if (dx > INPUT_BUFFER)
      {
        if (abs(horizontal - INPUT_MAX) < FAST_INPUT_THRESHOLD)
          inputDelay = FAST_INPUT_DELAY;
        if (currentTime - lastInputTime > inputDelay)
          inputDir = HORIZONTAL_INCREASING;
      }
      else if (dx < -INPUT_BUFFER)
      {
        if (abs(horizontal - INPUT_MIN) < FAST_INPUT_THRESHOLD)
          inputDelay = FAST_INPUT_DELAY;
        if (currentTime - lastInputTime > inputDelay)
          inputDir = opposite(HORIZONTAL_INCREASING);
      }
    } else
    {
      if (dy > INPUT_BUFFER)
      {
        if (abs(vertical - INPUT_MAX) < FAST_INPUT_THRESHOLD)
          inputDelay = FAST_INPUT_DELAY;
        if (currentTime - lastInputTime > inputDelay)
          inputDir = VERTICAL_INCREASING;
      }
      else if (dy < -INPUT_BUFFER)
      {
        if (abs(vertical - INPUT_MIN) < FAST_INPUT_THRESHOLD)
          inputDelay = FAST_INPUT_DELAY;
        if (currentTime - lastInputTime > inputDelay)
          inputDir = opposite(VERTICAL_INCREASING);
      }
    }
  } else if (!strobe)
  {
    inputDir = None;
  }

  if (strobe && inputDir != None)
  {
    lastInputTime = currentTime;
  }
  buttonPressed = !digitalRead(BUTTON_PIN); // flip the signal, aka LOW means true, HIGH means false
}

/**
 * @brief Move the position of the player in the given direction
 * 
 */
void MazeScene::movePlayer()
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
bool MazeScene::playerHasFinished()
{
  return playerX == MATRIX(GET_X(endNode->pos)) && playerY == MATRIX(GET_Y(endNode->pos));
}

/**
 * @brief Use a hint to show the solution for a short period of time
 * 
 */
void MazeScene::useHint()
{
  if (currentTime - lastHintTime < HINT_DURATION)
    return;

  if (--hints <= HINTS)
    lastHintTime = currentTime;
}

// ########## END CODE ##########

void EndScene::start()
{
  Scene::start();
  textX = matrix.width();
  hue = 0;
}

void EndScene::run()
{
  Scene::run();
  bool finished = displayFinishScreen();
  if (finished)
    settingsScene.start();
  Serial.println(finished);
}

/**
 * @brief Displays the finishing graphics
 *
 * @return whether the finish screen is done
 */
bool EndScene::displayFinishScreen()
{
  matrix.setTextColor(matrix.ColorHSV(hue, 255, 255, true));
  matrix.setCursor(textX, matrix.height() / 2 - 4);
  matrix.print(congrats);
  textX--; // move text left
  if (textX < textMin)
  {
    delay(CONGRATS_PAUSE_TIME);
    return true;
  }
  hue += 7;
  if (hue >= 1536)
    hue = 0;
  delay(CONGRATS_SCROLL_DELAY);
  return false;
}
