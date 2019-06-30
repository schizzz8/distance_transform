#include <iostream>
#include <limits>
#include <vector>
#include <unordered_set>
#include <queue>
#include <assert.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

struct Cell
{
  Cell():
    x(0),
    y(0),
    parent(nullptr),
    occupancy(255),
    distance(std::numeric_limits<double>::max()),
    cost(1.0){}

  int x,y;
  Cell* parent;
  int occupancy;
  double distance;
  double cost;
};

namespace std {
  template <> struct hash<Cell> {
    typedef Cell argument_type;
    typedef std::size_t result_type;
    std::size_t operator()(const Cell& id) const noexcept {
      return std::hash<int>()(id.x ^ (id.y << 4));
    }
  };
}

typedef std::vector<Cell> CellVector;
typedef std::unordered_set<Cell> CellUnorderedSet;
typedef std::array<int, 2> GridLocation;

struct Grid
{
  Grid(const size_t width_, const size_t height_):
    width(width_),
    height(height_)
  {
    cells.resize(width*height);
    for(size_t y=0; y < height; ++y)
    {
      for(size_t x=0; x < width; ++x)
      {
        Cell& c = cells[x + width*y];
        c.x = x;
        c.y = y;
      }
    }
  }

  bool hasCell(const GridLocation& id)
  {
    return 0 <= id[0] && id[0] < width
        && 0 <= id[1] && id[1] < height;
  }

  bool isTraversable(const GridLocation& id)
  {
    return cells[id[0] + width*id[1]].occupancy == 255;
  }

  Cell& operator()(const int x, const int y)
  {
    return cells[x + width*y];
  }
  const Cell& operator()(const int x, const int y) const
  {
    return cells[x + width*y];
  }

  Cell& operator()(const GridLocation& id)
  {
    return cells[id[0] + width*id[1]];
  }
  const Cell& operator()(const GridLocation& id) const
  {
    return cells[id[0] + width*id[1]];
  }

  size_t width,height;
  CellVector cells;

  static std::array<GridLocation, 8> DIRS;
};

std::array<GridLocation, 8> Grid::DIRS = {{ {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0}, {-1,1}, {0,1}, {1,1} }};

struct QElem{
  QElem(const double& d=std::numeric_limits<double>::max(), Cell* c=nullptr):
    distance(d),
    cell(c){}

  inline bool operator < (const QElem& e) const {
    return e.distance < distance ;
  }

  double distance;
  Cell* cell;
};

struct PriorityQueue{
  inline bool empty() const {
    return elements.empty();
  }

  inline void put(const double& d, Cell* c)
  {
    elements.emplace(QElem(d,c));
  }

  Cell* get()
  {
    Cell* best_elem = elements.top().cell;
    elements.pop();
    return best_elem;
  }

  std::priority_queue<QElem> elements;
};

void distanceTransform(Grid& grid)
{
  PriorityQueue frontier;
  for(size_t i = 0; i < grid.cells.size(); ++i)
  {
    Cell* current = &grid.cells[i];
    if(current->occupancy == 0)
    {
      frontier.put(current->distance,current);
      current->parent = current;
    }
  }
  std::cerr << std::endl;

  while (!frontier.empty())
  {
    Cell* current = frontier.get();
    for (GridLocation dir : Grid::DIRS)
    {
      GridLocation next{current->x + dir[0], current->y + dir[1]};
      if(!grid.hasCell(next) || !grid.isTraversable(next))
      {
        continue;
      }
      int x = next[0];
      int y = next[1];
      int dx = x - current->parent->x;
      int dy = y - current->parent->y;
      double dist = std::sqrt(dx*dx+dy*dy);

      Cell& child = grid(next);
      if(dist < child.distance)
      {
        child.parent = current->parent;
        child.distance = dist;
        frontier.put(child.distance,&child);
      }
    }
  }
}

void addRect(Grid& grid, const int x1, const int y1, const int x2, const int y2)
{
  for (int x = x1; x < x2; ++x) {
    for (int y = y1; y < y2; ++y) {
      grid(x,y).occupancy = 0;
      grid(x,y).distance = 0;
    }
  }
}

Grid makeDiagram()
{
  Grid grid(300, 150);
  addRect(grid, 30, 30, 50, 120);
  addRect(grid, 130, 40, 150, 150);
  addRect(grid, 210, 0, 230, 70);
  addRect(grid, 230, 50, 260, 70);
  return grid;
}

void drawGrid(const Grid& grid)
{
  cv::Mat image(grid.height,grid.width,CV_8UC3,cv::Vec3b(255,255,255));
  for(size_t y=0; y < grid.height; ++y)
  {
    for(size_t x=0; x < grid.width; ++x)
    {
      image.at<cv::Vec3b>(y,x) = cv::Vec3b(grid(x,y).distance,grid(x,y).distance,grid(x,y).distance);
    }
  }

  cv::namedWindow("grid",CV_WINDOW_NORMAL);
  cv::imshow("grid",image);
  cv::waitKey();
}


int main()
{
  Grid grid = makeDiagram();
  distanceTransform(grid);
  drawGrid(grid);
  return 0;
}
