#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <limits>

namespace wfd
{

// ---------------------------------------------------------------------------
// Grid cell representation
// ---------------------------------------------------------------------------

enum class CellState : uint8_t
{
  TRAVERSABLE = 0,
  OBSTACLE    = 1,
  UNEXPLORED  = 2
};

struct Point2D
{
  double x{0.0};
  double y{0.0};

  double distanceTo(const Point2D & other) const
  {
    const double dx = x - other.x;
    const double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
  }
};

// ---------------------------------------------------------------------------
// Grid map (no ROS dependency)
// ---------------------------------------------------------------------------

struct OccupancyGrid
{
  int width{0};
  int height{0};
  double resolution{0.05};   // metres per cell
  double origin_x{0.0};      // world coords of cell (0,0)
  double origin_y{0.0};
  std::string frame_id;
  std::vector<CellState> cells;  // row-major: index = row*width + col

  bool valid() const { return width > 0 && height > 0 && !cells.empty(); }

  int index(int col, int row) const { return row * width + col; }

  CellState at(int col, int row) const
  {
    if (col < 0 || col >= width || row < 0 || row >= height) {
      return CellState::OBSTACLE;  // treat out-of-bounds as obstacle
    }
    return cells[static_cast<size_t>(index(col, row))];
  }

  Point2D cellToWorld(int col, int row) const
  {
    return {origin_x + (col + 0.5) * resolution,
            origin_y + (row + 0.5) * resolution};
  }

  std::pair<int,int> worldToCell(double wx, double wy) const
  {
    int col = static_cast<int>((wx - origin_x) / resolution);
    int row = static_cast<int>((wy - origin_y) / resolution);
    return {col, row};
  }
};

// ---------------------------------------------------------------------------
// Frontier representation
// ---------------------------------------------------------------------------

struct Frontier
{
  std::vector<Point2D> cells;   // individual frontier cell world positions
  Point2D centroid;
  double size{0.0};             // number of cells (used as info-gain proxy)
  double score{0.0};            // exploration score
};

// ---------------------------------------------------------------------------
// WFD parameters (no ROS dependency)
// ---------------------------------------------------------------------------

struct WFDParams
{
  // Thresholding
  int free_threshold{50};       // raw occupancy value < this → TRAVERSABLE
  int occ_threshold{65};        // raw occupancy value >= this → OBSTACLE
                                // between the two is also treated as obstacle for safety

  // Frontier filtering
  int   min_frontier_size{3};   // discard frontiers smaller than this (cells)
  double max_frontier_split_size{1.5}; // split frontiers whose extent (m) exceeds this

  // Goal selection
  double lambda{0.5};           // weight: score = lambda * info_gain - (1-lambda) * distance
                                //  info_gain normalised by max, distance normalised by max
  double info_gain_exponent{1.0}; // raise info_gain to this power before scoring
};

}  // namespace wfd
