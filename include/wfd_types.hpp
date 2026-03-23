#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <limits>
#include <string>

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

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  double distanceTo(const Pose2D & other) const
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

  Pose2D cellToWorld(int col, int row) const
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
  std::vector<Pose2D> cells;   // individual frontier cell world positions
  Pose2D centroid;
  double size{0.0};             // number of cells (used as info-gain proxy)
  double score{-1e6};            // exploration score
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
  int    min_frontier_size{3};  // discard frontiers smaller than this (cells)
  double min_frontier_dist{3.0};

  // Frontier splitting via k-means.
  // Number of clusters: n_r = 1 + floor(f / (1.8 * D) + 0.5)
  // where f = number of frontier cells, D = sensor_range_cells.
  // sensor_range is expressed in metres and converted to cells internally.
  double sensor_range{3.0};     // sensor range D (metres)

  // K-means convergence
  int    kmeans_max_iter{100};  // maximum k-means iterations per frontier

  // Goal selection
  std::vector<double> weights{0., 1., 0.5}; // weight: score = w[0] * norm_info_gain^exponent + w[1] * norm_distance + w[2] * norm_yaw_diff
                                //  info_gain normalised by max, distance normalised by max
  double info_gain_exponent{1.0}; // raise info_gain to this power before scoring
};

}  // namespace wfd
