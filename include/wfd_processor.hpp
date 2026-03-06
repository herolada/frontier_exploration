#pragma once

#include "wfd_explorer/wfd_types.hpp"
#include "wfd_explorer/logger.hpp"

#include <vector>
#include <optional>

namespace wfd
{

/**
 * @brief Pure C++ (no-ROS) Wavefront Frontier Detection and frontier management.
 *
 * Responsibilities:
 *   1. Convert raw occupancy data into a thresholded OccupancyGrid.
 *   2. Run BFS-based WFD to extract frontier cells.
 *   3. Label connected frontier components.
 *   4. Split large frontiers and compute centroids.
 *   5. Score and rank frontier representatives given a robot position.
 */
class WFDProcessor
{
public:
  explicit WFDProcessor(const WFDParams & params, ROSLogger logger);

  /**
   * @brief Build a thresholded OccupancyGrid from raw nav_msgs data.
   *
   * @param raw   flat row-major vector of raw occupancy values (-1, 0..100)
   * @param width grid width in cells
   * @param height grid height in cells
   * @param resolution metres per cell
   * @param origin_x world x of origin
   * @param origin_y world y of origin
   * @param frame_id TF frame of the map
   * @return thresholded OccupancyGrid
   */
  OccupancyGrid buildGrid(
    const std::vector<int8_t> & raw,
    int width, int height,
    double resolution,
    double origin_x, double origin_y,
    const std::string & frame_id) const;

  /**
   * @brief Run WFD starting from robot position.
   *
   * @param grid      thresholded grid
   * @param robot_pos robot position in world coordinates
   * @return list of detected and split Frontier objects (unsorted)
   */
  std::vector<Frontier> detect(
    const OccupancyGrid & grid,
    const Point2D & robot_pos);

  /**
   * @brief Select the best frontier given robot position.
   *
   * Scores each frontier as:
   *   score = lambda * norm_info_gain^exponent - (1-lambda) * norm_distance
   *
   * @param frontiers detected frontiers
   * @param robot_pos robot world position
   * @return best frontier or nullopt if list is empty
   */
  std::optional<Frontier> selectBest(
    std::vector<Frontier> & frontiers,
    const Point2D & robot_pos);

  void updateParams(const WFDParams & params) { params_ = params; }
  const WFDParams & params() const { return params_; }

private:
  // ---------- helpers ----------

  /** Return true if cell (col,row) is a frontier cell:
   *  it must be TRAVERSABLE and have at least one UNEXPLORED neighbour. */
  bool isFrontierCell(const OccupancyGrid & grid, int col, int row) const;

  /** BFS to collect all cells reachable from (start_col, start_row) that
   *  also satisfy isFrontierCell.  Returns world positions of the component. */
  std::vector<Point2D> bfsFrontierComponent(
    const OccupancyGrid & grid,
    int start_col, int start_row,
    std::vector<bool> & visited_frontier) const;

  /** Split a frontier that is spatially large into sub-frontiers by
   *  k-means-style partitioning along the principal axis. */
  std::vector<Frontier> splitFrontier(const Frontier & f) const;

  /** Compute centroid of a list of points. */
  static Point2D centroid(const std::vector<Point2D> & pts);

  // 4-connected neighbour offsets (dx, dy)
  static constexpr int kDx[4] = {1, -1, 0,  0};
  static constexpr int kDy[4] = {0,  0, 1, -1};

  // 8-connected for frontier-cell detection
  static constexpr int kDx8[8] = {1, -1, 0,  0, 1, -1,  1, -1};
  static constexpr int kDy8[8] = {0,  0, 1, -1, 1, -1, -1,  1};

  WFDParams params_;
  ROSLogger logger_;
};

}  // namespace wfd
