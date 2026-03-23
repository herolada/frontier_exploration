#pragma once

#include "wfd_types.hpp"
#include "ros_logger.hpp"

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
    const Pose2D & robot_pos);

  /**
   * @brief Select the best frontier given robot position.
   *
   * Scores each frontier as:
   *   score = w[0] * norm_info_gain^exponent + w[1] * norm_distance + w[2] * norm_yaw_diff
   *
   * @param frontiers detected frontiers
   * @param robot_pos robot world position
   * @return best frontier or nullopt if list is empty
   */
  std::optional<Frontier> selectBest(
    std::vector<Frontier> & frontiers,
    const Pose2D & robot_pos);

  std::optional<Frontier> selectBest(
    std::vector<Frontier> & frontiers,
    const Pose2D & robot_pos,
    const Pose2D & center_pose);

  void updateParams(const WFDParams & params) { params_ = params; }
  const WFDParams & params() const { return params_; }

private:
  // ---------- helpers ----------

  /** Return true if cell (col,row) is a frontier cell:
   *  it must be TRAVERSABLE and have at least one UNEXPLORED neighbour. */
  bool isFrontierCell(const OccupancyGrid & grid, int col, int row) const;


  /**
   * @brief Split a frontier component into sub-frontiers using k-means.
   *
   * Number of clusters:  n_r = 1 + floor( f / (1.8 * D) + 0.5 )
   * where f = number of frontier cells, D = sensor_range converted to cells.
   * Returns the frontier unchanged when n_r == 1.
   *
   * Seeds are chosen as evenly-spaced indices into the BFS-ordered cell list,
   * giving good spatial spread without a separate distance computation pass.
   */
  std::vector<Frontier> splitFrontierKMeans(const Frontier & f);

  /** Compute centroid of a list of points. */
  static Pose2D centroid(const std::vector<Pose2D> & pts);

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
