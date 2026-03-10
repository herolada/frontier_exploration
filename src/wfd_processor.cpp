#include "wfd_processor.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace wfd
{

// ============================================================
//  Compile-time static member definitions
// ============================================================
constexpr int WFDProcessor::kDx[4];
constexpr int WFDProcessor::kDy[4];
constexpr int WFDProcessor::kDx8[8];
constexpr int WFDProcessor::kDy8[8];

// ============================================================
//  Constructor
// ============================================================
WFDProcessor::WFDProcessor(const WFDParams & params, ROSLogger logger)
: params_(params), logger_(std::move(logger))
{}

// ============================================================
//  buildGrid
// ============================================================
OccupancyGrid WFDProcessor::buildGrid(
  const std::vector<int8_t> & raw,
  int width, int height,
  double resolution,
  double origin_x, double origin_y,
  const std::string & frame_id) const
{
  OccupancyGrid grid;
  grid.width      = width;
  grid.height     = height;
  grid.resolution = resolution;
  grid.origin_x   = origin_x;
  grid.origin_y   = origin_y;
  grid.frame_id   = frame_id;
  grid.cells.resize(static_cast<size_t>(width * height));

  for (int i = 0; i < width * height; ++i) {
    const int8_t v = raw[static_cast<size_t>(i)];
    if (v == -1) {
      grid.cells[static_cast<size_t>(i)] = CellState::UNEXPLORED;
    } else if (v < params_.free_threshold) {
      grid.cells[static_cast<size_t>(i)] = CellState::TRAVERSABLE;
    } else {
      // Both [free_threshold, occ_threshold) and [occ_threshold, 100] → obstacle
      // to be conservative with narrow passages.
      grid.cells[static_cast<size_t>(i)] = CellState::OBSTACLE;
    }
  }
  return grid;
}

// ============================================================
//  isFrontierCell
// ============================================================
bool WFDProcessor::isFrontierCell(const OccupancyGrid & grid, int col, int row) const
{
  if (grid.at(col, row) != CellState::TRAVERSABLE) return false;
  for (int d = 0; d < 8; ++d) {
    if (grid.at(col + kDx8[d], row + kDy8[d]) == CellState::UNEXPLORED) return true;
  }
  return false;
}

// ============================================================
//  bfsFrontierComponent
// ============================================================
std::vector<Pose2D> WFDProcessor::bfsFrontierComponent(
  const OccupancyGrid & grid,
  int start_col, int start_row,
  std::vector<bool> & visited_frontier) const
{
  std::vector<Pose2D> component;
  std::queue<std::pair<int,int>> q;

  auto idx = [&](int c, int r){ return r * grid.width + c; };

  q.push({start_col, start_row});
  visited_frontier[static_cast<size_t>(idx(start_col, start_row))] = true;

  while (!q.empty()) {
    auto [col, row] = q.front();
    q.pop();

    component.push_back(grid.cellToWorld(col, row));

    for (int d = 0; d < 4; ++d) {
      int nc = col + kDx[d];
      int nr = row + kDy[d];
      if (nc < 0 || nc >= grid.width || nr < 0 || nr >= grid.height) continue;
      int ni = idx(nc, nr);
      if (!visited_frontier[static_cast<size_t>(ni)] && isFrontierCell(grid, nc, nr)) {
        visited_frontier[static_cast<size_t>(ni)] = true;
        q.push({nc, nr});
      }
    }
  }
  return component;
}

// ============================================================
//  centroid
// ============================================================
Pose2D WFDProcessor::centroid(const std::vector<Pose2D> & pts)
{
  Pose2D c{0.0, 0.0, 0.0};
  if (pts.empty()) return c;
  for (const auto & p : pts) { c.x += p.x; c.y += p.y; }
  c.x /= static_cast<double>(pts.size());
  c.y /= static_cast<double>(pts.size());
  return c;
}

// ============================================================
//  splitFrontierKMeans
// ============================================================
std::vector<Frontier> WFDProcessor::splitFrontierKMeans(const Frontier & f)
{
  const std::size_t n_cells = f.cells.size();

  // ── 1. Compute number of clusters ──────────────────────────────────────
  // n_r = 1 + floor( f / (1.8 * D) + 0.5 )
  // D is sensor_range in cells = sensor_range_m / resolution.
  // We store sensor_range in metres in params; the frontier cells are already
  // in world (metre) coordinates, so we derive D_m = sensor_range directly.
  const double D_m  = params_.sensor_range;              // metres
  const double f_d  = static_cast<double>(n_cells);
  const int    n_r  = 1 + static_cast<int>(std::floor(f_d / (1.8 * D_m) + 0.5));

  logger_.info("KMeans split: {} cells, D={:.1f} m → {} cluster(s)", n_cells, D_m, n_r);

  if (n_r <= 1) {
    return {f};  // no split needed
  }

  // ── 2. Seed initialisation ─────────────────────────────────────────────
  // Evenly-spaced indices into the BFS-ordered cell list give spatially
  // spread seeds without an extra distance pass.
  std::vector<Pose2D> means(static_cast<std::size_t>(n_r));
  for (int k = 0; k < n_r; ++k) {
    std::size_t idx = static_cast<std::size_t>(
      static_cast<double>(k) / static_cast<double>(n_r) * static_cast<double>(n_cells));
    idx = std::min(idx, n_cells - 1);
    means[static_cast<std::size_t>(k)] = f.cells[idx];
  }

  // ── 3. K-means iterations ──────────────────────────────────────────────
  std::vector<int> labels(n_cells, 0);

  for (int iter = 0; iter < params_.kmeans_max_iter; ++iter) {
    // Assignment step
    bool changed = false;
    for (std::size_t i = 0; i < n_cells; ++i) {
      double best_dist = std::numeric_limits<double>::max();
      int    best_k    = 0;
      for (int k = 0; k < n_r; ++k) {
        double d = f.cells[i].distanceTo(means[static_cast<std::size_t>(k)]);
        if (d < best_dist) { best_dist = d; best_k = k; }
      }
      if (labels[i] != best_k) { labels[i] = best_k; changed = true; }
    }

    // Update step
    std::vector<Pose2D>    sums(static_cast<std::size_t>(n_r), {0.0, 0.0, 0.0});
    std::vector<std::size_t> counts(static_cast<std::size_t>(n_r), 0);
    for (std::size_t i = 0; i < n_cells; ++i) {
      std::size_t k = static_cast<std::size_t>(labels[i]);
      sums[k].x += f.cells[i].x;
      sums[k].y += f.cells[i].y;
      ++counts[k];
    }
    for (int k = 0; k < n_r; ++k) {
      std::size_t sk = static_cast<std::size_t>(k);
      if (counts[sk] > 0) {
        means[sk].x = sums[sk].x / static_cast<double>(counts[sk]);
        means[sk].y = sums[sk].y / static_cast<double>(counts[sk]);
      }
    }

    if (!changed) {
      logger_.info("KMeans converged after {} iteration(s)", iter + 1);
      break;
    }
  }

  // ── 4. Build output Frontier objects ───────────────────────────────────
  std::vector<std::vector<Pose2D>> clusters(static_cast<std::size_t>(n_r));
  for (std::size_t i = 0; i < n_cells; ++i) {
    clusters[static_cast<std::size_t>(labels[i])].push_back(f.cells[i]);
  }

  std::vector<Frontier> result;
  result.reserve(static_cast<std::size_t>(n_r));
  for (int k = 0; k < n_r; ++k) {
    auto & cl = clusters[static_cast<std::size_t>(k)];
    if (static_cast<int>(cl.size()) < params_.min_frontier_size) continue;
    Frontier seg;
    seg.cells    = std::move(cl);
    seg.centroid = centroid(seg.cells);
    seg.size     = static_cast<double>(seg.cells.size());
    result.push_back(std::move(seg));
  }
  return result;
}

// ============================================================
//  detect
// ============================================================
std::vector<Frontier> WFDProcessor::detect(
  const OccupancyGrid & grid,
  const Pose2D & robot_pos)
{
  if (!grid.valid()) {
    logger_.warn("WFD: received invalid grid, skipping");
    return {};
  }

  auto t_start = std::chrono::steady_clock::now();

  const int total = grid.width * grid.height;
  std::vector<bool> visited_frontier(static_cast<size_t>(total), false);

  std::vector<Frontier> frontiers;

  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      int i = grid.index(col, row);
      if (visited_frontier[static_cast<size_t>(i)]) continue;
      if (!isFrontierCell(grid, col, row)) continue;

      visited_frontier[static_cast<size_t>(i)] = true;

      // BFS to collect connected frontier component
      auto cells = bfsFrontierComponent(grid, col, row, visited_frontier);

      if (static_cast<int>(cells.size()) < params_.min_frontier_size) continue;

      Frontier f;
      f.cells    = std::move(cells);
      f.centroid = centroid(f.cells);
      f.size     = static_cast<double>(f.cells.size());

      // Split via k-means (n_r formula)
      auto parts = splitFrontierKMeans(f);
      for (auto & part : parts) {
        frontiers.push_back(std::move(part));
      }
    }
  }

  auto t_end = std::chrono::steady_clock::now();
  double ms  = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  logger_.info("WFD: found {} frontiers in {:.1f} ms (grid {}x{})",
    frontiers.size(), ms, grid.width, grid.height);

  (void)robot_pos;  // could be used to sort by proximity to seed the BFS
  return frontiers;
}

// ============================================================
//  selectBest
// ============================================================
std::optional<Frontier> WFDProcessor::selectBest(
  std::vector<Frontier> & frontiers,
  const Pose2D & robot_pos)
{
  if (frontiers.empty()) return std::nullopt;

  // Normalisation factors
  double max_info = 0.0;
  double max_dist = 0.0;
  double max_yaw_diff = 0.0;
  for (const auto & f : frontiers) {
    max_info = std::max(max_info, f.size); // TODO replace this with something better than information gain == number of frontire cells connected to this component.
    max_dist = std::max(max_dist, robot_pos.distanceTo(f.centroid));
    max_yaw_diff = std::fabs(std::atan2(f.centroid.y - robot_pos.y, f.centroid.x - robot_pos.x) - robot_pos.yaw);
  }  
  
  if (max_info < 1e-9) max_info = 1.0;
  if (max_dist < 1e-9) max_dist = 1.0;
  if (max_yaw_diff < 1e-9) max_yaw_diff = 1.0;

  const std::vector<double> w = params_.weights;
  const double exp = params_.info_gain_exponent;

  for (auto & f : frontiers) {
    double norm_info = std::pow(f.size / max_info, exp);
    double norm_dist = robot_pos.distanceTo(f.centroid) / max_dist;
    double norm_yaw_diff = std::fabs(std::atan2(f.centroid.y - robot_pos.y, f.centroid.x - robot_pos.x) - robot_pos.yaw) / max_yaw_diff;
    f.score = w[0] * norm_info - w[1] * norm_dist - w[2] * norm_yaw_diff;

    logger_.warn(
      "  Frontier [x {:.1f},y {:.1f}]: size={:.0f}, dist={:.2f} m, yaw diff={:.2f}, score={:.3f}",
      f.centroid.x, f.centroid.y,
      f.size,
      robot_pos.distanceTo(f.centroid),
      std::fabs(std::atan2(f.centroid.y - robot_pos.y, f.centroid.x - robot_pos.x) - robot_pos.yaw),
      f.score);
  }

  auto best_it = std::max_element(frontiers.begin(), frontiers.end(),
    [](const Frontier & a, const Frontier & b){ return a.score < b.score; });

  logger_.info("WFD: best frontier idx={}, size={:.0f}, score={:.3f}, centroid=({:.2f},{:.2f})",
    std::distance(frontiers.begin(), best_it),
    best_it->size, best_it->score,
    best_it->centroid.x, best_it->centroid.y);

  return *best_it;
}

}  // namespace wfd
