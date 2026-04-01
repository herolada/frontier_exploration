#include "wfd_processor.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <map>
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
  const std::string & frame_id)
{
  OccupancyGrid grid;
  grid.width      = width;
  grid.height     = height;
  grid.resolution = resolution;
  grid.origin_x   = origin_x;
  grid.origin_y   = origin_y;
  grid.frame_id   = frame_id;
  grid.cells.resize(static_cast<size_t>(width * height));

  // // DEBUG: histogram of raw map values
  {
    std::map<int, int> hist;
    for (int i = 0; i < width * height; ++i) hist[raw[static_cast<size_t>(i)]]++;
    std::string out = "raw value histogram: ";
    for (auto & [val, cnt] : hist) out += std::to_string(val) + ":" + std::to_string(cnt) + " ";
    logger_.info("{}", out);
  }

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

  logger_.debug("KMeans split: {} cells, D={:.1f} m → {} cluster(s)", n_cells, D_m, n_r);

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
      logger_.debug("KMeans converged after {} iteration(s)", iter + 1);
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
//  detect  –  follows the WFD pseudocode from Keidar & Kaminka (2012)
//             Fig. 4, lines 1-30.
//
//  Notation mapping to the paper:
//    queue_m               → queue_m      (outer BFS, traversable free space)
//    queue_f               → queue_f      (inner BFS, one frontier component)
//    "Map-Open-List"       → map_open     (enqueued in queue_m)
//    "Map-Close-List"      → map_closed   (dequeued & processed by outer BFS)
//    "Frontier-Open-List"  → fron_open    (enqueued in queue_f)
//    "Frontier-Close-List" → fron_closed  (dequeued & processed by inner BFS)
//
//  Key reachability guarantee:
//    The outer BFS (queue_m) only enqueues TRAVERSABLE cells (lines 26-29).
//    Therefore every frontier component that is discovered is reachable from
//    the robot's starting cell through free space — obstacles and unexplored
//    cells are never expanded by the outer BFS.
//
//  Connectivity notes (matching common WFD practice):
//    - Outer BFS adj(p) : 4-connected  (lines 26-29)
//    - Inner BFS adj(q) : 8-connected  (lines 19-22) — ensures connected
//      frontier components along diagonal boundaries are captured in one
//      component rather than being fragmented.
//
//  Line 25 ("mark all points of NewFrontier as Map-Close-List"):
//    The paper marks every cell that the inner BFS *visited* (not just those
//    confirmed as frontier cells) so that they cannot seed a second inner BFS.
//    We therefore accumulate all cells dequeued from queue_f into
//    `inner_visited` and mark them map_closed after the inner loop.
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

  const std::size_t total = static_cast<std::size_t>(grid.width * grid.height);

  // Per-cell state flags corresponding to the paper's four marking sets.
  std::vector<bool> map_open   (total, false);  // "Map-Open-List"
  std::vector<bool> map_closed (total, false);  // "Map-Close-List"
  std::vector<bool> fron_open  (total, false);  // "Frontier-Open-List"
  std::vector<bool> fron_closed(total, false);  // "Frontier-Close-List"

  auto idx = [&](int c, int r) -> std::size_t {
    return static_cast<std::size_t>(r * grid.width + c);
  };
  auto inBounds = [&](int c, int r) -> bool {
    return c >= 0 && c < grid.width && r >= 0 && r < grid.height;
  };

  // Convert robot world position to nearest traversable grid cell.
  // (This is "pose" in line 2 of the pseudocode.)
  auto [robot_col, robot_row] = grid.worldToCell(robot_pos.x, robot_pos.y);

  if (grid.at(robot_col, robot_row) != CellState::TRAVERSABLE) {
    bool found = false;
    for (int radius = 1; radius <= 10 && !found; ++radius) {
      for (int dr = -radius; dr <= radius && !found; ++dr) {
        for (int dc = -radius; dc <= radius && !found; ++dc) {
          int c = robot_col + dc, r = robot_row + dr;
          if (inBounds(c, r) && grid.at(c, r) == CellState::TRAVERSABLE) {
            robot_col = c; robot_row = r; found = true;
          }
        }
      }
    }
    if (!found) {
      logger_.warn("WFD: robot cell is not traversable and no free neighbour found");
      return {};
    }
  }

  // ── Lines 1-3 ─────────────────────────────────────────────────────────
  // queue_m ← ∅
  // ENQUEUE(queue_m, pose)
  // mark pose as "Map-Open-List"
  std::queue<std::pair<int,int>> queue_m;
  queue_m.push({robot_col, robot_row});
  map_open[idx(robot_col, robot_row)] = true;

  int first_few_iterations = 10;

  std::vector<Frontier> frontiers;

  // ── Line 4 ────────────────────────────────────────────────────────────
  // while queue_m is not empty do
  while (!queue_m.empty()) {

    // Line 5: p ← DEQUEUE(queue_m)
    auto [p_col, p_row] = queue_m.front();
    queue_m.pop();

    // Lines 6-7: if p is marked as "Map-Close-List" then continue
    if (map_closed[idx(p_col, p_row)]) continue;

    // Line 8: if p is a frontier point then
    if (isFrontierCell(grid, p_col, p_row)) {

      // Lines 9-12:
      // queue_f ← ∅
      // NewFrontier ← ∅
      // ENQUEUE(queue_f, p)
      // mark p as "Frontier-Open-List"
      std::queue<std::pair<int,int>> queue_f;
      std::vector<Pose2D>           new_frontier_cells;

      // All cells dequeued from queue_f this iteration — needed for line 25.
      std::vector<std::size_t>       inner_visited;

      queue_f.push({p_col, p_row});
      fron_open[idx(p_col, p_row)] = true;

      // ── Line 13 ───────────────────────────────────────────────────────
      // while queue_f is not empty do
      while (!queue_f.empty()) {

        // Line 14: q ← DEQUEUE(queue_f)
        auto [q_col, q_row] = queue_f.front();
        queue_f.pop();
        const std::size_t qi = idx(q_col, q_row);

        // Lines 15-16: if q is marked as {"Map-Close-List","Frontier-Close-List"} then continue
        if (map_closed[qi] || fron_closed[qi]) continue;

        // Track every cell the inner BFS processes (for line 25 below).
        inner_visited.push_back(qi);

        // Line 17: if q is a frontier point then
        if (isFrontierCell(grid, q_col, q_row)) {

          // Line 18: add q to NewFrontier
          new_frontier_cells.push_back(grid.cellToWorld(q_col, q_row));

          // Lines 19-22: for all w ∈ adj(q) do
          //   (8-connected — captures diagonally-adjacent frontier cells
          //    in the same component)
          //   if w not marked as {"Frontier-Open-List","Frontier-Close-List",
          //                       "Map-Close-List"} then
          //     ENQUEUE(queue_f, w)
          //     mark w as "Frontier-Open-List"
          for (int d = 0; d < 8; ++d) {
            int wc = q_col + kDx8[d];
            int wr = q_row + kDy8[d];
            if (!inBounds(wc, wr)) continue;
            const std::size_t wi = idx(wc, wr);
            if (!fron_open[wi] && !fron_closed[wi] && !map_closed[wi]) {
              queue_f.push({wc, wr});
              fron_open[wi] = true;
            }
          }
        }

        // Line 23: mark q as "Frontier-Close-List"
        fron_closed[qi] = true;

      }  // end while queue_f  (line 13)

      // Line 24: save data of NewFrontier
      // Apply k-means splitting and store resulting sub-frontiers.
      if (static_cast<int>(new_frontier_cells.size()) >= params_.min_frontier_size) {
        Frontier f;
        f.cells    = new_frontier_cells;
        f.centroid = centroid(f.cells);
        f.size     = static_cast<double>(f.cells.size());

        auto parts = splitFrontierKMeans(f);
        for (auto & part : parts) {
          frontiers.push_back(std::move(part));
        }
      }

      // Line 25: mark all points of NewFrontier as "Map-Close-List"
      // We mark every cell the inner BFS visited (inner_visited), not only
      // the confirmed frontier cells.  This matches the paper's intent: once
      // a frontier component has been extracted, none of its constituent cells
      // should trigger a second inner BFS from the outer loop.
      for (const std::size_t ci : inner_visited) {
        map_closed[ci] = true;
      }

    }  // end if frontier point  (line 8)

    // Lines 26-29: for all v ∈ adj(p) do
    //   (4-connected outer BFS — walks through free space only)
    //   if v not marked as {"Map-Open-List","Map-Close-List"}
    //   and v has at least one "Map-Open-Space" neighbour then
    //     ENQUEUE(queue_m, v)
    //     mark v as "Map-Open-List"
    //
    // "has at least one Map-Open-Space neighbour" in the paper is the
    // condition that keeps the outer BFS in known free space.  Interpreting
    // "Map-Open-Space" as TRAVERSABLE, the simplest equivalent is: only
    // enqueue v if v itself is TRAVERSABLE.  A TRAVERSABLE cell always
    // neighbours at least one TRAVERSABLE cell (itself counts via the
    // isFrontierCell check, and its traversability is the criterion), which
    // satisfies the paper's neighbour condition.
    for (int d = 0; d < 4; ++d) {
      int vc = p_col + kDx[d];
      int vr = p_row + kDy[d];
      if (!inBounds(vc, vr)) continue;
      const std::size_t vi = idx(vc, vr);
      
      if (first_few_iterations > 0 && fron_closed[vi]) { // this helps avoid case where the robot starts in a frontier and cannot explore further
        map_closed[vi] = false;
      }
      
      if (map_open[vi] || map_closed[vi]) {
          continue;
      }

      // Only expand free/traversable cells (reachability guarantee).
      if (grid.at(vc, vr) != CellState::TRAVERSABLE) continue;

      queue_m.push({vc, vr});
      map_open[vi] = true;
    }
    --first_few_iterations;

    // Line 30: mark p as "Map-Close-List"
    map_closed[idx(p_col, p_row)] = true;

  }  // end while queue_m  (line 4)

  auto t_end = std::chrono::steady_clock::now();
  double ms  = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  logger_.info("WFD: found {} frontier(s) in {:.1f} ms (grid {}x{}, robot cell [{},{}])",
    frontiers.size(), ms, grid.width, grid.height, robot_col, robot_row);

  return frontiers;
}

// ============================================================
//  selectBest
// ============================================================

// std::optional<Frontier> WFDProcessor::selectBest(
//   std::vector<Frontier> & frontiers,
//   const Pose2D & robot_pos)
// {
//   return selectBestImpl(frontiers, robot_pos, nullptr);
// }

// std::optional<Frontier> WFDProcessor::selectBest(
//   std::vector<Frontier> & frontiers,
//   const Pose2D & robot_pos,
//   const Pose2D & center_pose)
// {
//   return selectBestImpl(frontiers, robot_pos, &center_pose);
// }

// std::optional<Frontier> WFDProcessor::selectBestImpl(
//   std::vector<Frontier> & frontiers,
//   const Pose2D & robot_pos,
//   const Pose2D * center_pose)
// }

void WFDProcessor::get_near_occupancy_degree(
  Frontier & f,
  const OccupancyGrid & grid)
{
  // int k = 0;
  auto [col, row] = grid.worldToCell(f.centroid.x, f.centroid.y);
  int how_many_neighbors = static_cast<size_t>(std::round(params_.frontier_near_occupancy_distance/grid.resolution));
  // logger_.warn(
  //           "neighb {}, occ dist, {}, reso {}", how_many_neighbors, params_.frontier_near_occupancy_distance, grid.resolution);
  if (how_many_neighbors > 0) {
    for (int i{1}; i < how_many_neighbors+1; ++i) {
      // if (grid.at(col, row+i) == CellState::OBSTACLE) {
      //   f.nearby_occupancy_degree += 1.; // linear scaling based on manhattan distance
      // }
      // if (grid.at(col, row-i) == CellState::OBSTACLE) {
      //   f.nearby_occupancy_degree += 1.; // linear scaling based on manhattan distance
      // }
      for (int j{-i}; j < i; ++j) {
        // logger_.warn(
        //     "{},{}", i,j);
        // logger_.warn(
        //     "{},{}   {},{}   {},{}   {},{}", col+i, row+j+1, col-i, row+j, col+j, row+i, col+j+1, row-i);
        // k+=4;
        if (grid.at(col+i, row+j+1) == CellState::OBSTACLE) {
          f.nearby_occupancy_degree += 1./i; // linear scaling based on manhattan distance
        }
        if (grid.at(col-i, row+j) == CellState::OBSTACLE) {
          // logger_.warn(
          //   "HIT");
          f.nearby_occupancy_degree += 1./i; // linear scaling based on manhattan distance
        }
        if (grid.at(col+j, row+i) == CellState::OBSTACLE) {
          f.nearby_occupancy_degree += 1./i; // linear scaling based on manhattan distance
        }
        if (grid.at(col+j+1, row-i) == CellState::OBSTACLE) {
          f.nearby_occupancy_degree += 1./i; // linear scaling based on manhattan distance
        }
        // logger_.warn(
        // "Cell [{}, {}] state {}",
        // col-i, row+j,
        // static_cast<int>(grid.at(col-i, row+j)));
      }
    }
  }
  f.nearby_occupancy_degree /= (8. * how_many_neighbors); // this is a pretty cool way to normalize the degree to <0,1>!
  // logger_.warn(
  //       "Frontier [{}, {}] occ_deg {:.4f}, neighbors checked: {}, total {}",
  //       col,row,
  //       f.nearby_occupancy_degree,
  //       how_many_neighbors, k);
}

std::optional<Frontier> WFDProcessor::selectBest(
  std::vector<Frontier> & frontiers,
  OccupancyGrid & grid,
  const Pose2D & robot_pos,
  const std::optional<Pose2D> & center_pose)
{
  if (frontiers.empty()) return std::nullopt;

  // Normalisation factors
  double max_info = 0.0;
  double max_dist = 0.0;
  double max_yaw_diff = 3.14;
  double max_center_dist = 0.0;
  double max_occ_deg = 1.0;
  
  for (auto & f : frontiers) {
    get_near_occupancy_degree(f, grid);
    max_info = std::max(max_info, f.size); // TODO replace this with something better than information gain == number of frontier cells connected to this component.
    max_dist = std::max(max_dist, robot_pos.distanceTo(f.centroid));
    // max_yaw_diff = std::max(max_yaw_diff, std::fabs(std::atan2(f.centroid.y - robot_pos.y, f.centroid.x - robot_pos.x) - robot_pos.yaw));
    if (center_pose) {
      max_center_dist = std::max(max_center_dist, center_pose->distanceTo(f.centroid));
    }
  }


  if (max_info < 1e-9) max_info = 1.0;
  if (max_dist < 1e-9) max_dist = 1.0;
  // if (max_yaw_diff < 1e-9) max_yaw_diff = 1.0;
  if (max_center_dist < 1e-9) max_center_dist = 1.0;

  const std::vector<double> w = params_.weights;
  const double exp = params_.info_gain_exponent;

  for (auto & f : frontiers) {
    double dist = robot_pos.distanceTo(f.centroid);
    if (dist < params_.min_frontier_dist) {
      continue;
    }
    double norm_info = std::pow(f.size / max_info, exp);
    double norm_dist = dist / max_dist;
    double norm_yaw_diff = std::fabs(std::atan2(f.centroid.y - robot_pos.y, f.centroid.x - robot_pos.x) - robot_pos.yaw) / max_yaw_diff;
    double norm_occ_deg = f.nearby_occupancy_degree / max_occ_deg;
    f.score = w[0] * norm_info - w[1] * norm_dist - w[2] * norm_yaw_diff - w[4] * norm_occ_deg;

    if (center_pose) {
      double norm_center_dist = center_pose->distanceTo(f.centroid) / max_center_dist;
      f.score -= w[3] * norm_center_dist;
      logger_.warn(
        "  Frontier [x {:.1f},y {:.1f}]: size={:.0f}, dist={:.2f} m, yaw diff={:.2f}, center_dist={:.2f}, occ_deg={:.3f}, score={:.3f}",
        f.centroid.x, f.centroid.y,
        f.size, dist,
        std::fabs(std::atan2(f.centroid.y - robot_pos.y, f.centroid.x - robot_pos.x) - robot_pos.yaw),
        center_pose->distanceTo(f.centroid),
        f.nearby_occupancy_degree,
        f.score);
    } else {
      logger_.warn(
        "  Frontier [x {:.1f},y {:.1f}]: size={:.0f}, dist={:.2f} m, yaw diff={:.2f}, occ_deg={:.3f}, score={:.3f}",
        f.centroid.x, f.centroid.y,
        f.size, dist,
        std::fabs(std::atan2(f.centroid.y - robot_pos.y, f.centroid.x - robot_pos.x) - robot_pos.yaw),
        f.nearby_occupancy_degree,
        f.score);
    }
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
