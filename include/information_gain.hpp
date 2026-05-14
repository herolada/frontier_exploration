// information_gain.hpp
#pragma once
#include <cmath>
#include <cstdint>
#include "wfd_types.hpp"   // OccupancyGrid, Frontier, Pose2D, CellState

namespace wfd
{

/// Approximate information gain for a LiDAR placed at `pose`
/// by casting NUM_RAYS rays out to max_range_m metres.
/// Returns the total count of UNKNOWN cells seen before each ray hits
/// an obstacle or the grid boundary.
double approximateInfoGain(
    ROSLogger*           logger,
    const Pose2D&        pose,
    const OccupancyGrid& grid,
    double               max_range_m = 12.0,
    int                  num_rays    = 32)
{
    if (!grid.valid()) return 0;

    // Convert sensor origin to cell coordinates (fractional is fine here).
    const double ox = (pose.x - grid.origin_x) / grid.resolution;
    const double oy = (pose.y - grid.origin_y) / grid.resolution;

    
    // Maximum ray length in cells.
    const int max_cells = static_cast<int>(max_range_m / grid.resolution);

    // No double dips.
    int square_side = 2 * max_cells + 1;
    std::vector<bool> visited(square_side * square_side, false);
    int visitable = 0;
    
    int info_gain = 0;

    for (int r = 0; r < num_rays; ++r)
    {
        const double angle = (2.0 * M_PI * r) / num_rays;
        const double dx    = std::cos(angle);
        const double dy    = std::sin(angle);

        // ---- Bresenham setup ------------------------------------------------
        // End point (clamped later via bounds check inside the loop).
        int x0 = static_cast<int>(ox);
        int y0 = static_cast<int>(oy);
        int x1 = static_cast<int>(ox + dx * max_cells);
        int y1 = static_cast<int>(oy + dy * max_cells);

        int sx = (x1 >= x0) ? 1 : -1;
        int sy = (y1 >= y0) ? 1 : -1;
        int ax = std::abs(x1 - x0);
        int ay = std::abs(y1 - y0);

        int x = x0, y = y0;

        if (ax >= ay)
        {
            // X-major
            int err = ax / 2;
            for (int i = 0; i <= ax; ++i)
            {   
                int vi = x-x0+max_cells + (y-y0+max_cells)*square_side;
                
                // delete this if everything works:
                if(!(vi >= 0 && vi < square_side * square_side)) {
                    logger->error("Adame mas tam chybu {}.", vi);
                }

                CellState s = grid.at(x, y);
                if (s == CellState::OBSTACLE) {
                    visitable += ax + 1 - i;
                    break;
                }
                if (!visited[vi]) {
                    ++visitable;
                    visited[vi] = true;
                    if (s == CellState::UNEXPLORED) {
                        ++info_gain;
                    }
                }   
                err -= ay;
                if (err < 0) { y += sy; err += ax; }
                x += sx;
            }
        }
        else
        {
            // Y-major
            int err = ay / 2;
            for (int i = 0; i <= ay; ++i)
            {
                int vi = x-x0+max_cells + (y-y0+max_cells)*square_side;
                
                // delete this if everything works:
                if(!(vi >= 0 && vi < square_side * square_side)) {
                    logger->error("Adame mas tam chybu {}.", vi);
                }

                CellState s = grid.at(x, y);
                if (s == CellState::OBSTACLE) {
                    visitable += ay + 1 - i;
                    break;
                }
                if (!visited[vi]) {
                    ++visitable;
                    visited[vi] = true;
                    if (s == CellState::UNEXPLORED) {
                        ++info_gain;
                    }
                }   
                err -= ax;
                if (err < 0) { x += sx; err += ay; }
                y += sy;
            }
        }
    }

    return visitable>0 ? static_cast<double>(info_gain)/visitable : 0.0;
}
} // namespace wfd