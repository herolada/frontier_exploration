#include <vector>
#include "wfd_types.hpp"
#include <algorithm>

namespace wfd
{
// Function to check if a point is inside a polygon using
// the ray-casting algorithm
// https://www.geeksforgeeks.org/cpp/point-in-polygon-in-cpp/
bool isPointInPolygon(const std::vector<Pose2D>& polygon,
                      const Pose2D& point)
{
    // Number of vertices in the polygon
    int n = polygon.size();
    // Count of intersections
    int count = 0;

    // Iterate through each edge of the polygon
    for (int i = 0; i < n; i++) {
        const Pose2D& p1 = polygon[i];
        // Ensure the last point connects to the first point
        const Pose2D& p2 = polygon[(i + 1) % n];

        // Check if the point's y-coordinate is within the
        // edge's y-range and if the point is to the left of
        // the edge
        if ((point.y > std::min(p1.y, p2.y))
            && (point.y <= std::max(p1.y, p2.y))
            && (point.x <= std::max(p1.x, p2.x))) {
            // Calculate the x-coordinate of the
            // intersection of the edge with a horizontal
            // line through the point
            double xIntersect = (point.y - p1.y)
                                    * (p2.x - p1.x)
                                    / (p2.y - p1.y)
                                + p1.x;
            // If the edge is vertical or the point's
            // x-coordinate is less than or equal to the
            // intersection x-coordinate, increment count
            if (p1.x == p2.x || point.x <= xIntersect) {
                count++;
            }
        }
    }
    // If the number of intersections is odd, the point is
    // inside the polygon
    return count % 2 == 1;
}

void remove_frontiers_outside_polygon(std::vector<wfd::Pose2D> &polygon, std::vector<Frontier> &frontiers)
{
    frontiers.erase(
        std::remove_if(frontiers.begin(), frontiers.end(),
            [&polygon](const auto& frontier) {
                return !isPointInPolygon(polygon, frontier.centroid);
            }),
        frontiers.end()
    );
}

}