#pragma once

// Non-ROS helpers for turning an MGRS polygon file into ECEF points.
// Depends only on GeographicLib and the AbstractLogger interface.

#include "abstract_logger.hpp"

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geocentric.hpp>

#include <array>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace wfd
{

/**
 * @brief Parse a file of MGRS coordinates (one per line, '#' comments allowed)
 *        and convert each to an ECEF point.
 * @return ECEF points (>= 3) on success, std::nullopt on any parse/IO error.
 */
inline std::optional<std::vector<std::array<double, 3>>>
parseMGRSFile(const std::string & path, AbstractLogger & logger)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    logger.error("Cannot open polygon file '{}'", path);
    return std::nullopt;
  }

  std::vector<std::array<double, 3>> ecef_points;
  std::string line;
  while (std::getline(file, line)) {
    line.erase(0, line.find_first_not_of(" \t\r\n"));
    if (line.empty() || line[0] == '#') continue;
    line.erase(line.find_last_not_of(" \t\r\n") + 1);

    try {
      int zone;
      bool northp;
      double x, y;
      int prec;
      GeographicLib::MGRS::Reverse(line, zone, northp, x, y, prec);

      double lat, lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, x, y, lat, lon);

      double X, Y, Z;
      GeographicLib::Geocentric::WGS84().Forward(lat, lon, 0.0, X, Y, Z);

      ecef_points.push_back({X, Y, Z});
      logger.info("  MGRS '{}' -> lat={:.6f} lon={:.6f} -> ECEF ({:.0f}, {:.0f}, {:.0f})",
        line, lat, lon, X, Y, Z);
    } catch (const std::exception & e) {
      logger.error("Failed to parse MGRS line '{}': {}", line, e.what());
      return std::nullopt;
    }
  }

  if (ecef_points.size() < 3) {
    logger.error("Polygon file '{}' has fewer than 3 valid points (got {})", path, ecef_points.size());
    return std::nullopt;
  }

  return ecef_points;
}

}  // namespace wfd
