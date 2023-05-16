#pragma once
#include <string>

#include "./radar/position.h"

#include "./terrain/contour_map.h"

#include "./trajectory/correction.h"
#include "./trajectory/target.h"

namespace TERCOM {
    class GuidanceSystem {
      private:
        TERCOM::Terrain::ContourMap& contour_map;

      public:
        explicit GuidanceSystem(std::string& contour_map_file_path, TERCOM::Radar::Position& location, TERCOM::Trajectory::Target& target);

        void SetTargetPosition(TERCOM::Trajectory::Target& target);

        TERCOM::Trajectory::Correction& CalculateCorrections(TERCOM::Radar::Position& location);
    };
} // namespace TERCOM
