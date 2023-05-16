#pragma once

// Deterministic procedural terrain for tests and the simulator. It builds a
// fake but plausible heightmap by summing several layers of smooth random
// "noise" at doubling frequencies -- a technique called fractional Brownian
// motion (fBm). Each layer (octave) is half the amplitude and twice the
// frequency of the one before, so big rolling hills get progressively finer
// detail added on top. The randomness comes from hashing the integer grid
// coordinates rather than from a global random generator, so the same seed
// always produces exactly the same terrain and runs are fully reproducible.

#include <cstddef>
#include <cstdint>

#include "tercom/geo/height_field.h"

namespace TERCOM { namespace Geo {

    struct FractalTerrainConfiguration {
        // Grid size in nodes. 512 x 512 nodes at 100 m spacing covers about a
        // 51 km square, a convenient default for a test scene.
        std::size_t cols = 512;
        std::size_t rows = 512;
        // Spacing between grid nodes in metres along both axes.
        double cell_size_m = 100.0;
        // ENU position of node (0, 0); both default to the local origin.
        double origin_east_m = 0.0;
        double origin_north_m = 0.0;

        // Mean (average) terrain elevation in metres, and the total peak-to-
        // trough relief in metres swung about that mean. Defaults give terrain
        // ranging roughly 150 m to 850 m above the reference, i.e. 500 +/- 350.
        double base_elevation_m = 500.0;
        double relief_m = 700.0;

        // Wavelength in metres of the largest (lowest-frequency) terrain
        // feature: how far apart the biggest hills sit. Finer octaves add
        // smaller detail on top.
        double feature_wavelength_m = 12000.0;
        // Number of noise layers summed. More octaves means more fine detail at
        // more cost; 6 is a good visual/performance balance.
        int octaves = 6;
        // Lacunarity: the frequency multiplier between successive octaves. 2.0
        // means each octave has half the wavelength of the previous one, the
        // conventional choice for fBm.
        double lacunarity = 2.0;
        // Gain (persistence): the amplitude multiplier between successive
        // octaves. 0.5 means each finer octave contributes half as much height
        // as the previous, so detail fades out smoothly; this is the standard
        // fBm value and gives natural-looking, not too rough, terrain.
        double gain = 0.5;

        // Seed for the coordinate hash. Any fixed non-zero value works; this
        // arbitrary constant just picks one particular reproducible landscape.
        std::uint64_t seed = 0x7E44017A11D1ULL;
    };

    HeightField GenerateFractalTerrain(const FractalTerrainConfiguration& configuration);

}} // namespace TERCOM::Geo
