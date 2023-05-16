#include "tercom/geo/synthetic_terrain.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace TERCOM { namespace Geo {

    namespace {

        // Turn an integer grid coordinate (ix, iy) plus a seed into a
        // repeatable pseudo-random height in [-1, 1]. This is a hash, not a
        // random generator: the same inputs always give the same output, so the
        // terrain is reproducible and needs no stored state. The bit-mixing
        // (multiply by an odd constant, then xor with a high-bit shift) is the
        // SplitMix64 "avalanche" pattern, which scrambles the bits so that
        // changing one input bit flips about half the output bits. The magic
        // multipliers are the published SplitMix64 constants and are not
        // adjustable; they are chosen for good statistical mixing, not meaning:
        //   0x9E3779B97F4A7C15 is the 64-bit golden-ratio constant
        //     (2^64 / golden ratio), the standard SplitMix64 increment.
        //   0xBF58476D1CE4E5B9 and 0x94D049BB133111EB are the two SplitMix64
        //     final-mix multipliers.
        //   0xC2B2AE3D27D4EB4F is the Fmix64 (MurmurHash3 finaliser) multiplier,
        //     reused here to fold in the second coordinate.
        // The shift amounts (30, 27, 31) are the matching SplitMix64/Fmix64
        // shifts.
        double LatticeValue(std::uint64_t seed, std::int64_t ix, std::int64_t iy) {
            std::uint64_t h = seed;
            h ^= static_cast<std::uint64_t>(ix) * 0x9E3779B97F4A7C15ULL;
            h ^= h >> 30;
            h *= 0xBF58476D1CE4E5B9ULL;
            h ^= static_cast<std::uint64_t>(iy) * 0xC2B2AE3D27D4EB4FULL;
            h ^= h >> 27;
            h *= 0x94D049BB133111EBULL;
            h ^= h >> 31;
            // Take the top 53 bits (the number of bits a double can hold
            // exactly) and scale by 1 / 2^53 (9007199254740992) to land in
            // [0, 1), then stretch to [-1, 1).
            const double unit = static_cast<double>(h >> 11) * (1.0 / 9007199254740992.0);
            return (unit * 2.0) - 1.0;
        }

        // Smoothstep-style easing curve 6t^5 - 15t^4 + 10t^3 (Perlin's
        // "fade"). It maps a fraction in [0, 1] to a similar value but with zero
        // slope and curvature at both ends, so when noise cells are stitched
        // together the seams are smooth rather than showing a grid pattern.
        double Fade(double t) {
            return t * t * t * ((t * ((t * 6.0) - 15.0)) + 10.0);
        }

        // Linear interpolation: blend from a (t = 0) to b (t = 1).
        double Lerp(double a, double b, double t) {
            return a + ((b - a) * t);
        }

        // Value noise: pick a hashed random height at each integer lattice
        // corner, then smoothly blend between the four corners surrounding the
        // sample point. The result is a continuous, seemingly random field that
        // is the building block for one fBm octave.
        double ValueNoise(std::uint64_t seed, double x, double y) {
            const double fx = std::floor(x);
            const double fy = std::floor(y);
            const auto ix = static_cast<std::int64_t>(fx);
            const auto iy = static_cast<std::int64_t>(fy);
            const double tx = Fade(x - fx);
            const double ty = Fade(y - fy);

            // Heights at the four lattice corners around (x, y).
            const double v00 = LatticeValue(seed, ix, iy);
            const double v10 = LatticeValue(seed, ix + 1, iy);
            const double v01 = LatticeValue(seed, ix, iy + 1);
            const double v11 = LatticeValue(seed, ix + 1, iy + 1);

            // Bilinear blend: interpolate along x on the two rows, then
            // interpolate that pair along y, using the eased fractions.
            return Lerp(Lerp(v00, v10, tx), Lerp(v01, v11, tx), ty);
        }

    } // namespace

    HeightField GenerateFractalTerrain(const FractalTerrainConfiguration& configuration) {
        if (configuration.octaves < 1) {
            throw std::invalid_argument("fractal terrain requires at least one octave");
        }
        if (configuration.feature_wavelength_m <= 0.0) {
            throw std::invalid_argument("fractal terrain feature wavelength must be positive");
        }

        std::vector<float> elevations(configuration.cols * configuration.rows);

        for (std::size_t row = 0; row < configuration.rows; ++row) {
            const double north = configuration.origin_north_m + (static_cast<double>(row) * configuration.cell_size_m);
            for (std::size_t col = 0; col < configuration.cols; ++col) {
                const double east = configuration.origin_east_m + (static_cast<double>(col) * configuration.cell_size_m);

                // Accumulate the octaves. The first octave has amplitude 1 and
                // frequency 1 / feature_wavelength (so one cycle spans the
                // largest feature). normaliser tracks the total amplitude so the
                // sum can be rescaled back to roughly [-1, 1] regardless of how
                // many octaves were added.
                double sum = 0.0;
                double amplitude = 1.0;
                double frequency = 1.0 / configuration.feature_wavelength_m;
                double normaliser = 0.0;

                for (int octave = 0; octave < configuration.octaves; ++octave) {
                    // Give each octave a different seed so its noise pattern is
                    // independent of the others (otherwise they would line up
                    // and look artificial). The stride 0x100000001B3 is the
                    // 64-bit FNV prime, an arbitrary odd constant chosen only to
                    // spread the per-octave seeds apart.
                    constexpr std::uint64_t kOctaveSeedStride = 0x100000001B3ULL;
                    const std::uint64_t octave_seed = configuration.seed + (static_cast<std::uint64_t>(octave) * kOctaveSeedStride);
                    sum += amplitude * ValueNoise(octave_seed, east * frequency, north * frequency);
                    normaliser += amplitude;
                    // Next octave: smaller height (gain) and finer detail
                    // (lacunarity raises the frequency).
                    amplitude *= configuration.gain;
                    frequency *= configuration.lacunarity;
                }

                const double fbm = sum / normaliser; // approximately [-1, 1]
                // Centre on the base elevation and scale by half the relief, so
                // fbm = +1 reaches base + relief/2 and fbm = -1 reaches
                // base - relief/2, spanning the full peak-to-trough relief.
                const double elevation = configuration.base_elevation_m + (configuration.relief_m * 0.5 * fbm);
                elevations[(row * configuration.cols) + col] = static_cast<float>(elevation);
            }
        }

        return HeightField(configuration.cols, configuration.rows, configuration.cell_size_m, configuration.origin_east_m, configuration.origin_north_m, std::move(elevations));
    }

}} // namespace TERCOM::Geo
