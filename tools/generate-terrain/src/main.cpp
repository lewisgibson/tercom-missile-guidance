// generate-terrain: synthesise a procedural fractal digital elevation model and
// write it as an Esri/USGS ASCII grid (.asc). Useful for producing reproducible
// terrain for the simulator, the examples, and the test fixtures.
//
// Run with --help (or see PrintUsage below, the single source of truth) for the
// full option list and defaults.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include "tercom/geo/ascii_grid.h"
#include "tercom/geo/synthetic_terrain.h"

namespace {

    void PrintUsage() {
        std::puts("Usage: generate-terrain [options]");
        std::puts("Options (all optional; defaults in brackets):");
        std::puts("  --out PATH       output .asc path [terrain.asc; \"-\" writes to stdout]");
        std::puts("  --cols N         grid columns [512]");
        std::puts("  --rows N         grid rows [512]");
        std::puts("  --cell METRES    node spacing [100]");
        std::puts("  --base METRES    mean elevation [500]");
        std::puts("  --relief METRES  peak-to-trough relief [700]");
        std::puts("  --wavelength M   largest feature wavelength [12000]");
        std::puts("  --octaves N      fractal octaves [6]");
        std::puts("  --seed N         noise seed [unsigned, default library seed]");
        std::puts("  --help           print this help");
    }

    // Fetch the value following a flag, or exit with a clear error.
    const char* RequireValue(int argc, char** argv, int& index, const char* flag) {
        if (index + 1 >= argc) {
            std::fprintf(stderr, "error: %s requires a value\n", flag);
            // Single-threaded command-line tool: exiting on a usage error is fine.
            // NOLINTNEXTLINE(concurrency-mt-unsafe)
            std::exit(2);
        }
        return argv[++index];
    }

} // namespace

int main(int argc, char** argv) {
    using namespace TERCOM;

    Geo::FractalTerrainConfiguration parameters;
    std::string out_path = "terrain.asc";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            PrintUsage();
            return 0;
        }
        if (arg == "--out") {
            out_path = RequireValue(argc, argv, i, "--out");
        } else if (arg == "--cols") {
            parameters.cols = static_cast<std::size_t>(std::strtoull(RequireValue(argc, argv, i, "--cols"), nullptr, 10));
        } else if (arg == "--rows") {
            parameters.rows = static_cast<std::size_t>(std::strtoull(RequireValue(argc, argv, i, "--rows"), nullptr, 10));
        } else if (arg == "--cell") {
            parameters.cell_size_m = std::strtod(RequireValue(argc, argv, i, "--cell"), nullptr);
        } else if (arg == "--base") {
            parameters.base_elevation_m = std::strtod(RequireValue(argc, argv, i, "--base"), nullptr);
        } else if (arg == "--relief") {
            parameters.relief_m = std::strtod(RequireValue(argc, argv, i, "--relief"), nullptr);
        } else if (arg == "--wavelength") {
            parameters.feature_wavelength_m = std::strtod(RequireValue(argc, argv, i, "--wavelength"), nullptr);
        } else if (arg == "--octaves") {
            parameters.octaves = static_cast<int>(std::strtol(RequireValue(argc, argv, i, "--octaves"), nullptr, 10));
        } else if (arg == "--seed") {
            parameters.seed = static_cast<std::uint64_t>(std::strtoull(RequireValue(argc, argv, i, "--seed"), nullptr, 10));
        } else {
            std::fprintf(stderr, "error: unknown argument '%s'\n", arg.c_str());
            PrintUsage();
            return 2;
        }
    }

    try {
        const Geo::HeightField field = Geo::GenerateFractalTerrain(parameters);
        if (out_path == "-") {
            Geo::WriteEsriAsciiGrid(std::cout, field);
        } else {
            Geo::WriteEsriAsciiGrid(out_path, field);
            std::fprintf(stderr, "wrote %zu x %zu grid (%.0f m posts) to %s\n", field.Cols(), field.Rows(), field.CellSizeM(), out_path.c_str());
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
    return 0;
}
