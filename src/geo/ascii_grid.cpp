#include "tercom/geo/ascii_grid.h"

#include <cctype>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace TERCOM { namespace Geo {

    namespace {
        std::string ToLower(std::string value) {
            for (char& ch : value) {
                ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            }
            return value;
        }
    } // namespace

    HeightField LoadEsriAsciiGrid(std::istream& input) {
        std::size_t cols = 0;
        std::size_t rows = 0;
        double cell_size = 0.0;
        double lower_left_x = 0.0;
        double lower_left_y = 0.0;
        bool x_is_centre = false;
        bool y_is_centre = false;
        bool have_cols = false;
        bool have_rows = false;
        bool have_cell = false;
        bool have_x = false;
        bool have_y = false;
        // The sentinel value the file uses for void (no-data) cells, if it declares
        // one. HeightField has no representation for voids, so a grid that actually
        // contains them is rejected below rather than silently treated as terrain.
        double nodata_value = -9999.0;
        bool have_nodata = false;

        // Header lines precede the raster. A header line starts with an
        // alphabetic key (such as "ncols"); the first token that begins with a
        // digit or sign marks the start of the elevation grid. So skip leading
        // whitespace and peek at the next character: a letter means another
        // header line, anything else means the data has begun.
        while (true) {
            input >> std::ws;
            const int next = input.peek();
            if (next == std::char_traits<char>::eof()) {
                throw std::runtime_error("ASCII grid ended before any elevation data");
            }
            if (std::isalpha(next) == 0) {
                break;
            }

            std::string key;
            input >> key;
            const std::string lower = ToLower(key);
            if (lower == "ncols") {
                input >> cols;
                have_cols = true;
            } else if (lower == "nrows") {
                input >> rows;
                have_rows = true;
            } else if (lower == "xllcorner") {
                input >> lower_left_x;
                x_is_centre = false;
                have_x = true;
            } else if (lower == "xllcenter") {
                input >> lower_left_x;
                x_is_centre = true;
                have_x = true;
            } else if (lower == "yllcorner") {
                input >> lower_left_y;
                y_is_centre = false;
                have_y = true;
            } else if (lower == "yllcenter") {
                input >> lower_left_y;
                y_is_centre = true;
                have_y = true;
            } else if (lower == "cellsize") {
                input >> cell_size;
                have_cell = true;
            } else if (lower == "nodata_value") {
                input >> nodata_value;
                have_nodata = true;
            } else {
                throw std::runtime_error("unknown ASCII grid header key: " + key);
            }

            if (!input) {
                throw std::runtime_error("malformed ASCII grid header near key: " + key);
            }
        }

        if (!have_cols || !have_rows || !have_cell || !have_x || !have_y) {
            throw std::runtime_error("ASCII grid header is missing a required field");
        }
        if (cols < 2 || rows < 2) {
            throw std::runtime_error("ASCII grid must be at least 2x2");
        }

        const std::size_t count = cols * rows;
        std::vector<float> file_order(count);
        for (std::size_t i = 0; i < count; ++i) {
            double value = 0.0;
            input >> value;
            if (!input) {
                throw std::runtime_error("ASCII grid has fewer values than ncols * nrows");
            }
            // Reject void cells: HeightField stores a dense grid of real elevations
            // with no way to mark a hole, so silently keeping the sentinel (often
            // -9999) would inject a cliff of bogus terrain into the reference map.
            if (have_nodata && std::fabs(value - nodata_value) < 1e-6) {
                throw std::runtime_error("ASCII grid contains NODATA (void) cells, which HeightField cannot represent");
            }
            file_order[i] = static_cast<float>(value);
        }

        // The file lists the northern row first; HeightField wants row 0 to be
        // the southernmost, so reverse the row order while copying.
        std::vector<float> elevations(count);
        for (std::size_t row = 0; row < rows; ++row) {
            const std::size_t source_row = rows - 1 - row;
            for (std::size_t col = 0; col < cols; ++col) {
                elevations[(row * cols) + col] = file_order[(source_row * cols) + col];
            }
        }

        // The header can give the reference point as either a cell corner
        // (xllcorner/yllcorner, the outer edge of the south-west cell) or a cell
        // centre (xllcenter/yllcenter). HeightField nodes sit at cell centres,
        // so when corners are supplied, shift inward by half a cell to land on
        // the centre of the south-west cell.
        const double origin_east = x_is_centre ? lower_left_x : lower_left_x + (cell_size / 2.0);
        const double origin_north = y_is_centre ? lower_left_y : lower_left_y + (cell_size / 2.0);

        return HeightField(cols, rows, cell_size, origin_east, origin_north, std::move(elevations));
    }

    HeightField LoadEsriAsciiGrid(const std::string& path) {
        std::ifstream file(path);
        if (!file) {
            throw std::runtime_error("could not open ASCII grid: " + path);
        }
        return LoadEsriAsciiGrid(file);
    }

    void WriteEsriAsciiGrid(std::ostream& output, const HeightField& field, double nodata_value) {
        // Write the origin as cell centres (xllcenter/yllcenter) because that is
        // exactly what HeightField stores; this avoids the half-cell shift on a
        // later reload.
        output << "ncols " << field.Cols() << '\n'
               << "nrows " << field.Rows() << '\n'
               << "xllcenter " << field.OriginEastM() << '\n'
               << "yllcenter " << field.OriginNorthM() << '\n'
               << "cellsize " << field.CellSizeM() << '\n'
               << "NODATA_value " << nodata_value << '\n';

        // Emit the northern row first to match the format convention.
        for (std::size_t row = 0; row < field.Rows(); ++row) {
            const std::size_t source_row = field.Rows() - 1 - row;
            for (std::size_t col = 0; col < field.Cols(); ++col) {
                if (col != 0) {
                    output << ' ';
                }
                output << field.NodeElevation(col, source_row);
            }
            output << '\n';
        }
    }

    void WriteEsriAsciiGrid(const std::string& path, const HeightField& field, double nodata_value) {
        std::ofstream file(path);
        if (!file) {
            throw std::runtime_error("could not open ASCII grid for writing: " + path);
        }
        WriteEsriAsciiGrid(file, field, nodata_value);
    }

}} // namespace TERCOM::Geo
