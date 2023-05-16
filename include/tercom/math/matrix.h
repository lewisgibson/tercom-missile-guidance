#pragma once

// Compact fixed-size dense matrix for the navigation filter. Dimensions are
// template parameters so every product is dimension-checked at compile time.
// Row-major storage. Deliberately dependency-free -- the Kalman filter needs
// only multiply, transpose, add, and a small square inverse.

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace TERCOM { namespace Math {

    // Rows and Cols are compile-time template parameters, so the dimensions are
    // part of the type. Multiplying mismatched shapes simply fails to compile,
    // which catches a whole class of filter bugs before the program runs.
    template <std::size_t Rows, std::size_t Cols>
    struct Matrix {
        // Row-major storage: element (row, col) lives at index row*Cols + col.
        std::array<double, Rows * Cols> data{};

        // Mutable element access by (row, col).
        double& operator()(std::size_t row, std::size_t col) {
            return data[(row * Cols) + col];
        }

        // Read-only element access by (row, col).
        double operator()(std::size_t row, std::size_t col) const {
            return data[(row * Cols) + col];
        }

        // All-zeros matrix (the array is value-initialised to 0.0).
        static Matrix Zero() {
            return Matrix{};
        }

        // Identity matrix: 1.0 on the diagonal, 0.0 elsewhere. Multiplying by it
        // leaves a matrix unchanged. Only meaningful for square matrices, which
        // the static_assert enforces at compile time.
        static Matrix Identity() {
            static_assert(Rows == Cols, "Identity requires a square matrix");
            Matrix result{};
            for (std::size_t i = 0; i < Rows; ++i) {
                result(i, i) = 1.0;
            }
            return result;
        }

        Matrix operator+(const Matrix& other) const {
            Matrix result{};
            for (std::size_t i = 0; i < Rows * Cols; ++i) {
                result.data[i] = data[i] + other.data[i];
            }
            return result;
        }

        Matrix operator-(const Matrix& other) const {
            Matrix result{};
            for (std::size_t i = 0; i < Rows * Cols; ++i) {
                result.data[i] = data[i] - other.data[i];
            }
            return result;
        }

        // Transpose: swap rows and columns, so a Rows-by-Cols matrix becomes
        // Cols-by-Rows. The Kalman update needs this for terms such as H * P * H'.
        Matrix<Cols, Rows> Transposed() const {
            Matrix<Cols, Rows> result{};
            for (std::size_t r = 0; r < Rows; ++r) {
                for (std::size_t c = 0; c < Cols; ++c) {
                    result(c, r) = (*this)(r, c);
                }
            }
            return result;
        }
    };

    // Matrix product. The shared Inner dimension must match (a is Rows-by-Inner,
    // b is Inner-by-Cols), which the template signature guarantees. The loop is
    // ordered r, k, c so the inner write is over contiguous result columns.
    template <std::size_t Rows, std::size_t Inner, std::size_t Cols>
    Matrix<Rows, Cols> operator*(const Matrix<Rows, Inner>& a, const Matrix<Inner, Cols>& b) {
        Matrix<Rows, Cols> result{};
        for (std::size_t r = 0; r < Rows; ++r) {
            for (std::size_t k = 0; k < Inner; ++k) {
                const double a_rk = a(r, k);
                for (std::size_t c = 0; c < Cols; ++c) {
                    result(r, c) += a_rk * b(k, c);
                }
            }
        }
        return result;
    }

    // Scalar times matrix: multiply every element by the scalar.
    template <std::size_t Rows, std::size_t Cols>
    Matrix<Rows, Cols> operator*(double scalar, const Matrix<Rows, Cols>& m) {
        Matrix<Rows, Cols> result{};
        for (std::size_t i = 0; i < Rows * Cols; ++i) {
            result.data[i] = scalar * m.data[i];
        }
        return result;
    }

    // Matrix inverse by Gauss-Jordan elimination with partial pivoting. The idea:
    // place the input and an identity side by side, then apply row operations
    // that turn the input into the identity; the same operations turn the
    // identity into the inverse. "work" holds the input being reduced and
    // "result" holds the identity that becomes the answer. Throws if the matrix
    // is singular (has no inverse). The filter uses this on the small
    // innovation-covariance matrix during each Kalman update.
    template <std::size_t N>
    Matrix<N, N> Inverse(const Matrix<N, N>& input) {
        Matrix<N, N> work = input;
        Matrix<N, N> result = Matrix<N, N>::Identity();

        for (std::size_t col = 0; col < N; ++col) {
            // Partial pivot: pick the row with the largest-magnitude entry in
            // this column as the pivot. Dividing by the largest available value
            // keeps rounding error small and avoids dividing by a tiny number.
            std::size_t pivot = col;
            double best = std::abs(work(col, col));
            for (std::size_t row = col + 1; row < N; ++row) {
                const double candidate = std::abs(work(row, col));
                if (candidate > best) {
                    best = candidate;
                    pivot = row;
                }
            }
            // If even the best pivot is essentially zero the columns are linearly
            // dependent and no inverse exists. 1e-15 is just above double
            // precision's roundoff floor (about 2.2e-16), so anything smaller is
            // numerical noise rather than a genuine value.
            if (best < 1e-15) {
                throw std::runtime_error("Matrix inverse: matrix is singular");
            }
            // Move the chosen pivot row up into place, swapping it in both
            // matrices so they stay in lockstep.
            if (pivot != col) {
                for (std::size_t c = 0; c < N; ++c) {
                    std::swap(work(col, c), work(pivot, c));
                    std::swap(result(col, c), result(pivot, c));
                }
            }

            // Scale the pivot row so its diagonal entry becomes 1.
            const double diagonal = work(col, col);
            for (std::size_t c = 0; c < N; ++c) {
                work(col, c) /= diagonal;
                result(col, c) /= diagonal;
            }

            // Subtract multiples of the pivot row from every other row to zero out
            // the rest of this column, applying the same operation to result.
            for (std::size_t row = 0; row < N; ++row) {
                if (row == col) {
                    continue;
                }
                const double factor = work(row, col);
                for (std::size_t c = 0; c < N; ++c) {
                    work(row, c) -= factor * work(col, c);
                    result(row, c) -= factor * result(col, c);
                }
            }
        }

        return result;
    }

}} // namespace TERCOM::Math
