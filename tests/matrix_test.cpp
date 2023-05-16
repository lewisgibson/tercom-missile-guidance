#include <gtest/gtest.h>

#include <tercom/math/matrix.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>

using TERCOM::Math::Inverse;
using TERCOM::Math::Matrix;

TEST(Matrix, ZeroIsAllZeros) {
    const Matrix<2, 3> m = Matrix<2, 3>::Zero();
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_DOUBLE_EQ(m(r, c), 0.0);
        }
    }
}

TEST(Matrix, IdentityHasOnesOnDiagonal) {
    const Matrix<3, 3> id = Matrix<3, 3>::Identity();
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            const double expected = (r == c) ? 1.0 : 0.0;
            EXPECT_DOUBLE_EQ(id(r, c), expected);
        }
    }
}

TEST(Matrix, ElementAccessGetAndSet) {
    Matrix<2, 2> m = Matrix<2, 2>::Zero();
    m(0, 0) = 1.5;
    m(0, 1) = -2.0;
    m(1, 0) = 3.25;
    m(1, 1) = 4.0;

    EXPECT_DOUBLE_EQ(m(0, 0), 1.5);
    EXPECT_DOUBLE_EQ(m(0, 1), -2.0);
    EXPECT_DOUBLE_EQ(m(1, 0), 3.25);
    EXPECT_DOUBLE_EQ(m(1, 1), 4.0);
}

TEST(Matrix, ConstElementAccessReadsStoredValues) {
    Matrix<2, 2> mutable_m = Matrix<2, 2>::Zero();
    mutable_m(1, 1) = 7.0;
    const Matrix<2, 2>& const_ref = mutable_m;
    EXPECT_DOUBLE_EQ(const_ref(1, 1), 7.0);
    EXPECT_DOUBLE_EQ(const_ref(0, 0), 0.0);
}

TEST(Matrix, RowMajorStorageLayout) {
    Matrix<2, 3> m = Matrix<2, 3>::Zero();
    m(0, 0) = 1.0;
    m(0, 1) = 2.0;
    m(0, 2) = 3.0;
    m(1, 0) = 4.0;
    m(1, 1) = 5.0;
    m(1, 2) = 6.0;

    // Row-major: index = row * Cols + col.
    EXPECT_DOUBLE_EQ(m.data[0], 1.0);
    EXPECT_DOUBLE_EQ(m.data[1], 2.0);
    EXPECT_DOUBLE_EQ(m.data[2], 3.0);
    EXPECT_DOUBLE_EQ(m.data[3], 4.0);
    EXPECT_DOUBLE_EQ(m.data[4], 5.0);
    EXPECT_DOUBLE_EQ(m.data[5], 6.0);
}

TEST(Matrix, Addition) {
    Matrix<2, 2> a = Matrix<2, 2>::Zero();
    a(0, 0) = 1.0;
    a(0, 1) = 2.0;
    a(1, 0) = 3.0;
    a(1, 1) = 4.0;

    Matrix<2, 2> b = Matrix<2, 2>::Zero();
    b(0, 0) = 10.0;
    b(0, 1) = 20.0;
    b(1, 0) = 30.0;
    b(1, 1) = 40.0;

    const Matrix<2, 2> sum = a + b;
    EXPECT_DOUBLE_EQ(sum(0, 0), 11.0);
    EXPECT_DOUBLE_EQ(sum(0, 1), 22.0);
    EXPECT_DOUBLE_EQ(sum(1, 0), 33.0);
    EXPECT_DOUBLE_EQ(sum(1, 1), 44.0);
}

TEST(Matrix, Subtraction) {
    Matrix<2, 2> a = Matrix<2, 2>::Zero();
    a(0, 0) = 10.0;
    a(0, 1) = 20.0;
    a(1, 0) = 30.0;
    a(1, 1) = 40.0;

    Matrix<2, 2> b = Matrix<2, 2>::Zero();
    b(0, 0) = 1.0;
    b(0, 1) = 2.0;
    b(1, 0) = 3.0;
    b(1, 1) = 4.0;

    const Matrix<2, 2> diff = a - b;
    EXPECT_DOUBLE_EQ(diff(0, 0), 9.0);
    EXPECT_DOUBLE_EQ(diff(0, 1), 18.0);
    EXPECT_DOUBLE_EQ(diff(1, 0), 27.0);
    EXPECT_DOUBLE_EQ(diff(1, 1), 36.0);
}

TEST(Matrix, AdditionThenSubtractionRoundTrip) {
    Matrix<2, 3> a = Matrix<2, 3>::Zero();
    a(0, 0) = 1.0;
    a(0, 1) = -2.0;
    a(0, 2) = 3.0;
    a(1, 0) = -4.0;
    a(1, 1) = 5.0;
    a(1, 2) = -6.0;

    Matrix<2, 3> b = Matrix<2, 3>::Zero();
    b(0, 0) = 7.0;
    b(1, 2) = 9.0;

    const Matrix<2, 3> restored = (a + b) - b;
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_DOUBLE_EQ(restored(r, c), a(r, c));
        }
    }
}

TEST(Matrix, ScalarMultiplication) {
    Matrix<2, 2> m = Matrix<2, 2>::Zero();
    m(0, 0) = 1.0;
    m(0, 1) = -2.0;
    m(1, 0) = 3.0;
    m(1, 1) = 4.0;

    const Matrix<2, 2> scaled = 2.5 * m;
    EXPECT_DOUBLE_EQ(scaled(0, 0), 2.5);
    EXPECT_DOUBLE_EQ(scaled(0, 1), -5.0);
    EXPECT_DOUBLE_EQ(scaled(1, 0), 7.5);
    EXPECT_DOUBLE_EQ(scaled(1, 1), 10.0);
}

TEST(Matrix, ScalarMultiplicationByZeroGivesZero) {
    Matrix<2, 2> m = Matrix<2, 2>::Zero();
    m(0, 0) = 5.0;
    m(1, 1) = -9.0;

    const Matrix<2, 2> scaled = 0.0 * m;
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 2; ++c) {
            EXPECT_DOUBLE_EQ(scaled(r, c), 0.0);
        }
    }
}

TEST(Matrix, MultiplyNonSquare) {
    // 2x3 times 3x2 yields a 2x2.
    Matrix<2, 3> a = Matrix<2, 3>::Zero();
    a(0, 0) = 1.0;
    a(0, 1) = 2.0;
    a(0, 2) = 3.0;
    a(1, 0) = 4.0;
    a(1, 1) = 5.0;
    a(1, 2) = 6.0;

    Matrix<3, 2> b = Matrix<3, 2>::Zero();
    b(0, 0) = 7.0;
    b(0, 1) = 8.0;
    b(1, 0) = 9.0;
    b(1, 1) = 10.0;
    b(2, 0) = 11.0;
    b(2, 1) = 12.0;

    const Matrix<2, 2> product = a * b;
    // Row 0: [1*7+2*9+3*11, 1*8+2*10+3*12] = [58, 64]
    // Row 1: [4*7+5*9+6*11, 4*8+5*10+6*12] = [139, 154]
    EXPECT_DOUBLE_EQ(product(0, 0), 58.0);
    EXPECT_DOUBLE_EQ(product(0, 1), 64.0);
    EXPECT_DOUBLE_EQ(product(1, 0), 139.0);
    EXPECT_DOUBLE_EQ(product(1, 1), 154.0);
}

TEST(Matrix, MultiplyByIdentityIsUnchanged) {
    Matrix<3, 3> a = Matrix<3, 3>::Zero();
    a(0, 0) = 1.0;
    a(0, 1) = 2.0;
    a(0, 2) = 3.0;
    a(1, 0) = 4.0;
    a(1, 1) = 5.0;
    a(1, 2) = 6.0;
    a(2, 0) = 7.0;
    a(2, 1) = 8.0;
    a(2, 2) = 10.0;

    const Matrix<3, 3> id = Matrix<3, 3>::Identity();
    const Matrix<3, 3> right = a * id;
    const Matrix<3, 3> left = id * a;
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_DOUBLE_EQ(right(r, c), a(r, c));
            EXPECT_DOUBLE_EQ(left(r, c), a(r, c));
        }
    }
}

TEST(Matrix, TransposeSwapsDimensionsAndValues) {
    Matrix<2, 3> m = Matrix<2, 3>::Zero();
    m(0, 0) = 1.0;
    m(0, 1) = 2.0;
    m(0, 2) = 3.0;
    m(1, 0) = 4.0;
    m(1, 1) = 5.0;
    m(1, 2) = 6.0;

    const Matrix<3, 2> t = m.Transposed();
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_DOUBLE_EQ(t(c, r), m(r, c));
        }
    }
    // Spot check explicit dimensions and value placement.
    EXPECT_DOUBLE_EQ(t(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(t(2, 1), 6.0);
    EXPECT_DOUBLE_EQ(t(1, 0), 2.0);
}

TEST(Matrix, DoubleTransposeIsOriginal) {
    Matrix<2, 3> m = Matrix<2, 3>::Zero();
    m(0, 0) = 1.0;
    m(0, 1) = -2.0;
    m(0, 2) = 3.0;
    m(1, 0) = 4.0;
    m(1, 1) = 5.0;
    m(1, 2) = -6.0;

    const Matrix<2, 3> back = m.Transposed().Transposed();
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_DOUBLE_EQ(back(r, c), m(r, c));
        }
    }
}

TEST(Matrix, Inverse2x2KnownValue) {
    Matrix<2, 2> a = Matrix<2, 2>::Zero();
    a(0, 0) = 4.0;
    a(0, 1) = 7.0;
    a(1, 0) = 2.0;
    a(1, 1) = 6.0;

    // det = 4*6 - 7*2 = 10. Inverse = (1/10) * [[6, -7], [-2, 4]].
    const Matrix<2, 2> inv = Inverse<2>(a);
    EXPECT_NEAR(inv(0, 0), 0.6, 1e-12);
    EXPECT_NEAR(inv(0, 1), -0.7, 1e-12);
    EXPECT_NEAR(inv(1, 0), -0.2, 1e-12);
    EXPECT_NEAR(inv(1, 1), 0.4, 1e-12);
}

TEST(Matrix, Inverse2x2TimesOriginalIsIdentity) {
    Matrix<2, 2> a = Matrix<2, 2>::Zero();
    a(0, 0) = 4.0;
    a(0, 1) = 7.0;
    a(1, 0) = 2.0;
    a(1, 1) = 6.0;

    const Matrix<2, 2> inv = Inverse<2>(a);
    const Matrix<2, 2> product = a * inv;
    const Matrix<2, 2> id = Matrix<2, 2>::Identity();
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 2; ++c) {
            EXPECT_NEAR(product(r, c), id(r, c), 1e-12);
        }
    }
}

TEST(Matrix, Inverse3x3TimesOriginalIsIdentity) {
    // A general invertible 3x3 matrix (det = 1) that needs pivoting.
    Matrix<3, 3> a = Matrix<3, 3>::Zero();
    a(0, 0) = 2.0;
    a(0, 1) = -1.0;
    a(0, 2) = 0.0;
    a(1, 0) = -1.0;
    a(1, 1) = 2.0;
    a(1, 2) = -1.0;
    a(2, 0) = 0.0;
    a(2, 1) = -1.0;
    a(2, 2) = 2.0;

    const Matrix<3, 3> inv = Inverse<3>(a);
    const Matrix<3, 3> id = Matrix<3, 3>::Identity();

    const Matrix<3, 3> right = a * inv;
    const Matrix<3, 3> left = inv * a;
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(right(r, c), id(r, c), 1e-10);
            EXPECT_NEAR(left(r, c), id(r, c), 1e-10);
        }
    }
}

TEST(Matrix, InverseRequiringPartialPivot) {
    // Zero leading pivot forces a row swap during Gauss-Jordan.
    Matrix<2, 2> a = Matrix<2, 2>::Zero();
    a(0, 0) = 0.0;
    a(0, 1) = 1.0;
    a(1, 0) = 1.0;
    a(1, 1) = 0.0;

    const Matrix<2, 2> inv = Inverse<2>(a);
    // This matrix is its own inverse (a swap permutation).
    EXPECT_NEAR(inv(0, 0), 0.0, 1e-12);
    EXPECT_NEAR(inv(0, 1), 1.0, 1e-12);
    EXPECT_NEAR(inv(1, 0), 1.0, 1e-12);
    EXPECT_NEAR(inv(1, 1), 0.0, 1e-12);
}

TEST(Matrix, InverseOfIdentityIsIdentity) {
    const Matrix<3, 3> id = Matrix<3, 3>::Identity();
    const Matrix<3, 3> inv = Inverse<3>(id);
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(inv(r, c), id(r, c), 1e-12);
        }
    }
}

TEST(Matrix, InverseThrowsOnZeroMatrix) {
    const Matrix<3, 3> zero = Matrix<3, 3>::Zero();
    EXPECT_THROW(Inverse<3>(zero), std::runtime_error);
}

TEST(Matrix, InverseThrowsOnRankDeficientMatrix) {
    // Second row is twice the first, so the matrix is singular.
    Matrix<2, 2> singular = Matrix<2, 2>::Zero();
    singular(0, 0) = 1.0;
    singular(0, 1) = 2.0;
    singular(1, 0) = 2.0;
    singular(1, 1) = 4.0;

    EXPECT_THROW(Inverse<2>(singular), std::runtime_error);
}

TEST(Matrix, InverseThrowsOnRankDeficient3x3) {
    // Third column equals the sum of the first two; columns are dependent.
    Matrix<3, 3> singular = Matrix<3, 3>::Zero();
    singular(0, 0) = 1.0;
    singular(0, 1) = 2.0;
    singular(0, 2) = 3.0;
    singular(1, 0) = 4.0;
    singular(1, 1) = 5.0;
    singular(1, 2) = 9.0;
    singular(2, 0) = 7.0;
    singular(2, 1) = 8.0;
    singular(2, 2) = 15.0;

    EXPECT_THROW(Inverse<3>(singular), std::runtime_error);
}
