//**************************************************************************
//   AkimaInterpolation
//   Structured C++ implementation of Akima interpolation
//
//   Copyright (C) 2026 Olaf Frommann (https://github.com/of33/AirfoilData)
//
//   This program is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with this program. If not, see <https://www.gnu.org/licenses/>.
//**************************************************************************

#ifndef AKIMAINTERPOLATION_H
#define AKIMAINTERPOLATION_H

#include <array>
#include <cstddef>
#include <vector>

/// One-dimensional piecewise cubic Akima interpolation.
class AkimaInterpolation
{
public:
    AkimaInterpolation() = default;

    /// Initializes the interpolator from finite support points.
    ///
    /// At least two points are required. The vectors must have equal
    /// lengths, and the x values must be strictly ascending. A failed call
    /// invalidates any previously stored interpolation state.
    ///
    /// @return true if the support points and calculated slopes are valid.
    [[nodiscard]] bool init(const std::vector<double>& x,
                            const std::vector<double>& y);

    /// Evaluates the interpolated value.
    ///
    /// Finite abscissas outside the support interval are accepted.
    /// On false, the output parameter remains unchanged.
    ///
    /// @return false if the object is uninitialized or the abscissa is
    ///         non-finite.
    [[nodiscard]] bool interpolate(double desiredAbscissa,
                                   double& value) const;

    /// Evaluates the interpolated value and first derivative.
    [[nodiscard]] bool interpolate(double desiredAbscissa,
                                   double& value,
                                   double& firstDerivative) const;

    /// Evaluates the value, first derivative, and second derivative.
    [[nodiscard]] bool interpolate(double desiredAbscissa,
                                   double& value,
                                   double& firstDerivative,
                                   double& secondDerivative) const;

private:
    using HermiteVector = std::array<double, 4>;
    using HermiteMatrix = std::array<std::array<double, 4>, 4>;

    void computeSlopes();

    double secantSlope(std::size_t a, std::size_t b) const
    {
        return (y_[b] - y_[a]) / (x_[b] - x_[a]);
    }

    static void matrixTimesVector(const HermiteVector& input,
                                  HermiteVector& result);

    static double epsilon();

    enum class DerivativeOrder
    {
        ValueOnly,
        First,
        Second
    };

    bool interpolateInternal(double desiredAbscissa,
                             double& value,
                             double& firstDerivative,
                             double& secondDerivative,
                             DerivativeOrder derivativeOrder) const;

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> slopes_;
    bool initialized_ = false;
};

#endif // AKIMAINTERPOLATION_H
