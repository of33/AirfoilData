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

#include "AkimaInterpolation.h"

#include <cmath>
#include <limits>


bool AkimaInterpolation::init(const std::vector<double>& x, const std::vector<double>& y)
{
    initialized_ = false;

    if (x.size() < 2 || x.size() != y.size())
    {
        return false;
    }

    for (std::size_t index = 0; index < x.size(); ++index)
    {
        if (!std::isfinite(x[index]) || !std::isfinite(y[index]))
        {
            return false;
        }
        if ((index > 0) && (x[index] <= x[index - 1]))
        {
            return false;
        }
    }

    x_ = x;
    y_ = y;
    computeSlopes();

    for (std::size_t i = 0; i < slopes_.size(); ++i)
    {
        if (!std::isfinite(slopes_[i]))
        {
            x_.clear();
            y_.clear();
            slopes_.clear();
            return false;
        }
    }

    initialized_ = true;
    return true;
}


bool AkimaInterpolation::interpolate(double desiredAbscissa, double& value) const
{
    double firstDerivative = 0.0;
    double secondDerivative = 0.0;
    return interpolateInternal(desiredAbscissa, value, firstDerivative, secondDerivative,
                               DerivativeOrder::ValueOnly);
}


bool AkimaInterpolation::interpolate(double desiredAbscissa, double& value,
                                     double& firstDerivative) const
{
    double secondDerivative = 0.0;
    return interpolateInternal(desiredAbscissa, value, firstDerivative, secondDerivative,
                               DerivativeOrder::First);
}


bool AkimaInterpolation::interpolate(double desiredAbscissa, double& value,
                                     double& firstDerivative, double& secondDerivative) const
{
    return interpolateInternal(desiredAbscissa, value, firstDerivative, secondDerivative,
                               DerivativeOrder::Second);
}


double AkimaInterpolation::epsilon()
{
    // Use twice the machine epsilon as the support-point boundary tolerance.
    return 2.0 * std::numeric_limits<double>::epsilon();
}


void AkimaInterpolation::matrixTimesVector(const HermiteVector& input, HermiteVector& result)
{
    // Fixed 4x4 matrix (Hermite coefficients, row-wise)
    static const HermiteMatrix kMatrix = {{
        {{  2.0, -2.0,  1.0,  1.0}},
        {{ -3.0,  3.0, -2.0, -1.0}},
        {{  0.0,  0.0,  1.0,  0.0}},
        {{  1.0,  0.0,  0.0,  0.0}}
    }};

    for (std::size_t line = 0; line < 4; ++line)
    {
        double sum = 0.0;
        for (std::size_t column = 0; column < 4; ++column)
        {
            sum += kMatrix[line][column] * input[column];
        }
        result[line] = sum;
    }
}


void AkimaInterpolation::computeSlopes()
{
    // Keep the secant values explicit: the equality tests select the fallback
    // branches used for locally constant slope patterns.
    const std::size_t pointCount = x_.size();
    slopes_.resize(pointCount);

    /// Two support points define a linear function.
    if (pointCount == 2)
    {
        const double w = secantSlope(0, 1);
        slopes_[0] = w;
        slopes_[1] = w;
        return;
    }

    // slope[0] = 1.5 * (y1-y0)/(x1-x0) - 0.5 * (y2-y1)/(x2-x1)
    slopes_[0] = (1.5 * (y_[1] - y_[0])) / (x_[1] - x_[0])
               - (0.5 * (y_[2] - y_[1])) / (x_[2] - x_[1]);
    // (the factor form is deliberately left unchanged - bit-exact, see above)

    // slope[1]
    if (pointCount == 3)
    {
        // Special case (nPoints==3) in C: slope[1] = 0.5 * (m2 + m3).
        slopes_[1] = 0.5 * (secantSlope(0, 1) + secantSlope(1, 2));
    }
    else
    {
        const double m2 = secantSlope(0, 1);
        const double m3 = secantSlope(1, 2);
        const double m4 = secantSlope(2, 3);
        if ((m2 == m3) && (m4 == m3))
        {
            slopes_[1] = 0.5 * (m2 + m3);
        }
        else
        {
            slopes_[1] = ((std::fabs(m2 - m3) * (y_[2] - y_[1])) / (x_[2] - x_[1]) +
                          (std::fabs(m3 - m4) * (y_[1] - y_[0])) / (x_[1] - x_[0])) /
                         (std::fabs(m2 - m3) + std::fabs(m3 - m4));
        }
    }

    // Three-point endpoint treatment.
    if (pointCount == 3)
    {
        // slope[2] = 1.5 * (y2-y1)/(x2-x1) - 0.5 * (y1-y0)/(x1-x0)
        slopes_[2] = (1.5 * (y_[2] - y_[1])) / (x_[2] - x_[1])
                   - (0.5 * (y_[1] - y_[0])) / (x_[1] - x_[0]);
        return;
    }

    // Interior Akima slopes.
    for (std::size_t index = 2; index + 2 < pointCount; ++index)
    {
        const double m1 = secantSlope(index - 2, index - 1);
        const double m2 = secantSlope(index - 1, index);
        const double m3 = secantSlope(index, index + 1);
        const double m4 = secantSlope(index + 1, index + 2);
        if ((m4 == m3) && (m2 == m1))
        {
            slopes_[index] = 0.5 * (m2 + m3);
        }
        else
        {
            slopes_[index] =
                ((std::fabs(-m3 + m4) * (y_[index] - y_[index - 1])) /
                 (x_[index] - x_[index - 1]) +
                 (std::fabs(m2 - m1) * (y_[index + 1] - y_[index])) /
                 (x_[index + 1] - x_[index])) /
                (std::fabs(-m3 + m4) + std::fabs(m2 - m1));
        }
    }

    // Penultimate support-point slope.
    {
        const std::size_t last = pointCount - 1;
        const double m1 = secantSlope(last - 3, last - 2);
        const double m2 = secantSlope(last - 2, last - 1);
        const double m3 = secantSlope(last - 1, last);
        if ((m3 == m2) && (m2 == m1))
        {
            slopes_[last - 1] = 0.5 * (m2 + m3);
        }
        else
        {
            slopes_[last - 1] =
                ((std::fabs(-m2 + m3) * (y_[last - 1] - y_[last - 2])) /
                 (x_[last - 1] - x_[last - 2]) +
                 (std::fabs(m2 - m1) * (y_[last] - y_[last - 1])) /
                 (x_[last] - x_[last - 1])) /
                (std::fabs(-m2 + m3) + std::fabs(m2 - m1));
        }
    }

    // Final endpoint slope.
    {
        const std::size_t last = pointCount - 1;
        slopes_[last] =
            (1.5 * (y_[last] - y_[last - 1])) /
                (x_[last] - x_[last - 1])
          - (0.5 * (y_[last - 1] - y_[last - 2])) /
                (x_[last - 1] - x_[last - 2]);
    }
}


bool AkimaInterpolation::interpolateInternal(double desiredAbscissaPoint, double& value,
                                             double& firstDerivative, double& secondDerivative,
                                             DerivativeOrder derivativeOrder) const
{
    if (!initialized_ || !std::isfinite(desiredAbscissaPoint))
    {
        return false;
    }

    const std::size_t maxPoints = x_.size();
    const double dEpsilon = epsilon();

    // desiredIndex is the right endpoint of the selected interval.
    std::size_t desiredIndex;
    if (maxPoints == 2)
    {
        desiredIndex = 1;
    }
    else
    {
        if (desiredAbscissaPoint < x_[maxPoints - 1])
        {
            std::size_t lowerIndex = 1;
            desiredIndex = maxPoints - 1;
            do
            {
                const std::size_t intermediateIndex = (lowerIndex + desiredIndex) / 2;
                if (desiredAbscissaPoint < x_[intermediateIndex])
                {
                    desiredIndex = intermediateIndex;
                }
                else
                {
                    lowerIndex = intermediateIndex + 1;
                }
            }
            while (desiredIndex > lowerIndex);
        }
        else
        {
            desiredIndex = maxPoints - 1;
        }
    }

    const double dInterval = x_[desiredIndex] -
                             x_[desiredIndex - 1];

    const HermiteVector adP = {{
        y_[desiredIndex - 1],
        y_[desiredIndex],
        slopes_[desiredIndex - 1] * dInterval,
        slopes_[desiredIndex] * dInterval
    }};

    HermiteVector adInterim = {{0.0, 0.0, 0.0, 0.0}};
    matrixTimesVector(adP, adInterim);

    const double dDeltaX = desiredAbscissaPoint - x_[desiredIndex - 1];

    if (dDeltaX <= dEpsilon)
    {
        // Use the left support-point value for targets at or to the left of the
        // selected interval boundary.
        value = y_[desiredIndex - 1];
        switch (derivativeOrder)
        {
            case DerivativeOrder::ValueOnly:
                firstDerivative = value;
                break;
            case DerivativeOrder::First:
                firstDerivative = slopes_[desiredIndex - 1];
                secondDerivative = firstDerivative;
                break;
            case DerivativeOrder::Second:
                firstDerivative = slopes_[desiredIndex - 1];
                if (desiredIndex == 1)
                {
                    secondDerivative = (-6.0 * adP[0] + 6.0 * adP[1] -
                                        4.0 * adP[2] - 2.0 * adP[3]) /
                                       (dInterval * dInterval);
                }
                else
                {
                    const double dLeftInterval =
                        x_[desiredIndex - 1] -
                        x_[desiredIndex - 2];
                    const double leftTerm =
                        (-6.0 * y_[desiredIndex - 1] +
                         6.0 * y_[desiredIndex] -
                         4.0 * slopes_[desiredIndex - 1] * dInterval -
                         2.0 * slopes_[desiredIndex] * dInterval) /
                        (dInterval * dInterval);
                    const double rightTerm =
                        (6.0 * y_[desiredIndex - 2] -
                         6.0 * y_[desiredIndex - 1] +
                         2.0 * slopes_[desiredIndex - 2] * dLeftInterval +
                         4.0 * slopes_[desiredIndex - 1] * dLeftInterval) /
                        (dLeftInterval * dLeftInterval);
                    secondDerivative = (leftTerm + rightTerm) * 0.5;
                }
        }
        return true;
    }
    else if ((dDeltaX >= (x_[desiredIndex] -
                          x_[desiredIndex - 1] - dEpsilon)) &&
             (dDeltaX <= (x_[desiredIndex] -
                          x_[desiredIndex - 1] + dEpsilon)))
    {
        // Use the exact right support-point data inside the endpoint tolerance.
        value = y_[desiredIndex];
        switch (derivativeOrder)
        {
            case DerivativeOrder::ValueOnly:
                firstDerivative = value;
                break;
            case DerivativeOrder::First:
                firstDerivative = slopes_[desiredIndex];
                secondDerivative = firstDerivative;
                break;
            case DerivativeOrder::Second:
                firstDerivative = slopes_[desiredIndex];
                secondDerivative =
                    (6.0 * y_[desiredIndex - 1] -
                     6.0 * y_[desiredIndex] +
                     2.0 * slopes_[desiredIndex - 1] * dInterval +
                     4.0 * slopes_[desiredIndex] * dInterval) /
                    (dInterval * dInterval);
        }
        return true;
    }
    else
    {
        // Evaluate the cubic Hermite polynomial inside the selected interval.
        const double dParU = (desiredAbscissaPoint -
                             x_[desiredIndex - 1]) / dInterval;

        value = (adInterim[0] * dParU * dParU * dParU +
                 adInterim[1] * dParU * dParU +
                 adInterim[2] * dParU +
                 adInterim[3]);

        switch (derivativeOrder)
        {
            case DerivativeOrder::ValueOnly:
                secondDerivative = firstDerivative = value;
                break;
            case DerivativeOrder::First:
                firstDerivative = (3.0 * adInterim[0] * dParU * dParU +
                                   2.0 * adInterim[1] * dParU +
                                   adInterim[2]) / dInterval;
                secondDerivative = firstDerivative;
                break;
            case DerivativeOrder::Second:
                firstDerivative = (3.0 * adInterim[0] * dParU * dParU +
                                   2.0 * adInterim[1] * dParU +
                                   adInterim[2]) / dInterval;
                secondDerivative = (6.0 * adInterim[0] * dParU +
                                    2.0 * adInterim[1]) /
                                   (dInterval * dInterval);
        }
        return true;
    }
}
