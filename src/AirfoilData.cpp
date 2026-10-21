//**************************************************************************
//   AirfoilData
//   Storing, manipulating and analysing airfoils in various ways
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

#include "AirfoilData.h"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include "AkimaInterpolation.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

static_assert(std::is_nothrow_move_assignable_v<AirfoilData>,
              "AirfoilData must be nothrow-move-assignable for the "
              "transactional commit");

namespace
{

bool allFiniteValues(const std::vector<double>& values)
{
    // Returns true if all vector elements are finite.
    for (std::size_t i = 0; i < values.size(); i++)
    {
        if (!std::isfinite(values[i]))
        {
            return false;
        }
    }
    return true;
}

template<typename... Parts>
std::string joinMessage(const Parts&... parts)
{
    std::string result;
    auto appendPart = [&result](const auto& part)
    {
        if constexpr (std::is_arithmetic_v<std::decay_t<decltype(part)>>)
            result += std::to_string(part);
        else
            result += std::string(part);
    };
    (appendPart(parts), ...);
    return result;
}

} // namespace


//********************************
// Construction and initialization
//********************************

void AirfoilData::clearAirfoilData()
{
    airfoilX.clear();
    airfoilY.clear();

    clearCalculatedGeometryData();
}


void AirfoilData::clearCalculatedGeometryData()
{
    // Clears all data derived from or associated with the current contour.
    // The contour coordinates, name, and desired point count are retained.
    thickness = 0.0;
    maxThicknessLocation = 0.0;
    teThickness = 0.0;
    camber = 0.0;
    maxCamberPosition = 0.0;
    dyNose = 0.0;
    area = 0.0;
    alpha0Geo = 0.0;
    cm0Geo = 0.0;

    upperHalfX.clear();
    upperHalfY.clear();
    lowerHalfX.clear();
    lowerHalfY.clear();
    camberX.clear();
    camberY.clear();
    camberDerivativeX.clear();
    camberDerivativeY.clear();
    thicknessDerivativeX.clear();
    thicknessDerivativeY.clear();
    conformalFunctionX.clear();
    conformalFunctionY.clear();
    conformalCircleX.clear();
    conformalCircleY.clear();
    sourceX.clear();
    sourceY.clear();
    circX.clear();
    circY.clear();
    thickDistX.clear();
    thickDistY.clear();
    gradientUpperX.clear();
    gradientLowerX.clear();
    upperGradient.clear();
    lowerGradient.clear();
    upperCurvature.clear();
    lowerCurvature.clear();

    alpha.clear();
    cL.clear();
    cD.clear();
    cDf.clear();
    cDp.clear();
    cM.clear();
    eVector.clear();
    converged.clear();
    alpha0 = 0.0;
    cm0 = 0.0;
    cl0 = 0.0;
    cd0 = 0.0;
    clMax = 0.0;
    eMax = 0.0;

    cpUpperX.clear();
    cpLowerX.clear();
    cpUpper.clear();
    cpLower.clear();

    created = false;
}

void AirfoilData::initAirfoil(std::size_t size)
{
    // The public template size follows the shared 99..511 rule. Even values
    // are reduced to the next lower odd value.
    std::size_t pointCount = size;
    if (false == validatePointCount(pointCount))
    {
        // An invalid request clears the current contour but retains the desired
        // target point count.
        clearAirfoilData();
        return;
    }

    std::vector<double> newX = createXDistribution(pointCount);
    std::vector<double> newY(pointCount);
    std::vector<double> newCamberX = createXDistribution(pointCount);
    std::vector<double> newCamberY(pointCount);
    std::vector<double> newThickDistX = createXDistribution(pointCount);
    std::vector<double> newThickDistY(pointCount);

    clearAirfoilData();

    desiredNumberOfAirfoilPoints = pointCount;
    airfoilX.swap(newX);
    airfoilY.swap(newY);
    camberX.swap(newCamberX);
    camberY.swap(newCamberY);
    thickDistX.swap(newThickDistX);
    thickDistY.swap(newThickDistY);
}

//***********
// IO methods
//***********

void AirfoilData::emitOutput(const std::string& text, AirfoilDataMessageType type)
{
    if (outputSuppressed_)
    {
        return;
    }
    if (nullptr != print)
    {
        // Use the application callback when installed.
        print(printContext, text, type);
        return;
    }

    // Otherwise write a prefixed message to the appropriate console stream.
    const char* prefix;
    switch (type)
    {
        case AirfoilDataMessageType::Error:   prefix = "[Error] ";   break;
        case AirfoilDataMessageType::ErrorSevere: prefix = "[ErrorSevere] "; break;
        case AirfoilDataMessageType::Warning: prefix = "[Warning] "; break;
        case AirfoilDataMessageType::WarningSevere: prefix = "[WarningSevere] "; break;
        case AirfoilDataMessageType::Info:    prefix = "[Info] ";    break;
        case AirfoilDataMessageType::Normal:  prefix = "[Normal] ";  break;
        default:                              prefix = "";           break;
    }
    if ((AirfoilDataMessageType::Error == type) || (AirfoilDataMessageType::Warning == type) ||
        (AirfoilDataMessageType::WarningSevere == type) || (AirfoilDataMessageType::ErrorSevere == type))
    {
        std::cerr << prefix << text << std::endl;
    }
    else
    {
        std::cout << prefix << text << std::endl;
    }
}

bool AirfoilData::saveAirfoilCoordinates(const std::string& fileName, const std::string& source)
{
    if (false == created)
    {
        emitOutput(std::string("No airfoil data to save!"), AirfoilDataMessageType::Error);
        return false;
    }
    // Append the standard suffix when it is not already present.
    std::string fName = fileName;
    const bool hasDatSuffix =
        fName.size() >= 4 &&
        0 == fName.compare(fName.size() - 4, 4, ".dat");
    if (!hasDatSuffix)
    {
        fName += ".dat";
    }

    // Write the thickness column only when sufficient thickness data exist.
    const bool hasThickness = (thickDistY.size() >= airfoilX.size());

    errno = 0;

    std::ofstream af(fName, std::ios::out);
    if (af.fail())
    {
        int openErrno = errno;
        emitOutput(std::string("Cannot open file ") + fName + std::string(".\n") +
                   std::string((0 != openErrno) ? strerror(openErrno) : "unknown error"),
                   AirfoilDataMessageType::Error);
        return false;
    }

    // Application-neutral: the caller supplies the source wording
    // (e.g. "AirfoilCenter method 'NacaAirfoil'"); an empty source
    // writes nothing.
    af << "# " << name
       << "\n# n = " << airfoilX.size();
    if (!source.empty())
    {
        af << "  Created by " << source;
    }
    af << "\n";
    if (hasThickness)
    {
        af << "#    x             y             yt\n";
    }
    else
    {
        af << "#    x             y\n";
    }

    for (std::size_t i = 0; i < airfoilX.size(); i++)
    {
        if (hasThickness)
        {
            af << std::scientific << airfoilX[i] << " " << airfoilY[i] << " " << thickDistY[i] << "\n";
        }
        else
        {
            af << std::scientific << airfoilX[i] << " " << airfoilY[i] << "\n";
        }
    }
    af.close();
    if (af.fail())
    {
        emitOutput(std::string("Error writing the file ") + fName,
                   AirfoilDataMessageType::Error);
        return false;
    }
    return true;
}

//*****************
// Geometry methods
//*****************

void AirfoilData::removeConsecutiveDuplicates(std::vector<double>& xCoordinates, std::vector<double>& yCoordinates)
{
    if (xCoordinates.empty() || yCoordinates.empty() || xCoordinates.size() != yCoordinates.size()) {
        return; // Early exit for invalid input
    }

    const double eps = AirfoilDataConstants::kDuplicateEpsilon;

    // Keep the last point of each consecutive duplicate group.
    const std::size_t n = xCoordinates.size();
    std::size_t write = 0;
    for (std::size_t read = 0; read < n; read++)
    {
        if ((read + 1 < n) && (std::fabs(xCoordinates[read] - xCoordinates[read + 1]) <= eps))
        {
            emitOutput(joinMessage("R ", xCoordinates[read], " ", xCoordinates[read + 1]),
                       AirfoilDataMessageType::Info);
        }
        else
        {
            xCoordinates[write] = xCoordinates[read];
            yCoordinates[write] = yCoordinates[read];
            write++;
        }
    }

    xCoordinates.resize(write);
    yCoordinates.resize(write);
}

bool AirfoilData::isInteriorNose(const std::vector<double>& xCoordinates, std::size_t noseIndex)
{
    // A valid interior leading edge must lie at the global x minimum within a
    // scale-dependent floating-point tolerance. Both contour branches must
    // contain at least one strictly larger x value.

    if ((noseIndex < 1) || (noseIndex + 1 >= xCoordinates.size()))
    {
        return false;
    }
    const std::size_t nose = noseIndex;
    const double minimumX = *std::min_element(xCoordinates.begin(), xCoordinates.end());
    const double scale = std::max(1.0, std::max(std::fabs(minimumX), std::fabs(xCoordinates[nose])));
    const double tolerance = 16.0 * std::numeric_limits<double>::epsilon() * scale;
    if (std::fabs(xCoordinates[nose] - minimumX) > tolerance)
    {
        return false;
    }

    bool largerValueBeforeNose = false;
    for (std::size_t index = 0; index < nose; ++index)
    {
        if (xCoordinates[index] > xCoordinates[nose])
        {
            largerValueBeforeNose = true;
            break;
        }
    }
    bool largerValueAfterNose = false;
    for (std::size_t index = nose + 1; index < xCoordinates.size(); ++index)
    {
        if (xCoordinates[index] > xCoordinates[nose])
        {
            largerValueAfterNose = true;
            break;
        }
    }
    return largerValueBeforeNose && largerValueAfterNose;
}

bool AirfoilData::isSecantNose(const std::vector<double>& xCoordinates, std::size_t noseIndex)
{
    // Validate a candidate obtained by the longest-secant method. Both contour
    // endpoints must lie near the maximum x coordinate and sufficiently close
    // to each other in x to represent the two trailing-edge points. Exact
    // geometric closure is not required.

    if ((noseIndex < 1) ||
        (noseIndex + 1 >= xCoordinates.size()))
    {
        return false;
    }
    if (xCoordinates.size() < 2)
    {
        return false;
    }
    const double first = xCoordinates.front();
    const double last = xCoordinates.back();

    // Tolerance relative to the x-extent (not to the magnitudes - otherwise
    // with tiny or extremely large coordinates an entire
    // monotone region would be accepted as "equal").
    const double minX = *std::min_element(xCoordinates.cbegin(), xCoordinates.cend());
    const double maxX = *std::max_element(xCoordinates.cbegin(), xCoordinates.cend());
    const double xRange = maxX - minX;

    // Tolerance relative to the x-extent: the trailing edge of a real
    // airfoil is not exactly closed (NACA upper/lower edges differ by
    // the residual thickness in x, order of magnitude 1e-4 of the chord).
    const double tolerance = 1e-3 * xRange;

    // The trailing edge lies at the maximum x - both end points must
    // lie there (a contour that begins/ends at the NOSE is not
    // recognizable as an airfoil).
    return (std::fabs(first - last) <= tolerance) &&
           (std::fabs(first - maxX) <= tolerance) &&
           (std::fabs(last - maxX) <= tolerance);
}

bool AirfoilData::getNoseIndex(const std::vector<double>& profileX,
                               const std::vector<double>& profileY,
                               NoseDetectionMode mode,
                               std::size_t& noseIndex) const
{
    if ((profileX.size() != profileY.size()) || (profileX.size() < 2))
    {
        return false;
    }

    for (std::size_t iFin = 0; iFin < profileX.size(); iFin++)
    {
        if (!std::isfinite(profileX[iFin]) || !std::isfinite(profileY[iFin]))
        {
            return false;
        }
    }

    std::size_t foundIndex = 0;
    if (NoseDetectionMode::LongestSecant == mode)
    {
        // Measure secant lengths from the midpoint of the two contour endpoints.
        // This avoids a lower- or upper-corner bias for open trailing edges.
        const std::size_t lastIndex = profileX.size() - 1;
        const double originX = profileX[0] + 0.5 * (profileX[lastIndex] - profileX[0]);
        const double originY = profileY[0] + 0.5 * (profileY[lastIndex] - profileY[0]);
        const double squareRootMaximum =
            std::sqrt(std::numeric_limits<double>::max() / 2.0);
        double longestSecant = 0.0;
        for (std::size_t i = profileX.size() - 2; i > 0; i--)
        {
            const double dx = originX - profileX[i];
            const double dy = profileY[i] - originY;
            if (!std::isfinite(dx) || !std::isfinite(dy))
            {
                return false;
            }
            double secantLength;
            if ((std::fabs(dx) > squareRootMaximum) ||
                (std::fabs(dy) > squareRootMaximum))
            {
                // Avoid overflow in the direct sum of squares.
                secantLength = std::hypot(dx, dy);
            }
            else
            {
                secantLength = std::sqrt(dx * dx + dy * dy);
            }
            if (secantLength > longestSecant)
            {
                longestSecant = secantLength;
                foundIndex = i;
            }
        }
    }
    else
    {
        for (std::size_t i = 1; i < profileX.size() - 1; i++)
        {
            // Retain the last strict local minimum found.
            if ((profileX[i - 1] > profileX[i]) && (profileX[i + 1] > profileX[i]))
            {
                foundIndex = i;
            }
        }
    }
    noseIndex = foundIndex;
    return true;
}

bool AirfoilData::mapAirfoilToSupportPoints(const std::vector<double>& inputX,
                                            const std::vector<double>& inputY,
                                            std::vector<double>& redistributedX,
                                            std::vector<double>& redistributedY,
                                            std::size_t newPointCount)
{
    const std::string methodName = "mapAirfoilToSupportPoints(): ";

    if (&redistributedX == &redistributedY)
    {
        emitOutput(methodName + "output vectors must be different",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (inputX.size() != inputY.size() || inputX.empty() || newPointCount < 3) {
        return false;
    }

    for (std::size_t i = 0; i < inputX.size(); ++i)
    {
        if (!std::isfinite(inputX[i]) || !std::isfinite(inputY[i]))
        {
            emitOutput(methodName + "input coordinates not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
    }

    // 1. Scale by the chordwise extent.
    double minX = *std::min_element(inputX.begin(), inputX.end());
    double maxX = *std::max_element(inputX.begin(), inputX.end());
    double rangeX = maxX - minX;

    if (!std::isfinite(rangeX) || 0.0 == rangeX)
    {
        emitOutput(methodName + "scaling range (maxX - minX) not finite or zero",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::vector<double> scaledX, scaledY;
    scaledX.reserve(inputX.size());
    scaledY.reserve(inputY.size());

    for (std::size_t i = 0; i < inputX.size(); ++i) {
        const double sx = (inputX[i] - minX) / rangeX;
        const double sy = inputY[i] / rangeX;

        if (!std::isfinite(sx) || !std::isfinite(sy))
        {
            emitOutput(methodName + "scaled coordinates not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
        scaledX.push_back(sx);
        scaledY.push_back(sy);
    }

    // 2. Establish the expected contour orientation and remove consecutive
    //    chordwise duplicates.
    sortDataClockwise(scaledX, scaledY);
    removeConsecutiveDuplicates(scaledX, scaledY);
    std::size_t noseIndex;
    if (!getNoseIndex(scaledX, scaledY, NoseDetectionMode::LocalMinimum, noseIndex))
    {
        emitOutput(methodName + "cannot determine nose position",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (!isInteriorNose(scaledX, noseIndex))
    {
        emitOutput(methodName + "no inner leading edge found",
                   AirfoilDataMessageType::Error);
        return false;
    }

    // 3. Transform both branches into one ascending interpolation sequence.
    for (std::size_t i = 0; i < noseIndex; i++)
    {
        scaledX[i] *= -1;
    }

    // 4. Initialize the contour interpolator.
    AkimaInterpolation akima;
    if (false == akima.init(scaledX, scaledY))
    {
        emitOutput(std::string("Error during initialization of akima"), AirfoilDataMessageType::Error);
        return false;
    }

    std::vector<double> newRedistributedX;
    std::vector<double> newRedistributedY;

    if (newPointCount > newRedistributedX.max_size())
    {
        emitOutput(methodName + "newPointCount exceeds max_size",
                   AirfoilDataMessageType::Error);
        return false;
    }
    newRedistributedX.reserve(newPointCount);
    newRedistributedY.reserve(newPointCount);

    const std::size_t lowerPointCount = newPointCount / 2 + newPointCount % 2;
    const std::size_t upperPointCount = newPointCount - lowerPointCount;

    // 5. Generate lower-side target coordinates.
    std::vector<double> newLowerX;
    newLowerX.reserve(lowerPointCount);

    for (std::size_t i = 0; i < lowerPointCount; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(lowerPointCount - 1);
        double phi = AirfoilDataConstants::kPi * t;

        double newX = -(1 - std::cos((phi - AirfoilDataConstants::kPi) / 2.0));
        newLowerX.push_back(newX);
    }

    // 6. Interpolate the lower side.
    for (std::size_t i = 0; i < newLowerX.size(); ++i) {
        double supportX = newLowerX[i];
        double interpolatedY = 0.0;
        if (!akima.interpolate(supportX, interpolatedY))
        {
                emitOutput(std::string("Akima interpolation not successful (lower side, extrapolation)"),
                           AirfoilDataMessageType::Error);
            return false;
        }

        if (!std::isfinite(interpolatedY))
        {
            emitOutput(methodName + "Akima result not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
        newRedistributedX.push_back(supportX);
        newRedistributedY.push_back(interpolatedY);
    }

    // 7. Interpolate the upper side.
    for (std::size_t index = upperPointCount; index > 0; --index) {
        const std::size_t j = index - 1;
        const double t = static_cast<double>(j) / static_cast<double>(upperPointCount);
        const double phi = AirfoilDataConstants::kPi * t;
        const double xUpper = (1 - std::cos((phi - AirfoilDataConstants::kPi) / 2.0));

        double interpolatedY = 0.0;
        if (!akima.interpolate(xUpper, interpolatedY))
        {
                emitOutput(std::string("Akima interpolation not successful (upper side, extrapolation)"),
                           AirfoilDataMessageType::Error);
            return false;
        }

        if (!std::isfinite(interpolatedY))
        {
            emitOutput(methodName + "Akima result not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }

        newRedistributedX.push_back(xUpper);
        newRedistributedY.push_back(interpolatedY);
    }

    // 8. Restore ordinary non-negative chordwise coordinates.
    for (std::size_t i = 0; i < newRedistributedX.size(); ++i) {
        if (newRedistributedX[i] < 0) {
            newRedistributedX[i] *= -1;
        }
    }

    if (newRedistributedX.size() != newPointCount ||
        newRedistributedY.size() != newPointCount)
    {
        emitOutput(joinMessage(methodName, "unexpected point count, expected ",
                               newPointCount, ", produced ", newRedistributedX.size()),
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (!allFiniteValues(newRedistributedX) || !allFiniteValues(newRedistributedY))
    {
        emitOutput(methodName + "result not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }

    redistributedX.swap(newRedistributedX);
    redistributedY.swap(newRedistributedY);
    return true;
}

bool AirfoilData::airfoilOK(TrailingEdgeCheck trailingEdgeCheck)
{
    const std::string methodName = "airfoilOK(): ";
    bool ok = true;
    std::size_t noseIndex;
    double yUpper;

    // Transform the complete contour into one ascending Akima abscissa sequence.

    if (!getNoseIndex(airfoilX, airfoilY, NoseDetectionMode::LocalMinimum, noseIndex) ||
        !isInteriorNose(airfoilX, noseIndex))
    {
        emitOutput(methodName + "no inner leading edge found",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::vector<double> ascendingX = airfoilX;
    for (std::size_t index = 0;
         index < noseIndex;
         ++index)
    {
        ascendingX[index] = -ascendingX[index];
    }

    AkimaInterpolation akima;

    if (!akima.init(ascendingX, airfoilY))
    {
        emitOutput(std::string("Error during initialization of akima"), AirfoilDataMessageType::Error);
        return false;
    }

    double xMax;
    if (TrailingEdgeCheck::SkipNearEdge == trailingEdgeCheck)
    {
        // avoid checking points after this value
        xMax = AirfoilDataConstants::kTrailingEdgeLimit;
    }
    else
    {
        xMax = 1.0;
    }

    // Skip the first trailing-edge point, which may coincide with the opposite
    // trailing-edge endpoint.
    for (std::size_t i = 1; i < noseIndex; i++)
    {
        if ((ascendingX[i] > -xMax) && (ascendingX[i] < xMax))
        {
            if (!akima.interpolate(-ascendingX[i], yUpper))
            {
                emitOutput(joinMessage("Akima interpolation not successful (extrapolation at point ", i, ")"),
                           AirfoilDataMessageType::Error);
                return false;
            }
            if (yUpper < airfoilY[i])
            {
                emitOutput(joinMessage("Airfoil has loop(s) (upper coordinate below lower coordinate)"),
                           AirfoilDataMessageType::Warning);
                return false;
            }
        }
    }

    return ok;
}

bool AirfoilData::calculateUpperLowerCoordinates()
{
    const std::string methodName = "calculateUpperLowerCoordinates(): ";

    std::size_t noseIndex;

    const std::vector<double>& profileX = getAirfoilX();
    const std::vector<double>& profileY = getAirfoilY();
    const std::size_t nSize = profileX.size();

    if (!getNoseIndex(profileX, profileY, NoseDetectionMode::LocalMinimum, noseIndex) ||
        !isInteriorNose(profileX, noseIndex))
    {
        emitOutput(methodName + "no inner leading edge found",
                   AirfoilDataMessageType::Error);
        return false;
    }

    // 0 <= noseIndex < nSize checked
    const std::size_t noseSize = noseIndex;

    std::vector<double> xLower(noseSize + 1);
    std::vector<double> yLower(noseSize + 1);
    std::vector<double> xUpper(nSize - noseSize);
    std::vector<double> yUpper(nSize - noseSize);

    for (std::size_t i = 0; i <= noseSize; i++)
    {
        xLower[i] = profileX[i];
        yLower[i] = profileY[i];
    }

    for (std::size_t i = noseSize; i < nSize; i++)
    {
        xUpper[nSize - i - 1] = profileX[i];
        yUpper[nSize - i - 1] = profileY[i];
    }

    lowerHalfX.swap(xLower);
    lowerHalfY.swap(yLower);
    upperHalfX.swap(xUpper);
    upperHalfY.swap(yUpper);
    return true;
}

void AirfoilData::sortDataClockwise(std::vector<double>& xCoordinates, std::vector<double>& yCoordinates)
{
    // Reverse the contour when the first branch has the greater mean ordinate.

    if ((xCoordinates.size() != yCoordinates.size()) || (yCoordinates.size() < 3))
    {
        return;
    }

    std::size_t noseIndex;
    if (!getNoseIndex(xCoordinates, yCoordinates, NoseDetectionMode::LongestSecant, noseIndex))
    {
        return;
    }

    const std::size_t pointCount = yCoordinates.size();
    if ((noseIndex < 1) || (noseIndex + 2 > pointCount))
    {
        return;
    }

    double sumFirstSide = 0.0;
    for (std::size_t i = 0; i <= noseIndex; ++i)
    {
        sumFirstSide += yCoordinates[i];
    }
    double sumSecondSide = 0.0;
    for (std::size_t i = noseIndex; i < pointCount; ++i)
    {
        sumSecondSide += yCoordinates[i];
    }

    const double meanFirstSide = sumFirstSide / (static_cast<double>(noseIndex) + 1.0);
    const double meanSecondSide = sumSecondSide /
        static_cast<double>(pointCount - noseIndex);

    if (meanFirstSide >= meanSecondSide)
    {
        reorderData(xCoordinates, yCoordinates);
    }

    return;
}

void AirfoilData::reorderData(std::vector<double>& xCoordinates, std::vector<double>& yCoordinates)
{
    if (xCoordinates.size() != yCoordinates.size())
    {
        return;
    }

    std::reverse(xCoordinates.begin(), xCoordinates.end());
    std::reverse(yCoordinates.begin(), yCoordinates.end());

    return;
}

std::vector<double> AirfoilData::createXDistribution(std::size_t pointCount)
{
    return createXDistribution(pointCount, XDistributionMode::HalfCosine);
}

std::vector<double> AirfoilData::createXDistribution(std::size_t pointCount, XDistributionMode mode)
{
    if (false == pointCountInAllowedRange(pointCount))
    {
        return std::vector<double>();
    }
    return createXDistributionUnchecked(pointCount, mode);
}

std::vector<double> AirfoilData::createXDistributionUnchecked(std::size_t pointCount)
{
    return createXDistributionUnchecked(pointCount, XDistributionMode::HalfCosine);
}

std::vector<double> AirfoilData::createXDistributionUnchecked(std::size_t pointCount, XDistributionMode mode)
{
    if (pointCount < 2)
    {
        return std::vector<double>();
    }

    std::vector<double> xDistribution(pointCount);

    if (XDistributionMode::HalfCosine == mode)
    {
        // Half-cosine distribution: leading-edge clustering.
        const double dPhi = AirfoilDataConstants::kPi / static_cast<double>(pointCount - 1);
        for (std::size_t i = 0; i < pointCount; i++)
        {
            const double phi = static_cast<double>(i) * dPhi;
            xDistribution[i] = 1.0 + std::cos(phi + 0.5 * AirfoilDataConstants::kPi);
        }
    }
    else
    {
        // Full cosine distribution: leading- and trailing-edge clustering.
        const double dPhi = (2 * AirfoilDataConstants::kPi) / static_cast<double>(pointCount - 1);
        for (std::size_t i = 0; i < pointCount; i++)
        {
            const double phi = static_cast<double>(i) * dPhi;
            xDistribution[i] = 0.5 * (1.0 + std::cos(phi));
        }
    }

    return xDistribution;
}

bool AirfoilData::sumSupportPointArea(const std::vector<double>& areaPointsX,
                                      const std::vector<double>& areaPointsY,
                                      double& calculatedArea)
{
    const std::string methodName = "sumSupportPointArea(): ";

    if ((areaPointsX.size() != areaPointsY.size()) ||
        (areaPointsX.size() < 4) ||
        ((areaPointsX.size() % 2) != 0))
    {
        emitOutput(methodName + "support point vectors invalid",
                   AirfoilDataMessageType::Error);
        return false;
    }
    for (std::size_t i = 0; i < areaPointsX.size(); ++i)
    {
        if (!std::isfinite(areaPointsX[i]) || !std::isfinite(areaPointsY[i]))
        {
            emitOutput(methodName + "support points not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
    }

    const std::size_t nPts = areaPointsX.size();
    const std::size_t halfPts = nPts / 2;
    double a = 0.0;

    // Calculating calculatedArea by summing up individual areas between points
    for (std::size_t j = 0; j + 1 < halfPts; j++)
    {
        // Usage of std::fabs() is necessary in case the airfoil data drops
        // (seen from right to left in nose calculatedArea)
        a += ((areaPointsY[nPts - j - 1] - areaPointsY[j]) *
              (areaPointsX[j] - areaPointsX[j + 1])); // rectangular calculatedArea
        a += (0.5 * std::fabs(areaPointsY[nPts - j - 2] - areaPointsY[nPts - j - 1]) *
              (areaPointsX[nPts - j - 1] - areaPointsX[nPts - j - 2])); // Upper triangle
        a += (0.5 * std::fabs(areaPointsY[j + 1] - areaPointsY[j])) *
              (areaPointsX[j] - areaPointsX[j + 1]); // Lower triangle
    }

    if (!std::isfinite(a))
    {
        emitOutput(methodName + "result not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }

    calculatedArea = a;
    return true;
}

bool AirfoilData::calculateArea(const std::vector<double>& xCoordinates, const std::vector<double>& yCoordinates,
                                double& calculatedArea)
{
    const std::string methodName = "calculateArea(): ";

    const std::size_t nPts = AirfoilDataConstants::kAreaSupportPoints;  // Has to be even
    const std::size_t halfPts = nPts / 2;

    if (xCoordinates.size() != yCoordinates.size())
    {
        emitOutput(methodName + "x/y have different sizes",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::size_t noseIndex;
    if (!getNoseIndex(xCoordinates, yCoordinates, NoseDetectionMode::LocalMinimum, noseIndex) ||
        !isInteriorNose(xCoordinates, noseIndex))
    {
        emitOutput(methodName + "no inner leading edge found",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::vector<double> areaPointsX(nPts);
    std::vector<double> areaPointsY(nPts);
    std::vector<double> localX(xCoordinates);

    // Evaluate the contour on a dense full-cosine grid.
    const double dPhi = (2 * AirfoilDataConstants::kPi) / (nPts - 1);
    for (std::size_t i = 0; i < nPts; i++)
    {
        const double phi = static_cast<double>(i) * dPhi;
        areaPointsX[i] = 0.5 * (1.0 + std::cos(phi));
    }

    // Transform the lower branch to obtain one ascending interpolation grid.
    for (std::size_t j = 0; j < noseIndex; j++)
    {
        localX[j] = -localX[j];
    }

    for (std::size_t j = 0; j < halfPts; j++)
    {
        areaPointsX[j] = -areaPointsX[j];
    }

    AkimaInterpolation akima;
    if (false == akima.init(localX, yCoordinates))
    {
        emitOutput(methodName + "Akima initialization failed",
                   AirfoilDataMessageType::Error);
        return false;
    }
    for (std::size_t j = 0; j < nPts; j++)
    {
        if (!akima.interpolate(areaPointsX[j], areaPointsY[j]))
        {
            emitOutput(methodName + "Akima interpolation failed",
                       AirfoilDataMessageType::Error);
            return false;
        }
    }

    // Restore ordinary chordwise coordinates before area summation.
    for (std::size_t j = 0; j < halfPts; j++)
    {
        areaPointsX[j] = -areaPointsX[j];
    }

    return sumSupportPointArea(areaPointsX, areaPointsY, calculatedArea);
}

bool AirfoilData::scale(const std::vector<double>& xIn, const std::vector<double>& yIn,
                        std::vector<double>& xOut, std::vector<double>& yOut,
                        double xMin, double xMax)
{
    const std::string methodName = "scale(): ";
    const std::size_t n = xIn.size();
    if ((yIn.size() != n) || (xOut.size() != n) || (yOut.size() != n))
    {
        emitOutput(methodName + "dimensions of xIn/yIn/xOut/yOut not consistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (&xOut == &yOut)
    {
        emitOutput(methodName + "xOut and yOut are the same vector",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (0 == n)
    {
        emitOutput(methodName + "no input points",
                   AirfoilDataMessageType::Error);
        return false;
    }

    for (std::size_t i = 0; i < n; i++)
    {
        if (!std::isfinite(xIn[i]) || !std::isfinite(yIn[i]))
        {
            emitOutput(methodName + "input values not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
    }

    double minValue = xIn[0];
    double maxValue = xIn[0];
    for (std::size_t i = 1; i < n; i++)
    {
        if (xIn[i] < minValue)
        {
            minValue = xIn[i];
        }
        if (xIn[i] > maxValue)
        {
            maxValue = xIn[i];
        }
    }
    const double length = maxValue - minValue;
    if (!std::isfinite(length) || 0.0 == length ||
        !std::isfinite(xMin) || !std::isfinite(xMax))
    {
        emitOutput(methodName + "invalid scaling range",
                   AirfoilDataMessageType::Error);
        return false;
    }

    const double targetLength = xMax - xMin;
    if (!std::isfinite(targetLength))
    {
        emitOutput(methodName + "target interval not representable",
                   AirfoilDataMessageType::Error);
        return false;
    }
    const double factor = targetLength / length;
    if (!std::isfinite(factor))
    {
        emitOutput(methodName + "scaling factor not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }

    // Calculate into temporary vectors so the outputs remain unchanged on
    // failure.
    std::vector<double> scaledX(n);
    std::vector<double> scaledY(n);
    for (std::size_t i = 0; i < n; i++)
    {
        const double inputDifference = xIn[i] - minValue;
        const double resultX = xMin + inputDifference * factor;
        const double resultY = yIn[i] * factor;
        if (!std::isfinite(resultX) || !std::isfinite(resultY))
        {
            emitOutput(methodName + "result not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
        scaledX[i] = resultX;
        scaledY[i] = resultY;
    }
    xOut.swap(scaledX);
    yOut.swap(scaledY);
    return true;
}

bool AirfoilData::rotate(const std::vector<double>& xIn, const std::vector<double>& yIn,
                         std::vector<double>& xOut, std::vector<double>& yOut,
                         double angleDeg)
{
    // Keep radians internal; all public AirfoilData angles use degrees.
    return rotateRadians(xIn, yIn, xOut, yOut,
                         angleDeg * AirfoilDataConstants::kPi / 180.0);
}

bool AirfoilData::rotateRadians(const std::vector<double>& xIn, const std::vector<double>& yIn,
                                std::vector<double>& xOut, std::vector<double>& yOut,
                                double angle)
{
    const std::string methodName = "rotate(): ";
    const std::size_t n = xIn.size();
    if ((yIn.size() != n) || (xOut.size() != n) || (yOut.size() != n))
    {
        emitOutput(methodName + "dimensions of xIn/yIn/xOut/yOut not consistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (&xOut == &yOut)
    {
        emitOutput(methodName + "xOut and yOut are the same vector",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (!std::isfinite(angle))
    {
        emitOutput(methodName + "rotation angle not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }

    for (std::size_t i = 0; i < n; i++)
    {
        if (!std::isfinite(xIn[i]) || !std::isfinite(yIn[i]))
        {
            emitOutput(methodName + "input coordinate not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
    }

    double sina = std::sin(angle);
    double cosa = std::cos(angle);

    // Calculate into temporary vectors so the outputs are committed together.
    std::vector<double> rotatedX(n);
    std::vector<double> rotatedY(n);
    for (std::size_t i = 0; i < n; i++)
    {
        const double xi = xIn[i];
        const double yi = yIn[i];
        const double resultX = xi * cosa + yi * sina;
        const double resultY = -xi * sina + yi * cosa;
        if (!std::isfinite(resultX) || !std::isfinite(resultY))
        {
            emitOutput(methodName + "result not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
        rotatedX[i] = resultX;
        rotatedY[i] = resultY;
    }
    xOut.swap(rotatedX);
    yOut.swap(rotatedY);
    return true;
}

bool AirfoilData::normalizeAirfoil(const std::vector<double>& xIn, const std::vector<double>& yIn,
                                   std::vector<double>& xOut, std::vector<double>& yOut,
                                   SortMode sort, InterpolationMode interpolate, double& rotatedAngle)
{
    const std::string methodName = "normalizeAirfoil(): ";

    if (&xOut == &yOut)
    {
        emitOutput(methodName + "xOut and yOut must be different",
                   AirfoilDataMessageType::Error);
        return false;
    }

    const std::size_t nIn = xIn.size();
    const std::size_t nOut = (InterpolationMode::Map == interpolate) ? xOut.size() : nIn;
    std::size_t nInSize = nIn; // Possibly reduced after x-hook repair
    const std::size_t nOutSize = nOut;

    if (xIn.size() != yIn.size())
    {
        emitOutput(methodName + "xIn/yIn have different sizes", AirfoilDataMessageType::Error);
        return false;
    }
    if (xOut.size() != yOut.size())
    {
        emitOutput(methodName + "xOut/yOut have different sizes", AirfoilDataMessageType::Error);
        return false;
    }
    if ((InterpolationMode::None == interpolate) && (xOut.size() != nIn))
    {
        emitOutput(methodName + "without interpolation the output size must equal the input size",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (nIn < AirfoilDataConstants::kMinNormalizeInputPoints)
    {
        emitOutput(joinMessage(methodName, "too few points (at least ",
                               AirfoilDataConstants::kMinNormalizeInputPoints, " required)"),
                   AirfoilDataMessageType::Error);
        return false;
    }

    if ((InterpolationMode::Map == interpolate) &&
        (xOut.size() < AirfoilDataConstants::kMinTargetSupportPoints))
    {
        emitOutput(joinMessage(methodName, "at least ",
                               AirfoilDataConstants::kMinTargetSupportPoints,
                               " target support points required"),
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::vector<double> xBuffer(nInSize);
    std::vector<double> xBuffer2(nInSize);
    std::vector<double> yBuffer(nInSize);
    std::vector<double> yBuffer2(nInSize);
    double rotationAlpha, yNose;
    const double eps = AirfoilDataConstants::kZeroCrossingEpsilon;
    double dydx;
    bool clockwise;
    std::size_t noseIndex, noseIndex2;
    // The trailing edge counts as closed when the first and the last input point
    // (lower and upper TE) have the same y value. Only then may the TE alignment bring
    // both TE points to the same y value; for a real (open) trailing edge
    // with thickness the difference is preserved.
    const bool trailingEdgeClosed = (yIn[0] == yIn[nInSize - 1]);

    if (verbosityLevel > 0)
    {
        emitOutput(joinMessage(methodName, "nIn=", nIn,
                   " nOut=", nOut, " sort=", (SortMode::Sort == sort) ? 1 : 0,
                   " interpolate=", (InterpolationMode::Map == interpolate) ? 1 : 0),
                   AirfoilDataMessageType::Info);
    }

    // scaling onto interval [0,1]
    if (verbosityLevel > 0)
    {
        emitOutput(methodName + "scaling", AirfoilDataMessageType::Info);
    }

    if (false == scale(xIn, yIn, xBuffer, yBuffer, 0.0, 1.0))
    {
        return false;
    }

    // Detect whether the contour already starts on the lower surface.
    clockwise = false;
    if (yBuffer[nInSize - 11] > yBuffer[10])
    {
        clockwise = true;
    }

    if ((false == clockwise) && (SortMode::Sort == sort))
    {
        // Reordering data, beginning from
        for (std::size_t i = 0; i < nInSize; i++) {
            // Trailing edge lower side
            xBuffer2[nInSize - i - 1] = xBuffer[i];
            yBuffer2[nInSize - i - 1] = yBuffer[i];
        }
    }
    else
    {
        for (std::size_t i = 0; i < nInSize; i++)
        {
            xBuffer2[i] = xBuffer[i];
            yBuffer2[i] = yBuffer[i];
        }
    }

    // Translating by yNose onto x-axis
    if (!getNoseIndex(xBuffer2, yBuffer2, NoseDetectionMode::LongestSecant, noseIndex))
    {
        emitOutput(methodName + "no inner leading edge found", AirfoilDataMessageType::Error);
        return false;
    }

    if (!isSecantNose(xBuffer2, noseIndex))
    {
        emitOutput(methodName + "no inner leading edge found", AirfoilDataMessageType::Error);
        return false;
    }
    yNose = yBuffer2[noseIndex];

    for (std::size_t i = 0; i < nInSize; i++)
    {
        yBuffer2[i] -= yNose;
    }

    // Move the detected leading edge exactly to the origin before rotation.
    for (std::size_t i = 0; i < nInSize; i++)
    {
        xBuffer2[i] -= xBuffer2[noseIndex];
    }

    // Rotate to 0 degree incidence
    if (std::fabs(yBuffer2[nInSize - 1] - yBuffer2[0]) < eps)
    {
        // Trailing edge points are approximately equal on upper and lower side
        rotationAlpha = std::atan(yBuffer2[0] / xBuffer2[0]);
    }
    else
    {
        // Use the midpoint of an open trailing edge as the chord endpoint.
        rotationAlpha = std::atan((yBuffer2[0] + (yBuffer2[nInSize - 1] - yBuffer2[0]) / 2.0) / xBuffer2[0]);
    }

    const double calculatedRotatedAngle = -180.0 * rotationAlpha / AirfoilDataConstants::kPi;

    if (verbosityLevel > 0)
    {
        emitOutput(methodName + "rotating, angle = " +
                   std::to_string(180.0 * rotationAlpha / AirfoilDataConstants::kPi) +
                   std::string(" Deg."), AirfoilDataMessageType::Info);
    }

    if (false == rotateRadians(xBuffer2, yBuffer2, xBuffer, yBuffer, rotationAlpha))
    {
        return false;
    }

    // Rescale after rotation to remove accumulated numerical drift.
    if (false == scale(xBuffer, yBuffer, xBuffer2, yBuffer2, 0.0, 1.0))
    {
        return false;
    }

    // Remove very small branch-wise x reversals before interpolation.
    std::size_t removedHooks = 0;
    if (!repairXHooks(xBuffer2, yBuffer2, removedHooks))
    {
        emitOutput(methodName + "x-hooks in the contour not repairable (self-overlap)",
                   AirfoilDataMessageType::Warning);
        return false;
    }
    if (removedHooks > 0)
    {
        emitOutput(methodName +
                   std::to_string(removedHooks) +
                   std::string(" non-consecutive points removed"),
                   AirfoilDataMessageType::Warning);
        nInSize = xBuffer2.size();
    }

    // Extend both trailing-edge endpoints to x = 1 when necessary.
    if (xBuffer2[0] < 1.0)
    {
        if (verbosityLevel > 0)
        {
            emitOutput(methodName + "adjusting lower trailing edge point", AirfoilDataMessageType::Info);
        }
        dydx = (yBuffer2[1] - yBuffer2[0]) / (xBuffer2[1] - xBuffer2[0]);
        xBuffer2[0] = 1.0;
        yBuffer2[0] = yBuffer2[1] + dydx * (xBuffer2[0] - xBuffer2[1]);
    }

    if (xBuffer2[nInSize - 1] < 1.0)
    {
        if (verbosityLevel > 0)
        {
            emitOutput(methodName + "adjusting upper trailing edge point", AirfoilDataMessageType::Info);
        }
        dydx = (yBuffer2[nInSize - 2] - yBuffer2[nInSize - 1]) / (xBuffer2[nInSize - 2] - xBuffer2[nInSize - 1]);
        xBuffer2[nInSize - 1] = 1.0;
        yBuffer2[nInSize - 1] = yBuffer2[nInSize - 2] + dydx * (xBuffer2[nInSize - 1] - xBuffer2[nInSize - 2]);
    }

    std::vector<double> normalizedX;
    std::vector<double> normalizedY;

    // Mapping onto desired x support points using akima interpolation
    if (InterpolationMode::Map == interpolate)
    {
        if (verbosityLevel > 0)
        {
            emitOutput(methodName + "interpolating", AirfoilDataMessageType::Info);
        }

        // In Map mode, xOut initially contains the target support coordinates.
        xBuffer.resize(nOutSize);
        yBuffer.resize(nOutSize);

        // Prepare for interpolation, make sure only ascending x-values
        std::size_t splitIndex;
        if (!getNoseIndex(xBuffer2, yBuffer2, NoseDetectionMode::LocalMinimum, splitIndex))
        {
            emitOutput(methodName + "target support points invalid",
                       AirfoilDataMessageType::Error);
            return false;
        }
        for (std::size_t i = 0; i < nInSize; i++)
        {
            if (i < splitIndex)
            {
                xBuffer2[i] = -xBuffer2[i];
            }
        }

        if (!getNoseIndex(xOut, xOut, NoseDetectionMode::LocalMinimum, noseIndex2))
        {
            emitOutput(methodName + "target support points invalid",
                       AirfoilDataMessageType::Error);
            return false;
        }

        for (std::size_t i = 0; i < nOutSize; i++)
        {
            if (i < noseIndex2)
            {
                xBuffer[i] = -xOut[i];
            }
            else
            {
                xBuffer[i] = xOut[i];
            }
        }

        // Now doing akima
        AkimaInterpolation akima;
        if (false == akima.init(xBuffer2, yBuffer2))
        {
            emitOutput(methodName + "Akima initialization failed",
                       AirfoilDataMessageType::Error);
            return false;
        }
        for (std::size_t i = 0; i < nOutSize; i++)
        {
            if (!akima.interpolate(xBuffer[i], yBuffer[i]))
            {
                emitOutput(methodName + "Akima interpolation failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
            if (verbosityLevel > 1)
            {
                // xBuffer2/yBuffer2 hold only nInSize elements (possibly reduced
                // by the x-hook repair); the source-point columns are emitted
                // only for indices that exist in them (nOutSize may be larger).
                std::string sourcePart;
                if ((i < xBuffer2.size()) && (i < yBuffer2.size()))
                {
                    sourcePart = std::string("\t") + std::to_string(xBuffer2[i]) +
                                 std::string("\t") + std::to_string(yBuffer2[i]);
                }
                emitOutput(methodName + std::to_string(i) + std::string("\t") +
                           std::to_string(xBuffer[i]) + std::string("\t") + std::to_string(yBuffer[i]) +
                           sourcePart, AirfoilDataMessageType::Info);
            }
        }

        // Again scaling onto interval [0,1], because of possible small interpolation errors
        const std::vector<double> targetX = xOut;
        normalizedX.resize(nOutSize);
        normalizedY.resize(nOutSize);
        if (false == scale(targetX, yBuffer, normalizedX, normalizedY, 0.0, 1.0))
        {
            return false;
        }

        // Set trailing edge points equal to overcome possible rounding error during normalization
        if (trailingEdgeClosed || (std::fabs(normalizedY[0] - normalizedY[nOutSize - 1]) < eps))
        {
            normalizedY[0] = normalizedY[nOutSize - 1];
        }
    }
    else
    {
        // Again scaling onto interval [0,1] after rotation
        normalizedX.resize(nInSize);
        normalizedY.resize(nInSize);
        if (false == scale(xBuffer2, yBuffer2, normalizedX, normalizedY, 0.0, 1.0))
        {
            return false;
        }

        // Set trailing edge points equal to overcome possible rounding errors during normalization
        if (trailingEdgeClosed || (std::fabs(normalizedY[0] - normalizedY[nInSize - 1]) < eps))
        {
            normalizedY[0] = normalizedY[nInSize - 1];
        }
    }

    // Eliminate possible double x values
    const std::size_t normalizedSize = normalizedX.size();
    for (std::size_t i = 0; i + 1 < normalizedSize; i++)
    {
        if (std::fabs(normalizedX[i] - normalizedX[i + 1]) < AirfoilDataConstants::kGradientEpsilon)
        {
            if ((i + 1) != (nOutSize - 1))
            {
                double dx = (normalizedX[i + 2] - normalizedX[i + 1]);
                double dy = (normalizedY[i + 2] - normalizedY[i + 1]);
                normalizedX[i + 1] = normalizedX[i + 1] + 0.5 * dx;
                normalizedY[i + 1] = normalizedY[i + 1] + 0.5 * dy;
            }
            else
            {
                double dx = (normalizedX[i - 1] - normalizedX[1]);
                double dy = (normalizedY[i - 1] - normalizedY[1]);
                normalizedX[i] = normalizedX[i] + 0.5 * dx;
                normalizedY[i] = normalizedY[i] + 0.5 * dy;
            }
        }
    }

    if (!allFiniteValues(normalizedX) || !allFiniteValues(normalizedY))
    {
        emitOutput(methodName + "result not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }

    xOut.swap(normalizedX);
    yOut.swap(normalizedY);

    if (verbosityLevel > 0)
    {
        emitOutput(methodName + "returning", AirfoilDataMessageType::Info);
    }

    rotatedAngle = calculatedRotatedAngle;

    return true;
}

bool AirfoilData::calculateAirfoilGeometryData()
{
    const std::string methodName = "calculateAirfoilGeometryData(): ";
    double thicknessValue = 0.0;
    double thicknessLocation = 0.0;
    double dyNoseValue = 0.0;
    double areaValue = 0.0;
    double camberValue = 0.0;
    double camberLocation = 0.0;
    double dummy = 0.0;

    const std::vector<double>& profileX = getAirfoilX();
    const std::vector<double>& profileY = getAirfoilY();

    if (false == calculateThickness(profileX, profileY, ThicknessMode::Absolute, 0.0, thicknessValue,
                                    dummy, thicknessLocation, SideMode::TwoSided))
    {
        emitOutput(methodName + "thickness calculation failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (false == calculateDyNose(profileX, profileY, dyNoseValue))
    {
        emitOutput(methodName + "dyNose calculation failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (false == calculateArea(profileX, profileY, areaValue))
    {
        emitOutput(methodName + "area calculation failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    // Determining camber
    const std::vector<double>& camberLineX = getCamberLineX();
    const std::vector<double>& camberLineY = getCamberLineY();
    const std::size_t nP = camberLineX.size();

    if (nP != camberLineY.size())
    {
        emitOutput(methodName + "number of camber points not consistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (false == calculateThickness(camberLineX, camberLineY, ThicknessMode::Absolute, 0.0,
                                    camberValue, dummy, camberLocation, SideMode::OneSided))
    {
        emitOutput(methodName + "camber calculation failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    // Determine the sign from the camber-line ordinate at the location of the
    // maximum absolute camber.
    double camberSign = 1.0;
    if (camberValue > 0.0)
    {
        AkimaInterpolation camberAkima;

        if (!camberAkima.init(camberLineX, camberLineY))
        {
            emitOutput(methodName + "Akima initialization of the camber line failed",
                       AirfoilDataMessageType::Error);
            return false;
        }

        double yAtMaximum = 0.0;
        if (!camberAkima.interpolate(camberLocation, yAtMaximum))
        {
            emitOutput(methodName + "Akima interpolation of the camber line failed",
                       AirfoilDataMessageType::Error);
            return false;
        }

        if (yAtMaximum < 0.0)
        {
            camberSign = -1.0;
        }
    }

    // For nearly symmetric airfoils, suppress the numerically meaningless
    // camber location.
    if (camberValue < 0.001 * thicknessValue)
    {
        camberLocation = 0.0;
    }

    // Commit the scalar geometry only after all calculations succeed.
    thickness = thicknessValue;
    maxThicknessLocation = thicknessLocation;
    dyNose = dyNoseValue;
    area = areaValue;
    camber = camberSign * camberValue;
    maxCamberPosition = camberLocation;

    calculateTeThickness();
    return true;
}

bool AirfoilData::calculateAirfoilProperties()
{
    const std::string methodName = "calculateAirfoilProperties(): ";

    // Preserve the complete previous state for transactional rollback.
    AirfoilData snapshot = *this;

    if (false == calculateAirfoilCamberAndThickness())
    {
        emitOutput(std::string("calculation of camber and thickness distribution failed"),
                   AirfoilDataMessageType::Error);
        *this = std::move(snapshot);
        return false;
    }

    // Determining properties
    if (!calculateAirfoilGeometryData())
    {
        // CalculateAirfoilGeometryData() already emitted the error message.
        *this = std::move(snapshot);
        return false;
    }

    const std::vector<double>& camberLineX = getCamberLineX();
    const std::vector<double>& camberLineY = getCamberLineY();
    std::size_t nP = camberLineX.size();

    if ((nP < 1) || (nP != camberLineY.size()))
    {
        emitOutput(methodName + "number of camber points not consistent",
                   AirfoilDataMessageType::Error);
        *this = std::move(snapshot);
        return false;
    }

    std::vector<double> xCirculation;
    std::vector<double> circulationStrength;

    if (!this->calculateCirculationCl05(camberLineX, camberLineY, xCirculation, circulationStrength))
    {
        emitOutput(methodName + "circulation calculation failed",
                   AirfoilDataMessageType::Error);
        *this = std::move(snapshot);
        return false;
    }
    xCirculation.resize(nP - 1);
    circulationStrength.resize(nP - 1);

    const std::vector<double>& thicknessX = getThicknessDistributionX();
    const std::vector<double>& thicknessY = getThicknessDistributionY();
    nP = thicknessX.size();

    if (nP != thicknessY.size())
    {
        emitOutput(methodName + "number of thickness points not consistent",
                   AirfoilDataMessageType::Error);
        *this = std::move(snapshot);
        return false;
    }

    std::size_t in;
    if (!getNoseIndex(thicknessX, thicknessY, NoseDetectionMode::LocalMinimum, in) ||
        !isInteriorNose(thicknessX, in))
    {
        emitOutput(methodName + "nose index in the thickness distribution invalid",
                   AirfoilDataMessageType::Error);
        *this = std::move(snapshot);
        return false;
    }

    std::vector<double> xThickness(thicknessX.begin() + static_cast<std::ptrdiff_t>(in), thicknessX.end());
    std::vector<double> yThickness(thicknessY.begin() + static_cast<std::ptrdiff_t>(in), thicknessY.end());
    std::vector<double> xq;
    std::vector<double> q;

    if (!this->calculateSourceStrength(xThickness, yThickness, xq, q))
    {
        emitOutput(methodName + "source strength calculation failed",
                   AirfoilDataMessageType::Error);
        *this = std::move(snapshot);
        return false;
    }

    if (!calculateUpperLowerCoordinates())
    {
        // calculateUpperLowerCoordinates() already emitted the error message.
        *this = std::move(snapshot);
        return false;
    }

    if (!calculateGradientCurvature())
    {
        // calculateGradientCurvature() already emitted the error message.
        *this = std::move(snapshot);
        return false;
    }

    // Commit circulation and scaled source data only after all remaining
    // calculations succeed.
    circX.swap(xCirculation);
    circY.swap(circulationStrength);
    sourceX.swap(xq);
    sourceY.swap(q);

    return true;
}

bool AirfoilData::redistributeAirfoil(std::size_t nPointsNew, RedistributeMode mode)
{
    const std::string methodName = "redistributeAirfoil(): ";
    std::size_t noseIndex, noseIndex2;
    double rotation;

    const std::vector<double>& profileX = getAirfoilX();
    const std::vector<double>& profileY = getAirfoilY();

    if (false == validatePointCount(nPointsNew))
    {
        emitOutput(methodName + "invalid point count (99..511, odd required)",
                   AirfoilDataMessageType::Error);
        emitOutput(joinMessage(methodName, "nPointsNew=", nPointsNew),
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (profileX.size() != profileY.size())
    {
        emitOutput(methodName + "x/y not consistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (profileX.size() <= 11)
    {
        emitOutput(methodName + "too few old points for a redistribution (at least 12)",
                   AirfoilDataMessageType::Error);
        emitOutput(joinMessage(methodName, "nPointsOld=", profileX.size()),
                   AirfoilDataMessageType::Error);
        return false;
    }

    const std::size_t nNewSize = nPointsNew;       // nPointsNew >= 11 checked
    const std::size_t nOldSize = profileX.size();  // profileX.size() > 11 checked

    if (!getNoseIndex(profileX, profileY, NoseDetectionMode::LocalMinimum, noseIndex) ||
        !isInteriorNose(profileX, noseIndex))
    {
        emitOutput(methodName + "no inner leading edge found",
                   AirfoilDataMessageType::Error);
        return false;
    }
    const std::size_t noseSize = noseIndex;

    std::vector<double> oldX(nOldSize);
    std::vector<double> newX(nNewSize);
    std::vector<double> newY(nNewSize);
    std::vector<double> newAirfoilX(nNewSize);
    std::vector<double> newAirfoilY(nNewSize);

    newAirfoilX = createXDistribution(nNewSize);

    for (std::size_t i = 0; i < nOldSize; i++)
    {
        if (i < noseSize)
        {
            oldX[i] = -profileX[i];
        }
        else
        {
            oldX[i] = profileX[i];
        }
    }

    if (!getNoseIndex(newAirfoilX, newAirfoilY, NoseDetectionMode::LocalMinimum, noseIndex2))
    {
        emitOutput(methodName + "internal logic: no nose found in the cosine distribution",
                   AirfoilDataMessageType::Error);
        return false;
    }
    const std::size_t nose2Size = noseIndex2;
    for (std::size_t i = 0; i < nNewSize; i++)
    {
        if (i < nose2Size)
        {
            newX[i] = -newAirfoilX[i];
        }
        else
        {
            newX[i] = newAirfoilX[i];
        }
    }

    // Now doing akima
    AkimaInterpolation akima;
    if (!akima.init(oldX, profileY))
    {
        emitOutput(methodName + "Akima initialization failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    for (std::size_t i = 0; i < nNewSize; i++)
    {
        if (!akima.interpolate(newX[i], newY[i]))
        {
            emitOutput(methodName + "Akima interpolation failed",
                       AirfoilDataMessageType::Error);
            return false;
        }
    }

    for (std::size_t i = 0; i < nose2Size; i++)
    {
        newX[i] = -newX[i];
    }

    if (!this->normalizeAirfoil(newX, newY, newAirfoilX, newAirfoilY, SortMode::Sort,
                                InterpolationMode::None, rotation))
    {
        emitOutput(methodName + "normalization failed",
                   AirfoilDataMessageType::Error);
        return false;
    }
    if (std::fabs(rotation) > 1.0e-2)
    {
        std::ostringstream angleStream;
        angleStream.width(6);
        angleStream.precision(2);
        angleStream << std::fixed << rotation;
        emitOutput(std::string("Airfoil rotated by ") + angleStream.str() + std::string(" degrees"),
                   AirfoilDataMessageType::Normal);
    }

    if (RedistributeMode::AirfoilOnly == mode)
    {
        if (!setAirfoilCoordinates(std::move(newAirfoilX), std::move(newAirfoilY)))
        {
            emitOutput(methodName + "airfoil data inconsistent",
                       AirfoilDataMessageType::Error);
            return false;
        }
        clearCalculatedGeometryData();
        return true;
    }

    AirfoilData updated(*this);
    if (!updated.setAirfoilCoordinates(std::move(newAirfoilX), std::move(newAirfoilY)))
    {
        emitOutput(methodName + "airfoil data inconsistent",
                   AirfoilDataMessageType::Error);
        return false;
    }
    if (!updated.calculateAirfoilProperties())
    {
        // calculateAirfoilProperties() already emitted the error message.
        return false;
    }

    *this = std::move(updated);
    return true;
}

//*****************************
// Camber and thickness methods
//*****************************

void AirfoilData::calculateTeThickness()
{
    const std::string methodName = "calculateTeThickness(): ";
    const std::vector<double>& xa = getAirfoilX();
    const std::vector<double>& ya = getAirfoilY();

    if (ya.empty() || (xa.size() != ya.size()))
    {
        emitOutput(methodName + "no consistent airfoil data",
                   AirfoilDataMessageType::Error);
        return;
    }

    double t = ya[ya.size() - 1] - ya[0];
    setTeThickness(t);
}

bool AirfoilData::repairXHooks(std::vector<double>& xCoordinates, std::vector<double>& yCoordinates,
                               std::size_t& removedCount)
{
    // Small x reversals can occur in highly nonlinear geometry generators.
    // Remove violating points until both contour branches are strictly monotone.
    const std::string methodName = "repairXHooks(): ";
    if (xCoordinates.size() < 3 || yCoordinates.size() != xCoordinates.size())
    {
        emitOutput(methodName + "x/y not consistent or too few points",
                   AirfoilDataMessageType::Error);
        removedCount = 0;
        return false;
    }

    std::size_t noseMin = 0;
    for (std::size_t i = 1; i < xCoordinates.size(); i++)
    {
        if (xCoordinates[i] < xCoordinates[noseMin])
        {
            noseMin = i;
        }
    }

    std::size_t removedLocal = 0;
    bool repaired = true;
    while (true == repaired)
    {
        repaired = false;
        // Lower branch (0..noseMin): x must be strictly decreasing
        for (std::size_t i = 1; i <= noseMin; i++)
        {
            if (xCoordinates[i] >= xCoordinates[i - 1])
            {
                xCoordinates.erase(xCoordinates.begin() + static_cast<std::ptrdiff_t>(i));
                yCoordinates.erase(yCoordinates.begin() + static_cast<std::ptrdiff_t>(i));
                removedLocal++;
                noseMin--;
                repaired = true;
                break;
            }
        }
        if (true == repaired)
        {
            continue;
        }
        // Upper branch (noseMin..end): x must be strictly increasing
        for (std::size_t i = noseMin + 1; i < xCoordinates.size(); i++)
        {
            if (xCoordinates[i] <= xCoordinates[i - 1])
            {
                xCoordinates.erase(xCoordinates.begin() + static_cast<std::ptrdiff_t>(i));
                yCoordinates.erase(yCoordinates.begin() + static_cast<std::ptrdiff_t>(i));
                removedLocal++;
                repaired = true;
                break;
            }
        }
        if (removedLocal > 20)
        {
            emitOutput(methodName + "x-hooks in the airfoil region not "
                                   "repairable (more than 20 points violate)",
                       AirfoilDataMessageType::Error);
            removedCount = 0;
            return false;
        }
    }
    removedCount = removedLocal;
    return true;
}

bool AirfoilData::checkCamberline(std::vector<double>& xCamberLine, std::vector<double>& yCamberLine,
                                  std::size_t& remainingCount)
{
    // Remove decreasing ordinates among the first few camber-line points.
    const std::string methodName = "checkCamberline(): ";

    if (xCamberLine.size() != yCamberLine.size())
    {
        emitOutput(methodName + "xCamberLine/yCamberLine have different sizes",
                   AirfoilDataMessageType::Error);
        remainingCount = 0;
        return false;
    }

    std::size_t n = xCamberLine.size();
    std::size_t k = 0;

    for (std::size_t i = 1; i < 10 && i < n; i++)  // Check only first few points
    {
        std::size_t j = i - k;

        if (yCamberLine[j] < yCamberLine[j - 1])  // Remove non monotone point
        {
            for (std::size_t shiftIndex = j; shiftIndex + 1 < n; shiftIndex++)
            {
                xCamberLine[shiftIndex] = xCamberLine[shiftIndex + 1];
                yCamberLine[shiftIndex] = yCamberLine[shiftIndex + 1];
            }
            k++;
            n--;
        }
    }
    xCamberLine.resize(n);
    yCamberLine.resize(n);
    remainingCount = n;
    return true;
}

bool AirfoilData::setAirfoilCoordinates(std::vector<double> xCoordinates,
                                        std::vector<double> yCoordinates)
{
    const std::string methodName = "setAirfoilCoordinates(): ";

    if (xCoordinates.size() != yCoordinates.size())
    {
        emitOutput(methodName + "x/y sizes inconsistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    airfoilX.swap(xCoordinates);
    airfoilY.swap(yCoordinates);
    return true;
}

bool AirfoilData::calculateAirfoilCamberAndThickness()
{
    const std::string methodName = "calculateAirfoilCamberAndThickness(): ";
    std::size_t noseIndex;
    const std::vector<double>& profileX = getAirfoilX();
    const std::vector<double>& profileY = getAirfoilY();

    const std::size_t nPSize = profileX.size();
    std::vector<double> xThickness(nPSize);
    std::vector<double> yThickness(nPSize);
    std::vector<double> yFullCamberLine(nPSize);
    std::vector<double> xCamberLine;
    std::vector<double> yCamberLine;

    // The internal support distribution must match the current contour size,
    // which may lie outside the public 99..511 interval.
    xThickness = createXDistributionUnchecked(nPSize);
    if (!getNoseIndex(xThickness, yThickness, NoseDetectionMode::LocalMinimum, noseIndex))
    {
        emitOutput(methodName + "no nose in the support point distribution",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (0 == noseIndex)
    {
        noseIndex = indexOfMinimum(xThickness);
    }
    if (!isInteriorNose(xThickness, noseIndex))
    {
        emitOutput(methodName + "no nose in the support point distribution",
                   AirfoilDataMessageType::Error);
        return false;
    }

    xCamberLine.resize(nPSize - noseIndex);
    yCamberLine.resize(nPSize - noseIndex);

    if (!this->extractCamberLine(profileX, profileY, xThickness, yThickness,
                                 yFullCamberLine, CamberDifferenceMode::Simple))
    {
        emitOutput(methodName + "camber line extraction failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    for (std::size_t i = noseIndex; i < nPSize; i++)
    {
        xCamberLine[i - noseIndex] = xThickness[i];
        yCamberLine[i - noseIndex] = yFullCamberLine[i];
    }

    std::size_t nCorr = 0;
    if (!checkCamberline(xCamberLine, yCamberLine, nCorr))
    {
        return false;
    }

    camberX.swap(xCamberLine);
    camberY.swap(yCamberLine);
    thickDistX.swap(xThickness);
    thickDistY.swap(yThickness);

    return true;
}

bool AirfoilData::calculateThickness(const std::vector<double>& xCoordinates, const std::vector<double>& yCoordinates,
                                      ThicknessMode thicknessMode, double givenLocation, double& thicknessOut,
                                      double& thicknessAtLocation, double& thicknessLocation, SideMode sideMode)
{
    const std::string methodName = "calculateThickness(): ";

    if ((xCoordinates.size() < 2) || (xCoordinates.size() != yCoordinates.size()))
    {
        emitOutput(methodName + "x/y not consistent or too few points",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if ((&thicknessOut == &thicknessAtLocation) ||
        (&thicknessOut == &thicknessLocation) ||
        (&thicknessAtLocation == &thicknessLocation))
    {
        emitOutput(methodName + "output parameters must be different",
                   AirfoilDataMessageType::Error);
        return false;
    }

    const std::size_t nPts = AirfoilDataConstants::kThicknessSupportPoints;
    std::size_t noseIndex;
    double yUpper = 0.0, yLow = 0.0, yHigh = 0.0, thu = 0.0, thl = 0.0, tmpThick = 0.0;
    double maxThickness = 0.0;
    double maxThickLoc = 0.0;
    double thicknessAtGivenLocation = 0.0;

    if (!std::isfinite(givenLocation) || givenLocation < 0.0 || givenLocation > 1.0)
    {
        emitOutput(joinMessage("Error: location outside allowed range: 0.0 !< ", givenLocation, " !< 1.0"),
                   AirfoilDataMessageType::Error);
        thicknessAtLocation = 0.0;
        return false;
    }

    if ((ThicknessMode::Projected == thicknessMode) && (SideMode::OneSided == sideMode))
    {
        emitOutput(methodName + "projected mode requires two sides",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::vector<double> interpolatedX(nPts);
    std::vector<double> interpolatedY(nPts);
    std::vector<double> ascendingX(xCoordinates);

    // Transform the lower branch so each interpolator receives ascending
    // chordwise coordinates.
    if (SideMode::TwoSided == sideMode)
    {
        noseIndex = indexOfMinimum(ascendingX);

        if (!isInteriorNose(ascendingX, noseIndex))
        {
            emitOutput(methodName + "no inner leading edge found",
                       AirfoilDataMessageType::Error);
            return false;
        }
        // When pointCount == vector size (checked above) the nose always lies
        // within the range; the previous sub-range check is dropped.
        for (std::size_t j = 0; j < noseIndex; j++)
        {
            ascendingX[j] = -ascendingX[j];
        }
    }
    else
    {
        noseIndex = 0;
    }

    std::vector<double> upperX(ascendingX.begin() + static_cast<std::ptrdiff_t>(noseIndex), ascendingX.end());
    std::vector<double> upperY(yCoordinates.begin() + static_cast<std::ptrdiff_t>(noseIndex), yCoordinates.end());
    std::vector<double> lowerX(ascendingX.begin(), ascendingX.begin() + static_cast<std::ptrdiff_t>(noseIndex) + 1);
    std::vector<double> lowerY(yCoordinates.begin(), yCoordinates.begin() + static_cast<std::ptrdiff_t>(noseIndex) + 1);
    AkimaInterpolation upperAkima;
    AkimaInterpolation lowerAkima;

    if (false == upperAkima.init(upperX, upperY))
    {
        emitOutput(methodName + "Akima initialization of the upper side failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (SideMode::TwoSided == sideMode && false == lowerAkima.init(lowerX, lowerY))
    {
        emitOutput(methodName + "Akima initialization of the lower side failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (ThicknessMode::Projected == thicknessMode)
    {
        // Projected thickness: determine the maximum upper and lower extents
        // independently.
        thu = 0.0;
        thl = 0.0;

        for (std::size_t j = 0; j < nPts; j++)
        {
            interpolatedX[j] = static_cast<double>(j) / static_cast<double>(nPts);
            if (!upperAkima.interpolate(interpolatedX[j], interpolatedY[j]))
            {
                emitOutput(methodName + "Akima interpolation of the upper side failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
        }
        for (std::size_t j = 0; j < nPts; j++)
        {
            if (std::fabs(interpolatedY[j]) > thu)
            {
                thu = std::fabs(interpolatedY[j]);
            }
        }

        for (std::size_t j = 0; j < nPts; j++)
        {
            interpolatedX[j] = 1.0 - static_cast<double>(j) / static_cast<double>(nPts);
            if (!lowerAkima.interpolate(-interpolatedX[j], interpolatedY[j]))
            {
                emitOutput(methodName + "Akima interpolation of the lower side failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
        }
        for (std::size_t j = 0; j < nPts; j++)
        {
            if (std::fabs(interpolatedY[j]) > thl)
            {
                thl = std::fabs(interpolatedY[j]);
            }
            maxThickness = thu + thl;
        }
        maxThickLoc = 0.0; // No sense for max. thickness location in this mode
    }
    else  // ThicknessMode::Absolute
    {
        // Absolute thickness: compare upper and lower ordinates at equal x.
        maxThickness = -1.0;
        for (std::size_t j = 0; j < nPts; j++)
        {
            interpolatedX[j] = static_cast<double>(j) / static_cast<double>(nPts);
            if (!upperAkima.interpolate(interpolatedX[j], yUpper))
            {
                emitOutput(methodName + "Akima interpolation of the upper side failed",
                           AirfoilDataMessageType::Error);
                return false;
            }

            if (SideMode::TwoSided == sideMode)
            {
                if (!lowerAkima.interpolate(-interpolatedX[j], interpolatedY[j]))
                {
                    emitOutput(methodName + "Akima interpolation of the lower side failed",
                               AirfoilDataMessageType::Error);
                    return false;
                }
                tmpThick = yUpper - interpolatedY[j];
            }
            else
            {
                tmpThick = std::fabs(yUpper);
            }

            if (tmpThick > maxThickness)
            {
                maxThickness = tmpThick;
                maxThickLoc = interpolatedX[j];
            }
        }
    }

    // Evaluate the thickness at the explicitly requested location.
    if (SideMode::TwoSided == sideMode)
    {
        if (!upperAkima.interpolate(givenLocation, yHigh))
        {
            emitOutput(methodName + "Akima interpolation of the upper side failed",
                       AirfoilDataMessageType::Error);
            return false;
        }
        if (!lowerAkima.interpolate(-givenLocation, yLow))
        {
            emitOutput(methodName + "Akima interpolation of the lower side failed",
                       AirfoilDataMessageType::Error);
            return false;
        }
        thicknessAtGivenLocation = yHigh - yLow;
    }

    if (!std::isfinite(maxThickness) || !std::isfinite(maxThickLoc) ||
        ((SideMode::TwoSided == sideMode) && !std::isfinite(thicknessAtGivenLocation)))
    {
        emitOutput(methodName + "result not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }

    thicknessOut = maxThickness;
    thicknessLocation = maxThickLoc;
    if (SideMode::TwoSided == sideMode)
    {
        thicknessAtLocation = thicknessAtGivenLocation;
    }

    return true;
}

bool AirfoilData::applyThickness(ThicknessJob job, std::vector<double>& xCoordinates,
                                 std::vector<double>& yCoordinates, double desiredThickness)
{
    // Calculate all scaled ordinates before replacing the input vector.
    const std::string methodName = "applyThickness(): ";
    double currentThickness = 0.0;
    double calculatedThickLocation = 0.0;
    double where = 0.0;

    if (!std::isfinite(desiredThickness))
    {
        emitOutput(methodName + "desired thickness not finite",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (false == calculateThickness(xCoordinates, yCoordinates, ThicknessMode::Absolute, 0.5,
                                    currentThickness, calculatedThickLocation, where, SideMode::TwoSided))
    {
        emitOutput(methodName + "cannot calculate thickness", AirfoilDataMessageType::Error);
        return false;
    }

    double coefficient = 1.0;
    if (ThicknessJob::FixedThickness == job)
    {
        if (!std::isfinite(currentThickness) || 0.0 == currentThickness)
        {
            emitOutput(methodName + "current thickness zero or invalid - scaling not possible",
                       AirfoilDataMessageType::Error);
            return false;
        }
        coefficient = desiredThickness / currentThickness;

        if (!std::isfinite(coefficient))
        {
            emitOutput(methodName + "scaling factor not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
    }

    if (verbosityLevel > 0)
    {
        emitOutput(joinMessage(methodName, "current thickness : ",
                   currentThickness * 100.0, " %, scaling by ",
                   coefficient, " to new thickness of ",
                   coefficient * currentThickness * 100.0, " %"),
                   AirfoilDataMessageType::Info);
    }

    std::vector<double> scaledY(yCoordinates.size());
    for (std::size_t index = 0; index < yCoordinates.size(); ++index)
    {
        const double scaledValue = yCoordinates[index] * coefficient;
        if (!std::isfinite(scaledValue))
        {
            emitOutput(methodName + "scaled result not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
        scaledY[index] = scaledValue;
    }
    yCoordinates.swap(scaledY);
    return true;
}

bool AirfoilData::extractCamberLine(const std::vector<double>& xCoordinates,
                                    const std::vector<double>& yCoordinates,
                                    const std::vector<double>& xThickness,
                                    std::vector<double>& yThickness,
                                    std::vector<double>& yCamber,
                                    CamberDifferenceMode mode)
{
    const std::string methodName = "extractCamberLine(): ";

    if (&yThickness == &yCamber)
    {
        emitOutput(methodName + "output vectors must be different",
                   AirfoilDataMessageType::Error);
        return false;
    }

    const std::size_t pointCount = xCoordinates.size();

    if ((xCoordinates.size() < 2) || (xCoordinates.size() != yCoordinates.size())
                                  || (xThickness.size() != xCoordinates.size()))
    {
        emitOutput(methodName + "x/y/xThickness not consistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (CamberDifferenceMode::Simple != mode)
    {
        emitOutput(methodName + "mode DIFF_CORRECT not yet implemented. Aborting!",
                   AirfoilDataMessageType::Error);
        return false;
    }

    // Work on local abscissa copies; the caller's support vectors remain
    // unchanged.
    std::vector<double> calculatedThickness(pointCount);
    std::vector<double> calculatedCamber(pointCount);

    // Transform the lower branches to obtain ascending interpolation grids.
    std::vector<double> localX(xCoordinates);
    std::vector<double> localThicknessX(xThickness);

    std::size_t noseIndex1, noseIndex2;
    if (!getNoseIndex(localX, yCoordinates, NoseDetectionMode::LocalMinimum, noseIndex1) ||
        !getNoseIndex(localThicknessX, localThicknessX,
                      NoseDetectionMode::LocalMinimum, noseIndex2))
    {
        emitOutput(methodName + "cannot determine nose position",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (0 == noseIndex1)
    {
        noseIndex1 = indexOfMinimum(localX);
    }
    if (0 == noseIndex2)
    {
        noseIndex2 = indexOfMinimum(localThicknessX);
    }
    if (!isInteriorNose(localX, noseIndex1) ||
        !isInteriorNose(localThicknessX, noseIndex2))
    {
        emitOutput(methodName + "no inner leading edge found",
                   AirfoilDataMessageType::Error);
        return false;
    }

    // Preparation for Akima interpolation, only ascending xCoordinates-points
    for (std::size_t i = 0; i < noseIndex1; i++)
    {
        localX[i] = -localX[i];
    }
    for (std::size_t i = 0; i < noseIndex2; i++)
    {
        localThicknessX[i] = -localThicknessX[i];
    }

    AkimaInterpolation akima;
    if (false == akima.init(localX, yCoordinates))
    {
        emitOutput(methodName + "Akima initialization failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    for (std::size_t i = 0; i < noseIndex2; i++)
    {
        double yLower = 0.0, yUpper = 0.0;
        if (!akima.interpolate(localThicknessX[i], yLower))
        {
            emitOutput(methodName + "Akima interpolation failed",
                       AirfoilDataMessageType::Error);
            return false;
        }
        if (!akima.interpolate(-localThicknessX[i], yUpper))
        {
            emitOutput(methodName + "Akima interpolation failed",
                       AirfoilDataMessageType::Error);
            return false;
        }
        calculatedThickness[i] = -0.5 * (yUpper - yLower);
        calculatedThickness[pointCount - i - 1] = 0.5 * (yUpper - yLower);
        calculatedCamber[i] = yLower + calculatedThickness[pointCount - i - 1];
        calculatedCamber[pointCount - i - 1] = yLower + calculatedThickness[pointCount - i - 1];
    }
    calculatedThickness[noseIndex2] = 0.0;
    calculatedCamber[noseIndex2] = 0.0;

    if (!allFiniteValues(calculatedThickness) || !allFiniteValues(calculatedCamber))
    {
        emitOutput(methodName + "result not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }

    yThickness.swap(calculatedThickness);
    yCamber.swap(calculatedCamber);

    return true;
}

bool AirfoilData::calculateDyNose(const std::vector<double>& xCoordinates,
                                  const std::vector<double>& yCoordinates,
                                  double& calculatedNoseHeight)
{
    const std::string methodName = "calculateDyNose(): ";
    std::size_t noseIndex;

    if (xCoordinates.size() != yCoordinates.size())
    {
        emitOutput(methodName + "x/y not consistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (!getNoseIndex(xCoordinates, yCoordinates, NoseDetectionMode::LocalMinimum, noseIndex))
    {
        emitOutput(methodName + "cannot determine nose position",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (noseIndex >= xCoordinates.size())
    {
        emitOutput(std::string("wrong nose index (noseIndex = ") + std::to_string(noseIndex) + std::string(")"),
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (verbosityLevel > 0)
    {
        emitOutput(std::string("noseIndex = ") + std::to_string(noseIndex), AirfoilDataMessageType::Info);
    }

    // Interpolate the upper surface at the standard x/c = 0.0125 location.
    std::vector<double> upperX(xCoordinates.begin() + static_cast<std::ptrdiff_t>(noseIndex), xCoordinates.end());
    std::vector<double> upperY(yCoordinates.begin() + static_cast<std::ptrdiff_t>(noseIndex), yCoordinates.end());
    AkimaInterpolation upperAkima;
    if (!upperAkima.init(upperX, upperY))
    {
        return false;
    }

    double noseHeight = 0.0;
    if (!upperAkima.interpolate(AirfoilDataConstants::kNoseLocation, noseHeight))
    {
        emitOutput(methodName + "Akima interpolation failed",
                   AirfoilDataMessageType::Error);
        return false;
    }
    if (!std::isfinite(noseHeight))
    {
        emitOutput(methodName + "result not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }
    calculatedNoseHeight = noseHeight;

    if (verbosityLevel > 0)
    {
        emitOutput(std::string("dyNose = ") + std::to_string(calculatedNoseHeight),
                   AirfoilDataMessageType::Info);
        emitOutput(std::string("returning"), AirfoilDataMessageType::Info);
    }

    return true;
}

//*******************
// Derivative methods
//*******************

bool AirfoilData::calculateGradientCurvature()
{
    const std::string methodName = "calculateGradientCurvature(): ";
    std::size_t noseIndex;

    const std::vector<double>& xAll = getAirfoilX();
    const std::vector<double>& yAll = getAirfoilY();
    const std::size_t naSize = xAll.size();

    if ((xAll.size() != yAll.size()))
    {
        emitOutput(methodName + "x/y not consistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (!getNoseIndex(xAll, yAll, NoseDetectionMode::LocalMinimum, noseIndex) ||
        !isInteriorNose(xAll, noseIndex))
    {
        emitOutput(methodName + "no inner leading edge found",
                   AirfoilDataMessageType::Error);
        return false;
    }

    const std::size_t noseSize = noseIndex;  // noseIndex >= 0 checked
    std::vector<double> xGradLower(noseSize + 1);
    std::vector<double> xGradUpper(naSize - noseSize);
    std::vector<double> firstDerivativeLower;
    std::vector<double> secondDerivativeLower;
    std::vector<double> thirdDerivativeLower;
    std::vector<double> firstDerivativeUpper;
    std::vector<double> secondDerivativeUpper;
    std::vector<double> thirdDerivativeUpper;

    if (!this->calculateDerivatives(xAll, yAll, firstDerivativeLower, secondDerivativeLower,
                                    thirdDerivativeLower, firstDerivativeUpper, secondDerivativeUpper,
                                    thirdDerivativeUpper, SideMode::TwoSided))
    {
        emitOutput(methodName + "derivatives failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    for (std::size_t i = 0; i <= noseSize; i++)
    {
        xGradLower[i] = xAll[i];
    }
    for (std::size_t i = noseSize; i < naSize; i++)
    {
        xGradUpper[i - noseSize] = xAll[i];
    }

    gradientLowerX.swap(xGradLower);
    gradientUpperX.swap(xGradUpper);
    upperGradient.swap(firstDerivativeUpper);
    lowerGradient.swap(firstDerivativeLower);
    upperCurvature.swap(secondDerivativeUpper);
    lowerCurvature.swap(secondDerivativeLower);

    return true;
}

bool AirfoilData::calculateDerivatives(const std::vector<double>& xCoordinates,
                                       const std::vector<double>& yCoordinates,
                                       std::vector<double>& firstLower,
                                       std::vector<double>& secondLower,
                                       std::vector<double>& thirdLower,
                                       std::vector<double>& firstUpper,
                                       std::vector<double>& secondUpper,
                                       std::vector<double>& thirdUpper,
                                       SideMode sideMode)
{
    const std::string methodName = "calculateDerivatives(): ";

    if ((&firstLower == &secondLower) || (&firstLower == &thirdLower) ||
        (&firstLower == &firstUpper) || (&firstLower == &secondUpper) ||
        (&firstLower == &thirdUpper) ||
        (&secondLower == &thirdLower) || (&secondLower == &firstUpper) ||
        (&secondLower == &secondUpper) || (&secondLower == &thirdUpper) ||
        (&thirdLower == &firstUpper) || (&thirdLower == &secondUpper) ||
        (&thirdLower == &thirdUpper) ||
        (&firstUpper == &secondUpper) || (&firstUpper == &thirdUpper) ||
        (&secondUpper == &thirdUpper))
    {
        emitOutput(methodName + "output vectors must be different",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::size_t noseIndex;
    double denominator, deltaX1, deltaX2, yTemporary = 0.0;

    const std::size_t pointCount = xCoordinates.size();

    if (xCoordinates.size() != yCoordinates.size())
    {
        emitOutput(methodName + "x/y have different sizes",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (!thirdLower.empty() || !thirdUpper.empty())
    {
        emitOutput(std::string("Third derivative not yet implemented"),
                   AirfoilDataMessageType::Warning);
        return false;
    }

    if (SideMode::TwoSided == sideMode)
    {
        if (!getNoseIndex(xCoordinates, yCoordinates, NoseDetectionMode::LocalMinimum, noseIndex))
        {
            emitOutput(methodName + "cannot determine nose position",
                       AirfoilDataMessageType::Error);
            return false;
        }
        if ((noseIndex < 3) || (pointCount - noseIndex < 4))
        {
            emitOutput(methodName + "too few points per side (at least 4)",
                       AirfoilDataMessageType::Error);
            return false;
        }
    }
    else
    {
        noseIndex = 0;
        if (pointCount < 4)
        {
            emitOutput(methodName + "too few points (at least 4)",
                       AirfoilDataMessageType::Error);
            return false;
        }
    }

    std::vector<double> localX = xCoordinates;
    const std::size_t noseSize = noseIndex;

    if (SideMode::TwoSided == sideMode)
    {
        // Reverse lower side for akima interpolation
        if (!getNoseIndex(localX, yCoordinates, NoseDetectionMode::LocalMinimum, noseIndex))
        {
            emitOutput(methodName + "cannot determine nose position",
                       AirfoilDataMessageType::Error);
            return false;
        }

        for (std::size_t j = 0; j < noseSize; j++)
        {
            localX[j] = -localX[j];
        }
    }
    else  // Usually upper side, so no inverting of xCoordinates values
    {
        noseIndex = 0;
    }

    AkimaInterpolation akima;
    if (false == akima.init(localX, yCoordinates))
    {
        emitOutput(methodName + "Akima initialization failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::vector<double> firstLowerOut;
    std::vector<double> secondLowerOut;
    std::vector<double> firstUpperOut;
    std::vector<double> secondUpperOut;

    // Calculate derivatives on the interpolated local coordinates.

    // First derivative lower side
    firstLowerOut.resize(noseSize + 1);
    for (std::size_t j = 1; j < noseSize; j++)
    {
        if (SideMode::OneSided == sideMode)
        {
            deltaX1 = localX[j + 1] - localX[j];  // delta xCoordinates to nose non inverted xCoordinates values
            deltaX2 = localX[j] - localX[j - 1];  // delta xCoordinates to t.e. non inverted xCoordinates values
        }
        else  // Already in opposite order due to inversion above
        {
            deltaX1 = localX[j + 1] - localX[j];  // delta xCoordinates to nose lower side
            deltaX2 = localX[j] - localX[j - 1];  // delta xCoordinates to t.e. lower side
        }
        if (deltaX1 < deltaX2) {  // use deltaX1 as distance h
            denominator = 2.0 * deltaX1;
            if (!akima.interpolate(localX[j] + deltaX1, yTemporary))
            {
                emitOutput(methodName + "Akima interpolation failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
            firstLowerOut[j] = -(-yCoordinates[j - 1] + yTemporary) / denominator;
        }
        else  // Use deltaX2 as distance h
        {
            denominator = 2.0 * deltaX2;
            if (!akima.interpolate(localX[j] - deltaX2, yTemporary))
            {
                emitOutput(methodName + "Akima interpolation failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
            firstLowerOut[j] = -(-yTemporary + yCoordinates[j + 1]) / denominator;
        }
    }

    if (noseIndex >= 2)
    {
        firstLowerOut[0] = firstLowerOut[1] + (localX[0] - localX[1]) * (firstLowerOut[1] - firstLowerOut[2]) / (localX[1] - localX[2]);
        firstLowerOut[noseSize] = firstLowerOut[noseSize - 1] + (localX[noseSize] - localX[noseSize - 1]) * (firstLowerOut[noseSize - 1] - firstLowerOut[noseSize - 2]) / (localX[noseSize - 1] - localX[noseSize - 2]);
    }

    // First derivative upper
    firstUpperOut.resize(pointCount - noseSize);
    for (std::size_t j = noseSize + 1; j + 1 < pointCount; j++)
    {
        deltaX1 = localX[j] - localX[j - 1];  // delta xCoordinates to nose, usually always ascending
        deltaX2 = localX[j + 1] - localX[j];  // delta xCoordinates to t.e., usually always ascending
        if (deltaX1 < deltaX2) {  // Use deltaX1 as distance h
            denominator = 2.0 * deltaX1;
            if (!akima.interpolate(localX[j] + deltaX1, yTemporary))
            {
                emitOutput(methodName + "Akima interpolation failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
            firstUpperOut[j - noseSize] = (-yCoordinates[j - 1] + yTemporary) / denominator;
        }
        else  // Use deltaX2 as distance h
        {
            denominator = 2.0 * deltaX2;
            if (!akima.interpolate(localX[j] - deltaX2, yTemporary))
            {
                emitOutput(methodName + "Akima interpolation failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
            firstUpperOut[j - noseSize] = (-yTemporary + yCoordinates[j + 1]) / denominator;
        }
    }

    firstUpperOut[0] = firstUpperOut[1] + (localX[noseSize] - localX[noseSize + 1]) * (firstUpperOut[1] - firstUpperOut[2]) / (localX[noseSize + 1] - localX[noseSize + 2]);
    firstUpperOut[pointCount - noseSize - 1] = firstUpperOut[pointCount - noseSize - 2] + (localX[pointCount - 1] - localX[pointCount - 2]) * (firstUpperOut[pointCount - noseSize - 2] - firstUpperOut[pointCount - noseSize - 3]) / (localX[pointCount - 2] - localX[pointCount - 3]);

    // Second derivative lower
    secondLowerOut.resize(noseSize + 1);
    for (std::size_t j = 1; j <= noseSize; j++)
    {
        deltaX1 = localX[j + 1] - localX[j];  // delta xCoordinates to nose
        deltaX2 = localX[j] - localX[j - 1];  // delta xCoordinates to t.e.
        if (deltaX1 < deltaX2) {  // Use deltaX1 as distance h
            denominator = deltaX1 * deltaX1;
            if (!akima.interpolate(localX[j] - deltaX1, yTemporary))
            {
                emitOutput(methodName + "Akima interpolation failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
            secondLowerOut[j - 1] = (yTemporary - 2.0 * yCoordinates[j] + yCoordinates[j + 1]) / denominator;
        }
        else  // Use deltaX2 as distance h
        {
            denominator = deltaX2 * deltaX2;
            if (!akima.interpolate(localX[j] + deltaX2, yTemporary))
            {
                emitOutput(methodName + "Akima interpolation failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
            secondLowerOut[j - 1] = (yCoordinates[j - 1] - 2.0 * yCoordinates[j] + yTemporary) / denominator;
        }
    }

    if (noseIndex >= 2)
    {
        // Extrapolating curvature values
        secondLowerOut[0] = secondLowerOut[1] + (localX[0] - localX[1]) * (secondLowerOut[1] - secondLowerOut[2]) / (localX[1] - localX[2]);
        secondLowerOut[noseSize] = secondLowerOut[noseSize - 1] - (localX[noseSize + 1] - localX[noseSize - 2]) * (secondLowerOut[noseSize - 1] - secondLowerOut[noseSize - 2]) / (localX[noseSize - 1] - localX[noseSize - 2]);
    }

    // Second derivative upper
    secondUpperOut.resize(pointCount - noseSize);
    for (std::size_t j = noseSize + 1; j + 1 < pointCount; j++)
    {
        deltaX1 = localX[j + 1] - localX[j];  // delta xCoordinates to nose
        deltaX2 = localX[j] - localX[j - 1];  // delta xCoordinates to t.e.
        if (deltaX1 < deltaX2) {  // Use deltaX1 as distance h
            denominator = deltaX1 * deltaX1;
            if (!akima.interpolate(localX[j] - deltaX1, yTemporary))
            {
                emitOutput(methodName + "Akima interpolation failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
            secondUpperOut[j - noseSize] = (yTemporary - 2.0 * yCoordinates[j] + yCoordinates[j + 1]) / denominator;
        }
        else  // Use deltaX2 as distance h
        {
            denominator = deltaX2 * deltaX2;
            if (!akima.interpolate(localX[j] + deltaX2, yTemporary))
            {
                emitOutput(methodName + "Akima interpolation failed",
                           AirfoilDataMessageType::Error);
                return false;
            }
            secondUpperOut[j - noseSize] = (yCoordinates[j - 1] - 2.0 * yCoordinates[j] + yTemporary) / denominator;
        }
    }

    secondUpperOut[0] = secondUpperOut[1] + (localX[noseSize] - localX[noseSize + 1]) * (secondUpperOut[1] - secondUpperOut[2]) /
                        (localX[noseSize + 1] - localX[noseSize + 2]);
    secondUpperOut[pointCount - 1 - noseSize] = secondUpperOut[pointCount - 2 - noseSize] + (localX[pointCount - 1] - localX[pointCount - 2]) *
                                            (secondUpperOut[pointCount - 2 - noseSize] - secondUpperOut[pointCount - 3 - noseSize]) /
                                            (localX[pointCount - 2] - localX[pointCount - 3]);

    if (!allFiniteValues(firstLowerOut) || !allFiniteValues(secondLowerOut) ||
        !allFiniteValues(firstUpperOut) || !allFiniteValues(secondUpperOut))
    {
        emitOutput(methodName + "derivative result not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }

    // Third derivative not implemented

    firstLower.swap(firstLowerOut);
    secondLower.swap(secondLowerOut);
    firstUpper.swap(firstUpperOut);
    secondUpper.swap(secondUpperOut);

    return true;
}

//********************
// Aerodynamic methods
//********************

bool AirfoilData::setAerodynamicData(const std::vector<double>& alpha_,
                                     const std::vector<double>& cD_, const std::vector<double>& cDf_,
                                     const std::vector<double>& cDp_, const std::vector<double>& cL_,
                                     const std::vector<double>& cM_, const AeroScalars& scalars,
                                     const std::vector<double>& eCurrent_,
                                     const std::vector<bool>& converged_)
{
    const std::string methodName = "setAerodynamicData(): ";
    const std::size_t nAlphaSize = alpha_.size();
    if ((cD_.size() != nAlphaSize) || (cDf_.size() != nAlphaSize) ||
        (cDp_.size() != nAlphaSize) || (cL_.size() != nAlphaSize) ||
        (cM_.size() != nAlphaSize) || (eCurrent_.size() != nAlphaSize) ||
        (converged_.size() != nAlphaSize))
    {
        emitOutput(methodName + "vector lengths not consistent with the number of angles of attack",
                   AirfoilDataMessageType::Error);
        return false;
    }

    for (std::size_t index = 0; index < nAlphaSize; ++index)
    {
        if (converged_[index] &&
            (!std::isfinite(alpha_[index]) || !std::isfinite(cD_[index]) ||
             !std::isfinite(cDf_[index]) || !std::isfinite(cDp_[index]) ||
             !std::isfinite(cL_[index]) || !std::isfinite(cM_[index]) ||
             !std::isfinite(eCurrent_[index])))
        {
            emitOutput(joinMessage(methodName, "non-finite value at converged point index=",
                                   static_cast<int>(index)),
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
    }
    if (!std::isfinite(scalars.alpha0) || !std::isfinite(scalars.cl0) ||
        !std::isfinite(scalars.cd0) || !std::isfinite(scalars.cm0) ||
        !std::isfinite(scalars.clMax) || !std::isfinite(scalars.eMax))
    {
        emitOutput(methodName + "non-finite scalar value",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }

    std::vector<double> newAlpha(alpha_);
    std::vector<double> newCD(cD_);
    std::vector<double> newCDf(cDf_);
    std::vector<double> newCDp(cDp_);
    std::vector<double> newCL(cL_);
    std::vector<double> newCM(cM_);
    std::vector<double> newE(eCurrent_);
    std::vector<bool> newConverged(converged_);

    alpha.swap(newAlpha);
    cD.swap(newCD);
    cDf.swap(newCDf);
    cDp.swap(newCDp);
    cL.swap(newCL);
    cM.swap(newCM);
    eVector.swap(newE);
    converged.swap(newConverged);

    alpha0 = scalars.alpha0;
    cl0 = scalars.cl0;
    cd0 = scalars.cd0;
    cm0 = scalars.cm0;
    clMax = scalars.clMax;
    eMax = scalars.eMax;
    return true;
}

bool AirfoilData::updateCirculationAndSourceStrength()
{
    AirfoilData snapshot = *this;

    if (false == calculateAirfoilCamberAndThickness())
    {
        *this = std::move(snapshot);
        return false;
    }
    if (false == updateCirculation(RecalcMode::Keep))
    {
        *this = std::move(snapshot);
        return false;
    }
    if (false == updateSourceStrength(RecalcMode::Keep))
    {
        *this = std::move(snapshot);
        return false;
    }
    return true;
}

bool AirfoilData::updateSourceStrength(RecalcMode reCalc)
{
    const std::string methodName = "updateSourceStrength(): ";

    AirfoilData snapshot;
    if (RecalcMode::Recompute == reCalc)
    {
        snapshot = *this;
        if (false == calculateAirfoilCamberAndThickness())
        {
            return false;
        }
    }

    const std::vector<double>& xt = getThicknessDistributionX();
    const std::vector<double>& yt = getThicknessDistributionY();

    if (xt.size() != yt.size())
    {
        if (RecalcMode::Recompute == reCalc)
        {
            *this = std::move(snapshot);
        }
        emitOutput(methodName + "number of thickness points not consistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::size_t in;

    if (!getNoseIndex(xt, yt, NoseDetectionMode::LocalMinimum, in) ||
        !isInteriorNose(xt, in))
    {
        if (RecalcMode::Recompute == reCalc)
        {
            *this = std::move(snapshot);
        }
        emitOutput(methodName + "nose index in the thickness distribution invalid",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::vector<double> xThickness(xt.begin() + static_cast<std::ptrdiff_t>(in), xt.end());
    std::vector<double> yThickness(yt.begin() + static_cast<std::ptrdiff_t>(in), yt.end());
    std::vector<double> xq;
    std::vector<double> q;

    if (!this->calculateSourceStrength(xThickness, yThickness, xq, q))
    {
        if (RecalcMode::Recompute == reCalc)
        {
            *this = std::move(snapshot);
        }
        emitOutput(methodName + "source strength calculation failed",
                   AirfoilDataMessageType::Error);
        return false;
    }

    sourceX.swap(xq);
    sourceY.swap(q);
    return true;
}

bool AirfoilData::calculateSourceStrength(const std::vector<double>& xThickness,
                                         const std::vector<double>& yThickness,
                                         std::vector<double>& xq,
                                         std::vector<double>& q)
{
    const std::string methodName = "calculateSourceStrength(): ";

    if (&xq == &q)
    {
        emitOutput(methodName + "output vectors must be different",
                   AirfoilDataMessageType::Error);
        return false;
    }
    const std::size_t nSourceSize = xThickness.size();
    std::vector<double> firstUpper;
    std::vector<double> unusedFirstLower;
    std::vector<double> unusedSecondLower;
    std::vector<double> unusedSecondUpper;
    std::vector<double> unusedThirdLower;
    std::vector<double> unusedThirdUpper;
    const bool derivativesResult = this->calculateDerivatives(xThickness, yThickness, unusedFirstLower,
                                                              unusedSecondLower, unusedThirdLower,
                                                              firstUpper, unusedSecondUpper, unusedThirdUpper,
                                                              SideMode::OneSided);
    if (!derivativesResult)
    {
        return false;
    }

    if (firstUpper.size() < nSourceSize)
    {
        emitOutput(methodName + "derivatives not available for all support points",
                   AirfoilDataMessageType::Error);
        return false;
    }

    // Internal buffers at original size; the result size nq is later applied to xq/q.
    std::vector<double> xSourceBuffer(nSourceSize);
    std::vector<double> qBuffer(nSourceSize);
    for (std::size_t i = 0; i < nSourceSize; i++)
    {
        xSourceBuffer[i] = xThickness[i];
        qBuffer[i] = firstUpper[i];
    }

    std::vector<double> calculatedXq(nSourceSize);
    std::vector<double> calculatedQ(nSourceSize);
    for (std::size_t index = 0; index < nSourceSize; index++)
    {
        if (!std::isfinite(xSourceBuffer[index]) || !std::isfinite(qBuffer[index]))
        {
            emitOutput(methodName + "result not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
        calculatedXq[index] = xSourceBuffer[index];
        calculatedQ[index] = qBuffer[index];
    }
    xq.swap(calculatedXq);
    q.swap(calculatedQ);

    return true;
}

bool AirfoilData::updateCirculation(RecalcMode reCalc)
{
    const std::string methodName = "updateCirculation(): ";
    AirfoilData snapshot;

    if (RecalcMode::Recompute == reCalc)
    {
        snapshot = *this;
        if (false == calculateAirfoilCamberAndThickness())
        {
            return false;
        }
    }

    const std::vector<double>& camberLineX = getCamberLineX();
    const std::vector<double>& camberLineY = getCamberLineY();
    const std::size_t nP = camberLineX.size();

    if ((nP < 1) || (nP != camberLineY.size()))
    {
        if (RecalcMode::Recompute == reCalc)
        {
            *this = std::move(snapshot);
        }
        emitOutput(methodName + "number of camber points not consistent",
                   AirfoilDataMessageType::Error);
        return false;
    }

    std::vector<double> xCirculation;
    std::vector<double> circulationStrength;

    if (!this->calculateCirculationCl05(camberLineX, camberLineY, xCirculation, circulationStrength))
    {
        if (RecalcMode::Recompute == reCalc)
        {
            *this = std::move(snapshot);
        }
        emitOutput(methodName + "circulation calculation failed",
                   AirfoilDataMessageType::Error);
        return false;
    }
    xCirculation.resize(nP - 1);
    circulationStrength.resize(nP - 1);
    circX.swap(xCirculation);
    circY.swap(circulationStrength);
    return true;
}

bool AirfoilData::linearInterpolate(const std::vector<double>& xNodes,
                                   const std::vector<double>& zNodes,
                                   double xQuery, double& result)
{
    const std::size_t nNodes = xNodes.size();

    if ((nNodes < 2) || (zNodes.size() < xNodes.size()))
    {
        result = 0.0;
        return false;
    }
    if (xQuery <= xNodes[0])
    {
        result = zNodes[0];
        return true;
    }
    if (xQuery >= xNodes[nNodes-1])
    {
        result = zNodes[nNodes-1];
        return true;
    }
    std::size_t upperIndex = 1;
    while (upperIndex + 1 < nNodes && xNodes[upperIndex] < xQuery)
    {
        upperIndex++;
    }
    const std::size_t lowerIndex = upperIndex - 1;
    double width = xNodes[upperIndex] - xNodes[lowerIndex];
    if (width <= 0.0)
    {
        result = zNodes[lowerIndex];
        return true;
    }
    double factor = (xQuery - xNodes[lowerIndex]) / width;
    result = (1.0 - factor) * zNodes[lowerIndex] + factor * zNodes[upperIndex];
    return true;
}

bool AirfoilData::calculateCirculation(const std::vector<double>& camberLineX,
                                       const std::vector<double>& camberLineY,
                                       std::vector<double>& xCirculation,
                                       std::vector<double>& circulationStrength,
                                       double angleOfAttack)
{
    const std::string methodName = "calculateCirculation(): ";

    if (&xCirculation == &circulationStrength)
    {
        emitOutput(methodName + "output vectors must be different",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (camberLineX.size() != camberLineY.size())
    {
        emitOutput(methodName + "camberLineX/camberLineY have different sizes",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if (!std::isfinite(angleOfAttack))
    {
        emitOutput(methodName + "angle of attack not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }
    const std::size_t pointCount = camberLineX.size();

    if (pointCount < 4)
    {
        emitOutput(methodName + "at least 4 points required",
                   AirfoilDataMessageType::Error);
        return false;
    }

    for (std::size_t iRange = 0; iRange < pointCount; iRange++)
    {
        // x must lie in [0,1]: negative values would push the argument of
        // std::acos(2.0 * x - 1.0) below -1 and produce NaN.
        if (!std::isfinite(camberLineX[iRange]) || (camberLineX[iRange] < 0.0)
                                                || (camberLineX[iRange] > 1.0))
        {
            emitOutput(joinMessage(methodName, "x value is not finite or outside [0,1]: ",
                                   camberLineX[iRange]),
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
    }

    const std::size_t nA = AirfoilDataConstants::kCirculationFourierCount;
    double A0;
    std::vector<double> firstDerivativeUpper;
    std::vector<double> xCircResult(pointCount);
    std::vector<double> circResult(pointCount);
    std::vector<double> dzdx(pointCount + 1);
    std::vector<double> phi(pointCount + 1);
    std::vector<double> An(nA + 1);
    std::vector<double> unusedFirstLower;
    std::vector<double> unusedSecondLower;
    std::vector<double> unusedSecondUpper;
    std::vector<double> unusedThirdLower;
    std::vector<double> unusedThirdUpper;

    // Calculate the camber-line slope.
    const bool derivativesResult = this->calculateDerivatives(camberLineX, camberLineY, unusedFirstLower,
                                                              unusedSecondLower, unusedThirdLower,
                                                              firstDerivativeUpper, unusedSecondUpper,
                                                              unusedThirdUpper, SideMode::OneSided);
    if (!derivativesResult)
    {
        return false;
    }

    for (std::size_t i = 1; i + 1 < pointCount; i++)
    {
        xCircResult[i] = camberLineX[i];
        dzdx[i] = firstDerivativeUpper[i];
    }
    xCircResult[0] = camberLineX[0];
    xCircResult[pointCount-1] = camberLineX[pointCount-1];
    dzdx[0] = firstDerivativeUpper[1];
    dzdx[pointCount-1] = dzdx[pointCount-2];

    // Map the support coordinates to the angular variable phi.
    for (std::size_t i = 0; i < pointCount; i++)
    {
        if (camberLineX[i] > AirfoilDataConstants::kZeroCrossingEpsilon)
        {
            phi[i] = std::acos(2.0 * camberLineX[i] - 1.0);
        }
        else
        {
            phi[i] = std::acos(2.0e-5 - 1.0);
        }
    }

    // Now calculating coefficients A0 and An for circulation k(phi)

    // Interpolate the slope onto the fine angular integration grid.
    const std::size_t nPhi = AirfoilDataConstants::kCirculationPhaseGrid;
    std::vector<double> phiFine(nPhi);
    std::vector<double> dzdxFine(nPhi);
    for (std::size_t i = 0; i < nPhi; i++)
    {
        phiFine[i] = AirfoilDataConstants::kPi * static_cast<double>(i) / static_cast<double>(nPhi - 1);
        double xPhi = 0.5 * (1.0 + std::cos(phiFine[i]));
        double interpolatedValue = 0.0;
        if (!linearInterpolate(camberLineX, dzdx, xPhi, interpolatedValue))
        {
            return false;
        }
        dzdxFine[i] = interpolatedValue;
    }

    // Determine the geometry-only zero-lift angle and moment coefficient.
    double integral0 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
    for (std::size_t i = 0; i + 1 < nPhi; i++)
    {
        double h = phiFine[i+1] - phiFine[i];
        integral0 += 0.5 * h * (dzdxFine[i] + dzdxFine[i+1]);
        double f1a = dzdxFine[i]   * std::cos(phiFine[i]);
        double f1b = dzdxFine[i+1] * std::cos(phiFine[i+1]);
        a1 += 0.5 * h * (f1a + f1b);
        double f2a = dzdxFine[i]   * std::cos(2.0 * phiFine[i]);
        double f2b = dzdxFine[i+1] * std::cos(2.0 * phiFine[i+1]);
        a2 += 0.5 * h * (f2a + f2b);
    }
    // A0 (camber-line contribution at alpha=0) = -(1/pi) * integral0
    double a0Camber = -(1.0 / AirfoilDataConstants::kPi) * integral0;
    // A1/A2 multiplied by the factor -2/pi:
    a1 *= (-2.0 / AirfoilDataConstants::kPi);
    a2 *= (-2.0 / AirfoilDataConstants::kPi);

    // alpha0 = -(A0 + A1/2) in rad; alpha0Geo in degrees; cm0 = -(pi/4)(A1 + A2)
    double alpha0Rad = -(a0Camber + 0.5 * a1);
    const double calculatedAlpha0Geo = alpha0Rad * 180.0 / AirfoilDataConstants::kPi;
    const double calculatedCm0Geo = -(AirfoilDataConstants::kPi / 4.0) * (a1 + a2);

    // A0 = alpha - (1/pi) * integral_0^pi (dz/dx) dphi   (trapezoidal rule, at the passed
    // angle of attack angleOfAttack in degrees)
    const double alphaRad = angleOfAttack * AirfoilDataConstants::kPi / 180.0;
    A0 = alphaRad + a0Camber;

    // An = -(2/pi) * integral_0^pi (dz/dx) std::cos(n phi) dphi   (trapezoidal rule)
    for (std::size_t j = 1; j <= nA; j++)
    {
        double integral = 0.0;
        for (std::size_t i = 0; i + 1 < nPhi; i++)
        {
            double h = phiFine[i+1] - phiFine[i];
            double f0 = dzdxFine[i]   * std::cos(static_cast<double>(j) * phiFine[i]);
            double f1 = dzdxFine[i+1] * std::cos(static_cast<double>(j) * phiFine[i+1]);
            integral += 0.5 * h * (f0 + f1);
        }
        An[j-1] = (-2.0 / AirfoilDataConstants::kPi) * integral;
    }

    // Evaluate the normalized vortex-sheet distribution k/UInf.
    for (std::size_t i = 0; i < pointCount; i++)
    {
        double sumAn;

        sumAn = 0.0;
        for (std::size_t j = 1; j <= nA; j++)
        {
            sumAn += (An[j-1] * std::sin(static_cast<double>(j) * phi[i]));
        }
        circResult[i] = 2.0 * (A0 * std::tan(0.5 * phi[i]) + sumAn);
    }

    if (!std::isfinite(calculatedAlpha0Geo) || !std::isfinite(calculatedCm0Geo))
    {
        emitOutput(methodName + "alpha0Geo/cm0Geo not finite",
                   AirfoilDataMessageType::ErrorSevere);
        return false;
    }
    for (std::size_t iCheck = 0; iCheck < pointCount; iCheck++)
    {
        if (!std::isfinite(circResult[iCheck]))
        {
            emitOutput(methodName + "circulation value not finite",
                       AirfoilDataMessageType::ErrorSevere);
            return false;
        }
    }

    xCirculation.swap(xCircResult);
    circulationStrength.swap(circResult);
    alpha0Geo = calculatedAlpha0Geo;
    cm0Geo = calculatedCm0Geo;

    return true;
}

// Circulation distribution at a fixed lift coefficient cl = 0.5. Calls the method with
// an angle of attack twice, so the computation core exists in only one place:
//   1st pass: compute alpha0 with an arbitrary angle of attack (alpha=0); alpha0 is a
//      pure camber-line property and thus independent of the angle of attack (sets
//      alpha0Geo/cm0Geo).
//   2nd pass: angle of attack for cl=0.5 from cl = 2*pi*(alpha - alpha0) -> alpha = cl/(2*pi) + alpha0
//      and compute the circulation distribution at this angle of attack (overwrites
//      xCirculation/circulationStrength).
bool AirfoilData::calculateCirculationCl05(const std::vector<double>& xCoordinates,
                                           const std::vector<double>& yCoordinates,
                                           std::vector<double>& xCirculation,
                                           std::vector<double>& circulationStrength)
{
    const std::string methodName = "calculateCirculationCl05(): ";

    if (&xCirculation == &circulationStrength)
    {
        emitOutput(methodName + "output vectors must be different",
                   AirfoilDataMessageType::Error);
        return false;
    }

    if ((xCoordinates.size() != yCoordinates.size()) ||
        (xCoordinates.size() < 4))
    {
        emitOutput(methodName + "x/y not consistent or too few support points",
                   AirfoilDataMessageType::Error);
        return false;
    }

    const double previousAlpha0Geo = alpha0Geo;
    const double previousCm0Geo = cm0Geo;

    // 1st pass: determine alpha0 with an arbitrary angle of attack (alpha=0).
    std::vector<double> firstPassX;
    std::vector<double> firstPassStrength;
    if (!calculateCirculation(xCoordinates, yCoordinates, firstPassX, firstPassStrength, 0.0))
    {
        return false;
    }

    // angle of attack for cl=0.5: alpha = cl/(2*pi) + alpha0 (alpha0 from alpha0Geo in degrees).
    const double alpha0Rad = alpha0Geo * AirfoilDataConstants::kPi / 180.0;
    const double angleOfAttack =
        ((0.5 / (2.0 * AirfoilDataConstants::kPi)) + alpha0Rad)
        * 180.0 / AirfoilDataConstants::kPi;

    if (!std::isfinite(angleOfAttack))
    {
        emitOutput(methodName + "computed angle of attack not finite",
                   AirfoilDataMessageType::ErrorSevere);
        alpha0Geo = previousAlpha0Geo;
        cm0Geo = previousCm0Geo;
        return false;
    }

    // 2nd pass: compute the circulation distribution at this angle of attack.
    std::vector<double> finalX;
    std::vector<double> finalStrength;
    if (!calculateCirculation(xCoordinates, yCoordinates, finalX, finalStrength, angleOfAttack))
    {
        alpha0Geo = previousAlpha0Geo;
        cm0Geo = previousCm0Geo;
        return false;
    }

    xCirculation.swap(finalX);
    circulationStrength.swap(finalStrength);
    return true;
}

//**************************
// Getter and setter methods
//**************************

double AirfoilData::getTrailingEdgeThickness() const noexcept
{
    return teThickness;
}

void AirfoilData::setTeThickness(double t)
{
    teThickness = t;
}

const std::vector<double>& AirfoilData::getE() const noexcept
{
    return eVector;
}

const std::vector<double>& AirfoilData::getCd() const noexcept
{
    return cD;
}

const std::vector<double>& AirfoilData::getCdf() const noexcept
{
    return cDf;
}

const std::vector<double>& AirfoilData::getCdp() const noexcept
{
    return cDp;
}

const std::vector<double>& AirfoilData::getCl() const noexcept
{
    return cL;
}

const std::vector<double>& AirfoilData::getCm() const noexcept
{
    return cM;
}

double AirfoilData::getAlpha0() const noexcept
{
    return alpha0;
}

double AirfoilData::getCm0() const noexcept
{
    return cm0;
}

double AirfoilData::getCl0() const noexcept
{
    return cl0;
}

double AirfoilData::getCd0() const noexcept
{
    return cd0;
}

double AirfoilData::getClMax() const noexcept
{
    return clMax;
}

double AirfoilData::getEMax() const noexcept
{
    return eMax;
}

const std::vector<bool>& AirfoilData::getConverged() const noexcept
{
    return converged;
}

void AirfoilData::setIsCreated(bool val)
{
    created = val;
}

bool AirfoilData::isCreated() const noexcept
{
    return created;
}

const std::vector<double>& AirfoilData::getGradientDataUpper() const noexcept
{
    return upperGradient;
}

const std::vector<double>& AirfoilData::getGradientDataLower() const noexcept
{
    return lowerGradient;
}

const std::vector<double>& AirfoilData::getCurvatureDataUpper() const noexcept
{
    return upperCurvature;
}

const std::vector<double>& AirfoilData::getCurvatureDataLower() const noexcept
{
    return lowerCurvature;
}

GradientData AirfoilData::getGradientData() const
{
    GradientData gradientData;

    if ((gradientUpperX.size() != upperGradient.size()) ||
        (gradientUpperX.size() != upperCurvature.size()) ||
        (gradientLowerX.size() != lowerGradient.size()) ||
        (gradientLowerX.size() != lowerCurvature.size()))
    {
        return gradientData;  // valid == false, all fields empty
    }

    gradientData.valid = true;
    gradientData.upperX = gradientUpperX;
    gradientData.lowerX = gradientLowerX;
    gradientData.upperGradient = upperGradient;
    gradientData.lowerGradient = lowerGradient;
    gradientData.upperCurvature = upperCurvature;
    gradientData.lowerCurvature = lowerCurvature;

    return gradientData;
}

void AirfoilData::setPointsConformalFunctionX(const std::vector<double>& v)
{
    conformalFunctionX = v;
}

void AirfoilData::setPointsConformalFunctionY(const std::vector<double>& v)
{
    conformalFunctionY = v;
}

int AirfoilData::getNumberOfConformalFunctionPoints() const noexcept
{
    return static_cast<int>(conformalFunctionX.size());
}

void AirfoilData::setPointsConformalCircleX(const std::vector<double>& v)
{
    conformalCircleX = v;
}

void AirfoilData::setPointsConformalCircleY(const std::vector<double>& v)
{
    conformalCircleY = v;
}

int AirfoilData::getNumberOfConformalCirclePoints() const noexcept
{
    return static_cast<int>(conformalCircleX.size());
}

void AirfoilData::setPointsCamberDerivativeX(const std::vector<double>& v)
{
    camberDerivativeX = v;
}

void AirfoilData::setPointsCamberDerivativeY(const std::vector<double>& v)
{
    camberDerivativeY = v;
}

int AirfoilData::getNumberOfCamberDerivativePoints() const noexcept
{
    return static_cast<int>(camberDerivativeX.size());
}

void AirfoilData::setPointsThicknessDerivativeX(const std::vector<double>& v)
{
    thicknessDerivativeX = v;
}

void AirfoilData::setPointsThicknessDerivativeY(const std::vector<double>& v)
{
    thicknessDerivativeY = v;
}

int AirfoilData::getNumberOfThicknessDerivativePoints() const noexcept
{
    return static_cast<int>(thicknessDerivativeX.size());
}

const std::string& AirfoilData::getAirfoilName() const noexcept
{
    return name;
}

void AirfoilData::setAirfoilName(const std::string& newName)
{
    name = newName;
}

int AirfoilData::getNumberOfThicknessDistributionPoints() const noexcept
{
    return static_cast<int>(thickDistX.size());
}

std::size_t AirfoilData::getNumberOfDesiredAirfoilPoints() const noexcept
{
    return desiredNumberOfAirfoilPoints;
}

int AirfoilData::getNumberOfUpperGradientPoints() const noexcept
{
    return static_cast<int>(gradientUpperX.size());
}

int AirfoilData::getNumberOfLowerGradientPoints() const noexcept
{
    return static_cast<int>(gradientLowerX.size());
}

int AirfoilData::getNumberOfCamberPoints() const noexcept
{
    return static_cast<int>(camberX.size());
}

int AirfoilData::getNumberOfCirculationPoints() const noexcept
{
    return static_cast<int>(circX.size());
}

int AirfoilData::getNumberOfSourcePoints() const noexcept
{
    return static_cast<int>(sourceX.size());
}

double AirfoilData::getAlpha0Geo() const noexcept
{
    return alpha0Geo;
}

double AirfoilData::getCm0Geo() const noexcept
{
    return cm0Geo;
}

double AirfoilData::getArea() const noexcept
{
    return area;
}

double AirfoilData::getDyNose() const noexcept
{
    return dyNose;
}

double AirfoilData::getThickness() const noexcept
{
    return thickness;
}

double AirfoilData::getThicknessLocation() const noexcept
{
    return maxThicknessLocation;
}

double AirfoilData::getCamber() const noexcept
{
    return camber;
}

double AirfoilData::getCamberLocation() const noexcept
{
    return maxCamberPosition;
}

const std::vector<double>& AirfoilData::getGradientDataUpperX() const noexcept
{
    return gradientUpperX;
}

const std::vector<double>& AirfoilData::getGradientDataLowerX() const noexcept
{
    return gradientLowerX;
}

const std::vector<double>& AirfoilData::getAirfoilX() const noexcept
{
    return airfoilX;
}

const std::vector<double>& AirfoilData::getAirfoilY() const noexcept
{
    return airfoilY;
}

const std::vector<double>& AirfoilData::getUpperSurfaceX() const noexcept
{
    return upperHalfX;
}

const std::vector<double>& AirfoilData::getUpperSurfaceY() const noexcept
{
    return upperHalfY;
}

const std::vector<double>& AirfoilData::getLowerSurfaceX() const noexcept
{
    return lowerHalfX;
}

const std::vector<double>& AirfoilData::getLowerSurfaceY() const noexcept
{
    return lowerHalfY;
}

const std::vector<double>& AirfoilData::getCamberLineX() const noexcept
{
    return camberX;
}

const std::vector<double>& AirfoilData::getCamberLineY() const noexcept
{
    return camberY;
}

const std::vector<double>& AirfoilData::getCirculationX() const noexcept
{
    return circX;
}

const std::vector<double>& AirfoilData::getCirculationY() const noexcept
{
    return circY;
}

const std::vector<double>& AirfoilData::getSourceX() const noexcept
{
    return sourceX;
}

const std::vector<double>& AirfoilData::getSourceY() const noexcept
{
    return sourceY;
}

const std::vector<double>& AirfoilData::getThicknessDistributionX() const noexcept
{
    return thickDistX;
}

const std::vector<double>& AirfoilData::getThicknessDistributionY() const noexcept
{
    return thickDistY;
}

const std::vector<double>& AirfoilData::getCamberDerivativeX() const noexcept
{
    return camberDerivativeX;
}

const std::vector<double>& AirfoilData::getCamberDerivativeY() const noexcept
{
    return camberDerivativeY;
}

const std::vector<double>& AirfoilData::getThicknessDerivativeX() const noexcept
{
    return thicknessDerivativeX;
}

const std::vector<double>& AirfoilData::getThicknessDerivativeY() const noexcept
{
    return thicknessDerivativeY;
}

const std::vector<double>& AirfoilData::getConformalFunctionX() const noexcept
{
    return conformalFunctionX;
}

const std::vector<double>& AirfoilData::getConformalFunctionY() const noexcept
{
    return conformalFunctionY;
}

const std::vector<double>& AirfoilData::getConformalCircleX() const noexcept
{
    return conformalCircleX;
}

const std::vector<double>& AirfoilData::getConformalCircleY() const noexcept
{
    return conformalCircleY;
}

const std::vector<double>& AirfoilData::getAlphas() const noexcept
{
    return alpha;
}

void AirfoilData::setThickness(double thickness_)
{
    thickness = thickness_;
}

void AirfoilData::setThickLocation(double maxThicknessLocation_)
{
    maxThicknessLocation = maxThicknessLocation_;
}

void AirfoilData::setCamber(double camber_)
{
    camber = camber_;
}

void AirfoilData::setCamberLocation(double maxCamberLocation_)
{
    maxCamberPosition = maxCamberLocation_;
}

const std::vector<std::vector<double>>& AirfoilData::getUpperCp() const noexcept
{
    return cpUpper;
}

const std::vector<std::vector<double>>& AirfoilData::getLowerCp() const noexcept
{
    return cpLower;
}

const std::vector<double>& AirfoilData::getCpUpperX() const noexcept
{
    return cpUpperX;
}

const std::vector<double>& AirfoilData::getCpLowerX() const noexcept
{
    return cpLowerX;
}

void AirfoilData::setCpUpper(const std::vector<std::vector<double>>& m)
{
    cpUpper = m;
}

void AirfoilData::setCpLower(const std::vector<std::vector<double>>& m)
{
    cpLower = m;
}

void AirfoilData::setCpUpperX(const std::vector<double>& v)
{
    cpUpperX = v;
}

void AirfoilData::setCpLowerX(const std::vector<double>& v)
{
    cpLowerX = v;
}

void AirfoilData::setThicknessX(const std::vector<double>& v)
{
    thickDistX = v;
}

void AirfoilData::setThicknessY(const std::vector<double>& v)
{
    thickDistY = v;
}

void AirfoilData::setCamberLineX(const std::vector<double>& v)
{
    camberX = v;
}

void AirfoilData::setCamberLineY(const std::vector<double>& v)
{
    camberY = v;
}

void AirfoilData::setPointsX(const std::vector<double>& v)
{
    airfoilX = v;
}

void AirfoilData::setPointsY(const std::vector<double>& v)
{
    airfoilY = v;
}

bool AirfoilData::setAirfoilPoints(const std::vector<double>& xCoordinates,
                                   const std::vector<double>& yCoordinates)
{
    const std::string methodName = "setAirfoilPoints(): ";
    if (xCoordinates.size() != yCoordinates.size())
    {
        emitOutput(joinMessage(methodName, "x/y sizes inconsistent (",
                               xCoordinates.size(), " != ", yCoordinates.size(), ")"),
                   AirfoilDataMessageType::Error);
        return false;
    }
    if (false == setAirfoilCoordinates(xCoordinates, yCoordinates))
    {
        return false;
    }
    // A new contour invalidates all derived, imported, and application-state
    // data associated with the previous contour.
    clearCalculatedGeometryData();
    return true;
}

void AirfoilData::setPointsUpperX(const std::vector<double>& v)
{
    upperHalfX = v;
}

void AirfoilData::setPointsUpperY(const std::vector<double>& v)
{
    upperHalfY = v;
}

void AirfoilData::setPointsLowerX(const std::vector<double>& v)
{
    lowerHalfX = v;
}

void AirfoilData::setPointsLowerY(const std::vector<double>& v)
{
    lowerHalfY = v;
}

void AirfoilData::setVerbosityLevel(std::size_t level)
{
    // Diagnostic level 0..3; larger values are clamped to 3.
    verbosityLevel = (level > 3) ? 3 : level;
}

bool AirfoilData::pointCountInAllowedRange(std::size_t np)
{
    return (np >= 99) && (np <= 511);
}

std::size_t AirfoilData::indexOfMinimum(const std::vector<double>& values)
{
    std::size_t minIndex = 0;
    for (std::size_t i = 1; i < values.size(); i++)
    {
        if (values[i] < values[minIndex])
        {
            minIndex = i;
        }
    }
    return minIndex;
}

bool AirfoilData::validatePointCount(std::size_t& np)
{
    // Public contour grids use 99..511 points. Reduce even values to preserve
    // one distinct leading-edge support point.
    if (false == pointCountInAllowedRange(np))
    {
        return false;
    }
    if (0 == (np % 2))  // Even number
    {
        np--;
    }
    return true;
}

bool AirfoilData::setNumberOfDesiredAirfoilPoints(std::size_t np)
{
    if (false == validatePointCount(np))
    {
        return false;
    }
    desiredNumberOfAirfoilPoints = np;
    return true;
}
