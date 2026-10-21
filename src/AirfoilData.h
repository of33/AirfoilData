//**************************************************************************
//   AirfoilData
//   Storing, manipulating and analyzing airfoils
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

#ifndef AIRFOILDATA_H
#define AIRFOILDATA_H

#include <cstddef>
#include <string>
#include <vector>

/// Numerical and geometric constants used by AirfoilData.
namespace AirfoilDataConstants
{
    inline constexpr double kPi =
        3.1415926535897932384626433832795;

    // Numerical evaluation grids.
    inline constexpr std::size_t kThicknessSupportPoints = 4096;
    inline constexpr std::size_t kAreaSupportPoints = 32768; // Must be even.
    inline constexpr std::size_t kCirculationFourierCount = 101;
    inline constexpr std::size_t kCirculationPhaseGrid = 2000;

    // Numerical tolerances.
    inline constexpr double kDuplicateEpsilon = 1.0e-8;
    inline constexpr double kZeroCrossingEpsilon = 1.0e-5;
    inline constexpr double kGradientEpsilon = 1.0e-7;

    // Geometric reference values.
    inline constexpr double kNoseLocation = 0.0125;
    inline constexpr double kTrailingEdgeLimit = 0.95;

    // Minimum point counts used by normalizeAirfoil().
    inline constexpr std::size_t kMinNormalizeInputPoints = 11;
    inline constexpr std::size_t kMinTargetSupportPoints = 3;
}

/// Selects the leading-edge detection method.
enum class NoseDetectionMode
{
    /// Greatest distance from the midpoint of the contour endpoints.
    LongestSecant,

    /// Strict local minimum of the chordwise coordinate.
    LocalMinimum
};

/// Selects the thickness definition.
enum class ThicknessMode
{
    /// Vertical distance at equal chordwise positions.
    Absolute,

    /// Sum of the maximum projected upper and lower extents.
    Projected
};

/// Selects the complete-contour support-point distribution.
enum class XDistributionMode
{
    /// Clustering near the leading and trailing edges.
    Cosine,

    /// Clustering mainly near the leading edge.
    HalfCosine
};

/// Selects the operation performed by applyThickness().
enum class ThicknessJob
{
    /// Retains the current thickness.
    ScaleOnly,

    /// Scales the ordinates to the requested thickness.
    FixedThickness
};

/// Selects one-sided or two-sided processing.
enum class SideMode
{
    TwoSided,
    OneSided
};

/// Selects the camber and thickness extraction method.
enum class CamberDifferenceMode
{
    /// Uses the vertical midpoint of corresponding surface points.
    Simple,

    /// Reserved for a perpendicular-distance implementation.
    Correct
};

/// Selects whether contour orientation may be corrected.
enum class SortMode
{
    None,
    Sort
};

/// Selects whether normalization maps onto target support points.
enum class InterpolationMode
{
    /// Retains the input point count.
    None,

    /// Uses the original contents of xOut as target coordinates.
    Map
};

/// Selects the scope of redistributeAirfoil().
enum class RedistributeMode
{
    /// Replaces the contour and recalculates all derived data.
    AllData,

    /// Replaces only the contour and clears derived data.
    AirfoilOnly
};

/// Selects whether camber and thickness data are recomputed.
enum class RecalcMode
{
    Recompute,
    Keep
};

/// Selects the chordwise range checked by airfoilOK().
enum class TrailingEdgeCheck
{
    Full,
    SkipNearEdge
};

/// Scalar aerodynamic values imported with coefficient vectors.
struct AeroScalars
{
    double alpha0 = 0.0;
    double cl0 = 0.0;
    double cd0 = 0.0;
    double cm0 = 0.0;
    double clMax = 0.0;
    double eMax = 0.0;
};

/// Classifies messages emitted by AirfoilData.
enum class AirfoilDataMessageType
{
    Normal,
    Info,
    Warning,
    Error,
    WarningSevere,
    ErrorSevere
};

/// Bundles stored surface coordinates and derivative data.
struct GradientData
{
    bool valid = false;
    std::vector<double> upperX;
    std::vector<double> lowerX;
    std::vector<double> upperGradient;
    std::vector<double> lowerGradient;
    std::vector<double> upperCurvature;
    std::vector<double> lowerCurvature;
};

/// Stores, transforms, and analyzes two-dimensional airfoil data.
class AirfoilData
{
public:
    AirfoilData() = default;

    // =========================================================
    // Construction and basic object state
    // =========================================================

    /// Initializes an empty airfoil template.
    ///
    /// The requested size must lie in 99..511. Even values are reduced to
    /// the next lower odd value. An invalid size clears the airfoil data and
    /// leaves the desired target point count unchanged.
    void initAirfoil(std::size_t size);

    /// Clears the contour and all associated or derived data.
    void clearAirfoilData();

    /// Assigns a complete airfoil contour.
    ///
    /// On success, data derived from the previous contour are cleared and
    /// the created state is reset. The name and desired point count remain.
    ///
    /// @return false if the vector lengths differ; the object then remains
    ///         unchanged.
    [[nodiscard]] bool setAirfoilPoints(
        const std::vector<double>& xCoordinates,
        const std::vector<double>& yCoordinates);

    void setAirfoilName(const std::string& newName);
    const std::string& getAirfoilName() const noexcept;

    bool isCreated() const noexcept;
    void setIsCreated(bool value);

    // =========================================================
    // Output and diagnostics
    // =========================================================

    using PrintMethod = void (*)(void* context,
                                 const std::string& text,
                                 AirfoilDataMessageType type);

    /// Installs a non-owning output callback and context pointer.
    void setOutputMethod(PrintMethod method, void* context)
    {
        print = method;
        printContext = context;
    }

    PrintMethod outputMethod() const
    {
        return print;
    }

    void* outputContext() const
    {
        return printContext;
    }

    void setOutputSuppressed(bool suppress)
    {
        outputSuppressed_ = suppress;
    }

    bool outputSuppressed() const
    {
        return outputSuppressed_;
    }

    /// Emits a message through the callback or console fallback.
    void emitOutput(const std::string& text,
                    AirfoilDataMessageType type);

    /// Sets the diagnostic level to 0..3; larger values are clamped.
    void setVerbosityLevel(std::size_t level);

    // =========================================================
    // Basic API
    // =========================================================

    /// Checks the stored contour for upper/lower surface crossings.
    [[nodiscard]] bool airfoilOK(
        TrailingEdgeCheck trailingEdgeCheck);

    /// Calculates the complete standard airfoil-property data set.
    ///
    /// The operation is transactional. If any processing step fails, the
    /// previous object state is restored.
    [[nodiscard]] bool calculateAirfoilProperties();

    /// Redistributes the stored contour.
    ///
    /// @param nPointsNew Point count in 99..511. Even values are reduced
    ///        to the next lower odd value.
    /// @param mode Selects whether only the contour or all data are updated.
    [[nodiscard]] bool redistributeAirfoil(
        std::size_t nPointsNew,
        RedistributeMode mode);

    /// Writes the contour to a .dat file.
    ///
    /// The object must be marked as created. If source is not empty,
    /// "Created by <source>" is appended to the file header.
    [[nodiscard]] bool saveAirfoilCoordinates(
        const std::string& fileName,
        const std::string& source);

    const std::vector<double>& getAirfoilX() const noexcept;
    const std::vector<double>& getAirfoilY() const noexcept;
    const std::vector<double>& getUpperSurfaceX() const noexcept;
    const std::vector<double>& getUpperSurfaceY() const noexcept;
    const std::vector<double>& getLowerSurfaceX() const noexcept;
    const std::vector<double>& getLowerSurfaceY() const noexcept;
    const std::vector<double>& getCamberLineX() const noexcept;
    const std::vector<double>& getCamberLineY() const noexcept;
    const std::vector<double>& getThicknessDistributionX() const noexcept;
    const std::vector<double>& getThicknessDistributionY() const noexcept;
    const std::vector<double>& getCirculationX() const noexcept;
    const std::vector<double>& getCirculationY() const noexcept;
    const std::vector<double>& getSourceX() const noexcept;
    const std::vector<double>& getSourceY() const noexcept;

    double getThickness() const noexcept;
    double getThicknessLocation() const noexcept;
    double getCamber() const noexcept;
    double getCamberLocation() const noexcept;
    double getTrailingEdgeThickness() const noexcept;
    double getArea() const noexcept;
    double getDyNose() const noexcept;
    double getAlpha0Geo() const noexcept;
    double getCm0Geo() const noexcept;

    /// Returns copies of the stored surface derivative data.
    GradientData getGradientData() const;

    // =========================================================
    // Advanced API: support points and geometry transformation
    // =========================================================

    /// Generates a complete-contour chordwise distribution.
    ///
    /// @return An empty vector if pointCount is outside 99..511.
    [[nodiscard]] static std::vector<double> createXDistribution(
        std::size_t pointCount,
        XDistributionMode mode);

    [[nodiscard]] static std::vector<double> createXDistribution(
        std::size_t pointCount);

    std::size_t getNumberOfDesiredAirfoilPoints() const noexcept;

    /// Stores a desired target count in 99..511.
    /// Even values are reduced to the next lower odd value.
    [[nodiscard]] bool setNumberOfDesiredAirfoilPoints(
        std::size_t pointCount);

    int getNumberOfCamberPoints() const noexcept;
    int getNumberOfUpperGradientPoints() const noexcept;
    int getNumberOfLowerGradientPoints() const noexcept;
    int getNumberOfCirculationPoints() const noexcept;
    int getNumberOfSourcePoints() const noexcept;
    int getNumberOfThicknessDistributionPoints() const noexcept;

    /// Determines a leading-edge candidate.
    ///
    /// LongestSecant measures distances from the midpoint of the two contour
    /// endpoints. LocalMinimum searches for a strict local x minimum.
    /// Callers requiring a valid interior leading edge must perform the
    /// corresponding geometric validation.
    [[nodiscard]] bool getNoseIndex(
        const std::vector<double>& profileX,
        const std::vector<double>& profileY,
        NoseDetectionMode mode,
        std::size_t& noseIndex) const;

    /// Reverses both coordinate vectors while preserving point pairs.
    void reorderData(std::vector<double>& xCoordinates,
                     std::vector<double>& yCoordinates);

    /// Scales a coordinate set to a target chordwise interval.
    [[nodiscard]] bool scale(
        const std::vector<double>& xIn,
        const std::vector<double>& yIn,
        std::vector<double>& xOut,
        std::vector<double>& yOut,
        double xMin,
        double xMax);

    /// Rotates a coordinate set around the origin.
    ///
    /// Positive angles rotate clockwise. The angle is specified in degrees.
    [[nodiscard]] bool rotate(
        const std::vector<double>& xIn,
        const std::vector<double>& yIn,
        std::vector<double>& xOut,
        std::vector<double>& yOut,
        double angleDeg);

    /// Normalizes an airfoil contour to unit chord and zero incidence.
    ///
    /// In Map mode, the original values in xOut define the target support
    /// distribution. rotatedAngle is returned in degrees.
    [[nodiscard]] bool normalizeAirfoil(
        const std::vector<double>& xIn,
        const std::vector<double>& yIn,
        std::vector<double>& xOut,
        std::vector<double>& yOut,
        SortMode sort,
        InterpolationMode interpolate,
        double& rotatedAngle);

    /// Maps a contour onto a newly generated support-point distribution.
    [[nodiscard]] bool mapAirfoilToSupportPoints(
        const std::vector<double>& inputX,
        const std::vector<double>& inputY,
        std::vector<double>& redistributedX,
        std::vector<double>& redistributedY,
        std::size_t newPointCount);

    /// Removes small branch-wise chordwise reversals.
    ///
    /// On a repair-limit failure, the coordinate vectors may already have
    /// been modified and removedCount is reset to zero.
    [[nodiscard]] bool repairXHooks(
        std::vector<double>& xCoordinates,
        std::vector<double>& yCoordinates,
        std::size_t& removedCount);

    // =========================================================
    // Advanced API: geometry calculations
    // =========================================================

    [[nodiscard]] bool calculateAirfoilCamberAndThickness();
    [[nodiscard]] bool calculateAirfoilGeometryData();
    [[nodiscard]] bool calculateUpperLowerCoordinates();

    [[nodiscard]] bool calculateThickness(
        const std::vector<double>& xCoordinates,
        const std::vector<double>& yCoordinates,
        ThicknessMode thicknessMode,
        double givenLocation,
        double& thickness,
        double& thicknessAtLocation,
        double& thicknessLocation,
        SideMode sideMode);

    [[nodiscard]] bool applyThickness(
        ThicknessJob job,
        std::vector<double>& xCoordinates,
        std::vector<double>& yCoordinates,
        double desiredThickness);

    [[nodiscard]] bool calculateDyNose(
        const std::vector<double>& xCoordinates,
        const std::vector<double>& yCoordinates,
        double& calculatedNoseHeight);

    [[nodiscard]] bool calculateArea(
        const std::vector<double>& xCoordinates,
        const std::vector<double>& yCoordinates,
        double& calculatedArea);

    [[nodiscard]] bool extractCamberLine(
        const std::vector<double>& xCoordinates,
        const std::vector<double>& yCoordinates,
        const std::vector<double>& xThickness,
        std::vector<double>& yThickness,
        std::vector<double>& yCamber,
        CamberDifferenceMode mode);

    void calculateTeThickness();

    // =========================================================
    // Advanced API: derivatives and singularity distributions
    // =========================================================

    [[nodiscard]] bool calculateDerivatives(
        const std::vector<double>& xCoordinates,
        const std::vector<double>& yCoordinates,
        std::vector<double>& firstLower,
        std::vector<double>& secondLower,
        std::vector<double>& thirdLower,
        std::vector<double>& firstUpper,
        std::vector<double>& secondUpper,
        std::vector<double>& thirdUpper,
        SideMode sideMode);

    [[nodiscard]] bool calculateGradientCurvature();

    const std::vector<double>& getGradientDataUpperX() const noexcept;
    const std::vector<double>& getGradientDataLowerX() const noexcept;
    const std::vector<double>& getGradientDataUpper() const noexcept;
    const std::vector<double>& getGradientDataLower() const noexcept;
    const std::vector<double>& getCurvatureDataUpper() const noexcept;
    const std::vector<double>& getCurvatureDataLower() const noexcept;

    /// Calculates normalized vortex-sheet values k/UInf.
    ///
    /// angleOfAttack and the updated alpha0Geo are expressed in degrees.
    [[nodiscard]] bool calculateCirculation(
        const std::vector<double>& camberLineX,
        const std::vector<double>& camberLineY,
        std::vector<double>& xCirculation,
        std::vector<double>& circulationStrength,
        double angleOfAttack);

    /// Calculates normalized vortex-sheet values k/UInf at cl = 0.5.
    [[nodiscard]] bool calculateCirculationCl05(
        const std::vector<double>& xCoordinates,
        const std::vector<double>& yCoordinates,
        std::vector<double>& xCirculation,
        std::vector<double>& circulationStrength);

    [[nodiscard]] bool updateCirculation(RecalcMode reCalc);

    /// Calculates q/(2*UInf) = dz^(t)/dx.
    ///
    /// The returned values are geometry descriptors, not dimensional source
    /// strengths.
    [[nodiscard]] bool calculateSourceStrength(
        const std::vector<double>& xThickness,
        const std::vector<double>& yThickness,
        std::vector<double>& xq,
        std::vector<double>& q);

    [[nodiscard]] bool updateSourceStrength(RecalcMode reCalc);
    [[nodiscard]] bool updateCirculationAndSourceStrength();

    // =========================================================
    // Integration API: imported aerodynamic and pressure data
    // =========================================================

    [[nodiscard]] bool setAerodynamicData(
        const std::vector<double>& alpha,
        const std::vector<double>& cD,
        const std::vector<double>& cDf,
        const std::vector<double>& cDp,
        const std::vector<double>& cL,
        const std::vector<double>& cM,
        const AeroScalars& scalars,
        const std::vector<double>& efficiency,
        const std::vector<bool>& converged);

    const std::vector<double>& getAlphas() const noexcept;
    const std::vector<double>& getCd() const noexcept;
    const std::vector<double>& getCdf() const noexcept;
    const std::vector<double>& getCdp() const noexcept;
    const std::vector<double>& getCl() const noexcept;
    const std::vector<double>& getCm() const noexcept;
    const std::vector<double>& getE() const noexcept;
    const std::vector<bool>& getConverged() const noexcept;

    double getAlpha0() const noexcept;
    double getCl0() const noexcept;
    double getCd0() const noexcept;
    double getCm0() const noexcept;
    double getClMax() const noexcept;
    double getEMax() const noexcept;

    void setCpUpper(const std::vector<std::vector<double>>& values);
    void setCpLower(const std::vector<std::vector<double>>& values);
    void setCpUpperX(const std::vector<double>& values);
    void setCpLowerX(const std::vector<double>& values);

    const std::vector<std::vector<double>>& getUpperCp() const noexcept;
    const std::vector<std::vector<double>>& getLowerCp() const noexcept;
    const std::vector<double>& getCpUpperX() const noexcept;
    const std::vector<double>& getCpLowerX() const noexcept;

    // =========================================================
    // Integration API: low-level field access
    // =========================================================

    // These independent setters do not validate related dimensions and do
    // not clear derived data. Prefer setAirfoilPoints() for ordinary input.
    void setPointsX(const std::vector<double>& values);
    void setPointsY(const std::vector<double>& values);

    void setCamberLineX(const std::vector<double>& values);
    void setCamberLineY(const std::vector<double>& values);
    void setThicknessX(const std::vector<double>& values);
    void setThicknessY(const std::vector<double>& values);
    void setPointsUpperX(const std::vector<double>& values);
    void setPointsUpperY(const std::vector<double>& values);
    void setPointsLowerX(const std::vector<double>& values);
    void setPointsLowerY(const std::vector<double>& values);

    void setThickness(double value);
    void setTeThickness(double value);
    void setThickLocation(double value);
    void setCamber(double value);
    void setCamberLocation(double value);

    void setPointsCamberDerivativeX(
        const std::vector<double>& values);
    void setPointsCamberDerivativeY(
        const std::vector<double>& values);
    void setPointsThicknessDerivativeX(
        const std::vector<double>& values);
    void setPointsThicknessDerivativeY(
        const std::vector<double>& values);

    const std::vector<double>& getCamberDerivativeX() const noexcept;
    const std::vector<double>& getCamberDerivativeY() const noexcept;
    const std::vector<double>& getThicknessDerivativeX() const noexcept;
    const std::vector<double>& getThicknessDerivativeY() const noexcept;

    int getNumberOfCamberDerivativePoints() const noexcept;
    int getNumberOfThicknessDerivativePoints() const noexcept;

    void setPointsConformalFunctionX(
        const std::vector<double>& values);
    void setPointsConformalFunctionY(
        const std::vector<double>& values);
    void setPointsConformalCircleX(
        const std::vector<double>& values);
    void setPointsConformalCircleY(
        const std::vector<double>& values);

    const std::vector<double>& getConformalFunctionX() const noexcept;
    const std::vector<double>& getConformalFunctionY() const noexcept;
    const std::vector<double>& getConformalCircleX() const noexcept;
    const std::vector<double>& getConformalCircleY() const noexcept;

    int getNumberOfConformalFunctionPoints() const noexcept;
    int getNumberOfConformalCirclePoints() const noexcept;

    // =========================================================
    // Version information
    // =========================================================

    static constexpr int versionMajor() noexcept
    {
        return kAirfoilDataVersionMajor;
    }

    static constexpr int versionMinor() noexcept
    {
        return kAirfoilDataVersionMinor;
    }

    static constexpr int versionPatch() noexcept
    {
        return kAirfoilDataVersionPatch;
    }

    static constexpr const char* versionString() noexcept
    {
        return kAirfoilDataVersionString;
    }

private:
    // =========================================================
    // Internal helpers
    // =========================================================

    inline static constexpr int kAirfoilDataVersionMajor = 1;
    inline static constexpr int kAirfoilDataVersionMinor = 0;
    inline static constexpr int kAirfoilDataVersionPatch = 0;
    inline static constexpr const char* kAirfoilDataVersionString = "1.0.0";

    PrintMethod print = nullptr;
    void* printContext = nullptr;
    bool outputSuppressed_ = false;

    static bool isInteriorNose(const std::vector<double>& xCoordinates,
                               std::size_t noseIndex);

    static bool isSecantNose(const std::vector<double>& xCoordinates,
                             std::size_t noseIndex);

    static bool pointCountInAllowedRange(std::size_t pointCount);

    static std::size_t indexOfMinimum(const std::vector<double>& values);
    static bool validatePointCount(std::size_t& pointCount);

    static std::vector<double> createXDistributionUnchecked(
        std::size_t pointCount,
        XDistributionMode mode);

    static std::vector<double> createXDistributionUnchecked(
        std::size_t pointCount);

    bool rotateRadians(const std::vector<double>& xIn,
                       const std::vector<double>& yIn,
                       std::vector<double>& xOut,
                       std::vector<double>& yOut,
                       double angleRad);

    void clearCalculatedGeometryData();

    bool sumSupportPointArea(const std::vector<double>& areaPointsX,
                             const std::vector<double>& areaPointsY,
                             double& calculatedArea);

    bool checkCamberline(std::vector<double>& xCamberLine,
                         std::vector<double>& yCamberLine,
                         std::size_t& remainingCount);

    void sortDataClockwise(std::vector<double>& xCoordinates,
                           std::vector<double>& yCoordinates);

    void removeConsecutiveDuplicates(
        std::vector<double>& xCoordinates,
        std::vector<double>& yCoordinates);

    bool setAirfoilCoordinates(std::vector<double> xCoordinates,
                               std::vector<double> yCoordinates);

    static bool linearInterpolate(const std::vector<double>& xNodes,
                                  const std::vector<double>& zNodes,
                                  double xQuery,
                                  double& result);

    // =========================================================
    // Stored data
    // =========================================================

    std::size_t desiredNumberOfAirfoilPoints = 255;

    double thickness = 0.0;
    double maxThicknessLocation = 0.0;
    double teThickness = 0.0;
    double camber = 0.0;
    double maxCamberPosition = 0.0;
    double dyNose = 0.0;
    double area = 0.0;

    std::size_t verbosityLevel = 0;

    std::vector<double> airfoilX;
    std::vector<double> airfoilY;
    std::vector<double> upperHalfX;
    std::vector<double> lowerHalfX;
    std::vector<double> upperHalfY;
    std::vector<double> lowerHalfY;
    std::vector<double> camberX;
    std::vector<double> camberY;
    std::vector<double> camberDerivativeX;
    std::vector<double> camberDerivativeY;
    std::vector<double> thicknessDerivativeX;
    std::vector<double> thicknessDerivativeY;
    std::vector<double> conformalFunctionX;
    std::vector<double> conformalFunctionY;
    std::vector<double> conformalCircleX;
    std::vector<double> conformalCircleY;
    std::vector<double> sourceX;
    std::vector<double> sourceY;
    std::vector<double> circX;
    std::vector<double> circY;
    std::vector<double> thickDistX;
    std::vector<double> thickDistY;
    std::vector<double> gradientUpperX;
    std::vector<double> gradientLowerX;
    std::vector<double> upperGradient;
    std::vector<double> lowerGradient;
    std::vector<double> upperCurvature;
    std::vector<double> lowerCurvature;

    std::vector<double> cpUpperX;
    std::vector<double> cpLowerX;
    std::vector<std::vector<double>> cpUpper;
    std::vector<std::vector<double>> cpLower;

    std::vector<double> alpha;
    std::vector<double> cL;
    std::vector<double> cD;
    std::vector<double> cDf;
    std::vector<double> cDp;
    std::vector<double> cM;
    std::vector<double> eVector;

    double alpha0Geo = 0.0;
    double cm0Geo = 0.0;
    double alpha0 = 0.0;
    double cm0 = 0.0;
    double cl0 = 0.0;
    double cd0 = 0.0;
    double clMax = 0.0;
    double eMax = 0.0;

    std::vector<bool> converged;
    bool created = false;
    std::string name;
};

#endif // AIRFOILDATA_H
