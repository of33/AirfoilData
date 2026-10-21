// Scenario corpus for the golden generator and the test program.
// Convention: each scenario step emits one or more lines
//   tag|value   (value deterministically serialized, doubles as %a)
//   tag|console|N|line   (N lines of captured core output)
//
// The corpus exercises every public method of AirfoilData and
// AkimaInterpolation at least once (main profile: NACA4618.dat). Private helpers
// (sortDataClockwise, removeConsecutiveDuplicates, sumSupportPointArea,
// checkCamberline, linearInterpolate, isInteriorNose/isSecantNose,
// clearCalculatedGeometryData, setAirfoilCoordinates) are covered indirectly via
// the public methods that call them.

#include "harness_common.h"

#include "AirfoilData.h"
#include "AkimaInterpolation.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <streambuf>
#include <string>
#include <vector>

// ------------------------------------------------------------------
// Serialization
// ------------------------------------------------------------------

std::string harnessDouble(double value)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%a", value);
    return std::string(buffer);
}

std::string harnessBool(bool value)
{
    return value ? std::string("1") : std::string("0");
}

void emitDouble(HarnessSink& sink, const std::string& tag, double value)
{
    sink.emit(tag + "|" + harnessDouble(value));
}

void emitInt(HarnessSink& sink, const std::string& tag, int value)
{
    sink.emit(tag + "|" + std::to_string(value));
}

void emitBool(HarnessSink& sink, const std::string& tag, bool value)
{
    sink.emit(tag + "|" + harnessBool(value));
}

void emitString(HarnessSink& sink, const std::string& tag, const std::string& value)
{
    sink.emit(tag + "|" + value);
}

static void emitVectorElement(HarnessSink& sink, const std::string& tag,
                              std::size_t index, double value)
{
    sink.emit(tag + "[" + std::to_string(index) + "]|" + harnessDouble(value));
}

void emitVector(HarnessSink& sink, const std::string& tag, const std::vector<double>& v)
{
    sink.emit(tag + "|size=" + std::to_string(v.size()));
    if (v.size() <= 12)
    {
        for (std::size_t i = 0; i < v.size(); i++)
            emitVectorElement(sink, tag, i, v[i]);
    }
    else
    {
        for (std::size_t i = 0; i < 5; i++)
            emitVectorElement(sink, tag, i, v[i]);
        for (std::size_t i = v.size() - 5; i < v.size(); i++)
            emitVectorElement(sink, tag, i, v[i]);
    }
}

void emitVectorSize(HarnessSink& sink, const std::string& tag, const std::vector<double>& v)
{
    sink.emit(tag + "|size=" + std::to_string(v.size()));
}

void emitVector2d(HarnessSink& sink, const std::string& tag, const std::vector<std::vector<double>>& m)
{
    sink.emit(tag + "|rows=" + std::to_string(m.size()));
    for (std::size_t r = 0; r < m.size(); r++)
    {
        sink.emit(tag + ".row" + std::to_string(r) + "|size=" + std::to_string(m[r].size()));
        for (std::size_t i = 0; i < m[r].size(); i++)
            emitVectorElement(sink, tag + ".row" + std::to_string(r), i, m[r][i]);
    }
}

void emitVectorBool(HarnessSink& sink, const std::string& tag, const std::vector<bool>& v)
{
    std::string bits;
    for (std::size_t i = 0; i < v.size(); i++)
        bits += v[i] ? "1" : "0";
    sink.emit(tag + "|size=" + std::to_string(v.size()) + "|" + bits);
}

// ------------------------------------------------------------------
// ConsoleCapture
// ------------------------------------------------------------------

ConsoleCapture::ConsoleCapture()
    : oldCout_(std::cout.rdbuf(buffer_.rdbuf()))
    , oldCerr_(std::cerr.rdbuf(buffer_.rdbuf()))
{
}

ConsoleCapture::~ConsoleCapture()
{
    std::cout.rdbuf(oldCout_);
    std::cerr.rdbuf(oldCerr_);
}

std::string ConsoleCapture::text() const
{
    return buffer_.str();
}

void emitConsole(HarnessSink& sink, const std::string& tag, const std::string& captured)
{
    std::vector<std::string> lines;
    std::string current;
    for (std::size_t i = 0; i < captured.size(); i++)
    {
        if (captured[i] == '\n')
        {
            lines.push_back(current);
            current.clear();
        }
        else
        {
            current += captured[i];
        }
    }
    if (!current.empty())
        lines.push_back(current);
    sink.emit(tag + "|console|" + std::to_string(lines.size()));
    for (std::size_t i = 0; i < lines.size(); i++)
        sink.emit(tag + "|console|" + std::to_string(i) + "|" + lines[i]);
}

// ------------------------------------------------------------------
// Profile file
// ------------------------------------------------------------------

bool loadProfile(const std::string& path, std::vector<double>& x, std::vector<double>& y)
{
    std::ifstream file(path);
    if (!file.is_open())
        return false;
    x.clear();
    y.clear();
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream stream(line);
        double xi = 0.0;
        double yi = 0.0;
        if (!(stream >> xi >> yi))
            continue;
        x.push_back(xi);
        y.push_back(yi);
    }
    return !x.empty();
}

// ------------------------------------------------------------------
// Helper profiles
// ------------------------------------------------------------------

static std::vector<double> cosineX(std::size_t n)
{
    std::vector<double> xs(n);
    const double dPhi = (2.0 * AirfoilDataConstants::kPi) / (static_cast<double>(n) - 1.0);
    for (std::size_t i = 0; i < n; i++)
        xs[i] = 0.5 * (1.0 + std::cos(static_cast<double>(i) * dPhi));
    return xs;
}

// Circle around [0.5:0.0] with r=0.5, closed trailing edge, n points (n even).
static void circleProfile(std::size_t n, std::vector<double>& x, std::vector<double>& y)
{
    x = cosineX(n);
    y.assign(n, 0.0);
    const std::size_t noseIndex = n / 2;  // x[noseIndex] = 0
    for (std::size_t i = 0; i < noseIndex; i++)
    {
        const double xi = x[i];
        const double yy = -std::sqrt(0.25 - (xi - 0.5) * (xi - 0.5));
        y[i] = yy;
        y[n - i - 1] = -yy;
    }
    y[noseIndex] = 0.0;
}

// Loads a profile into an AirfoilData via the public setters.
static void loadInto(AirfoilData& ad, const std::vector<double>& x, const std::vector<double>& y)
{
    ad.setPointsX(x);
    ad.setPointsY(y);
}

// ------------------------------------------------------------------
// Scenarios
// ------------------------------------------------------------------

static void scenarioVersion(HarnessSink& sink)
{
    emitInt(sink, "version.major", AirfoilData::versionMajor());
    emitInt(sink, "version.minor", AirfoilData::versionMinor());
    emitInt(sink, "version.patch", AirfoilData::versionPatch());
    emitString(sink, "version.string", AirfoilData::versionString());
}

static void scenarioDefaultState(HarnessSink& sink)
{
    AirfoilData ad;
    emitString(sink, "default.name", ad.getAirfoilName());
    emitBool(sink, "default.isCreated", ad.isCreated());
    emitBool(sink, "default.outputSuppressed", ad.outputSuppressed());
    emitInt(sink, "default.desiredPoints", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
    emitVectorSize(sink, "default.airfoilX", ad.getAirfoilX());
    emitVectorSize(sink, "default.airfoilY", ad.getAirfoilY());
    emitVectorSize(sink, "default.upperX", ad.getUpperSurfaceX());
    emitVectorSize(sink, "default.lowerX", ad.getLowerSurfaceX());
    emitVectorSize(sink, "default.camberX", ad.getCamberLineX());
    emitVectorSize(sink, "default.circX", ad.getCirculationX());
    emitVectorSize(sink, "default.sourceX", ad.getSourceX());
    emitVectorSize(sink, "default.thickDistX", ad.getThicknessDistributionX());
    emitVectorSize(sink, "default.camberDerivX", ad.getCamberDerivativeX());
    emitVectorSize(sink, "default.thickDerivX", ad.getThicknessDerivativeX());
    emitVectorSize(sink, "default.confFuncX", ad.getConformalFunctionX());
    emitVectorSize(sink, "default.confCircleX", ad.getConformalCircleX());
    emitVectorSize(sink, "default.gradUpperX", ad.getGradientDataUpperX());
    emitVectorSize(sink, "default.gradLowerX", ad.getGradientDataLowerX());
    emitVectorSize(sink, "default.curvUpper", ad.getCurvatureDataUpper());
    emitVectorSize(sink, "default.curvLower", ad.getCurvatureDataLower());
    emitVectorSize(sink, "default.alphas", ad.getAlphas());
    emitVectorSize(sink, "default.cl", ad.getCl());
    emitVectorSize(sink, "default.cd", ad.getCd());
    emitVectorSize(sink, "default.cdf", ad.getCdf());
    emitVectorSize(sink, "default.cdp", ad.getCdp());
    emitVectorSize(sink, "default.cm", ad.getCm());
    emitVectorSize(sink, "default.e", ad.getE());
    emitVectorBool(sink, "default.converged", ad.getConverged());
    emitVector2d(sink, "default.upperCp", ad.getUpperCp());
    emitVector2d(sink, "default.lowerCp", ad.getLowerCp());
    emitVectorSize(sink, "default.cpUpperX", ad.getCpUpperX());
    emitVectorSize(sink, "default.cpLowerX", ad.getCpLowerX());
    emitDouble(sink, "default.thickness", ad.getThickness());
    emitDouble(sink, "default.thickLoc", ad.getThicknessLocation());
    emitDouble(sink, "default.teThickness", ad.getTrailingEdgeThickness());
    emitDouble(sink, "default.camber", ad.getCamber());
    emitDouble(sink, "default.camberLoc", ad.getCamberLocation());
    emitDouble(sink, "default.area", ad.getArea());
    emitDouble(sink, "default.dyNose", ad.getDyNose());
    emitDouble(sink, "default.alpha0Geo", ad.getAlpha0Geo());
    emitDouble(sink, "default.cm0Geo", ad.getCm0Geo());
    emitDouble(sink, "default.alpha0", ad.getAlpha0());
    emitDouble(sink, "default.cm0", ad.getCm0());
    emitInt(sink, "default.nCamber", ad.getNumberOfCamberPoints());
    emitInt(sink, "default.nUpperGrad", ad.getNumberOfUpperGradientPoints());
    emitInt(sink, "default.nLowerGrad", ad.getNumberOfLowerGradientPoints());
    emitInt(sink, "default.nCirc", ad.getNumberOfCirculationPoints());
    emitInt(sink, "default.nSource", ad.getNumberOfSourcePoints());
    emitInt(sink, "default.nThickDist", ad.getNumberOfThicknessDistributionPoints());
    emitInt(sink, "default.nCamberDeriv", ad.getNumberOfCamberDerivativePoints());
    emitInt(sink, "default.nThickDeriv", ad.getNumberOfThicknessDerivativePoints());
    emitInt(sink, "default.nConfFunc", ad.getNumberOfConformalFunctionPoints());
    emitInt(sink, "default.nConfCircle", ad.getNumberOfConformalCirclePoints());
}

static void scenarioScalarSetters(HarnessSink& sink)
{
    AirfoilData ad;
    ad.setThickness(0.12);
    ad.setTeThickness(0.005);
    ad.setThickLocation(0.3);
    ad.setCamber(0.04);
    ad.setCamberLocation(0.45);
    emitDouble(sink, "scalars.thickness", ad.getThickness());
    emitDouble(sink, "scalars.teThickness", ad.getTrailingEdgeThickness());
    emitDouble(sink, "scalars.thickLoc", ad.getThicknessLocation());
    emitDouble(sink, "scalars.camber", ad.getCamber());
    emitDouble(sink, "scalars.camberLoc", ad.getCamberLocation());
}

static void scenarioNameAndCreated(HarnessSink& sink)
{
    AirfoilData ad;
    ad.setAirfoilName("TestAirfoil");
    emitString(sink, "name.after", ad.getAirfoilName());
    ad.setIsCreated(true);
    emitBool(sink, "created.true", ad.isCreated());
    ad.setIsCreated(false);
    emitBool(sink, "created.false", ad.isCreated());
}

// An output callback that collects the emitted messages.
static void collectPrint(void* context, const std::string& text, AirfoilDataMessageType type)
{
    std::vector<std::string>* out = static_cast<std::vector<std::string>*>(context);
    std::string prefix = "?";
    switch (type)
    {
        case AirfoilDataMessageType::Normal:        prefix = "Normal";        break;
        case AirfoilDataMessageType::Info:          prefix = "Info";          break;
        case AirfoilDataMessageType::Warning:       prefix = "Warning";       break;
        case AirfoilDataMessageType::Error:         prefix = "Error";         break;
        case AirfoilDataMessageType::WarningSevere: prefix = "WarningSevere"; break;
        case AirfoilDataMessageType::ErrorSevere:   prefix = "ErrorSevere";   break;
    }
    out->push_back(prefix + ": " + text);
}

static void scenarioOutputMethod(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<std::string> collected;
    ad.setOutputMethod(&collectPrint, static_cast<void*>(&collected));
    emitBool(sink, "output.hasMethod", ad.outputMethod() != nullptr);
    emitBool(sink, "output.hasContext", ad.outputContext() != nullptr);
    ad.emitOutput("hello", AirfoilDataMessageType::Info);
    ad.emitOutput("warning", AirfoilDataMessageType::Warning);
    ad.emitOutput("error", AirfoilDataMessageType::Error);
    for (std::size_t i = 0; i < collected.size(); i++)
        emitString(sink, "output.cb", collected[i]);
    // Suppression: after suppressing, no further messages.
    ad.setOutputSuppressed(true);
    emitBool(sink, "output.suppressed", ad.outputSuppressed());
    ad.emitOutput("invisible", AirfoilDataMessageType::Error);
    emitInt(sink, "output.cbCountAfterSuppress", static_cast<int>(collected.size()));
    ad.setOutputSuppressed(false);
    // Without callback -> console fallback (gets captured).
    AirfoilData ad2;
    {
        ConsoleCapture capture;
        ad2.emitOutput("console path", AirfoilDataMessageType::Warning);
        emitConsole(sink, "output.console", capture.text());
    }
}

static void scenarioVerbosity(HarnessSink& sink)
{
    // setVerbosityLevel has no getter; it becomes visible through the debug
    // output of the geometry methods. We run applyThickness at level 2.
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    loadInto(ad, xi, yi);
    ad.setVerbosityLevel(2);
    {
        ConsoleCapture capture;
        (void) ad.applyThickness(ThicknessJob::ScaleOnly, xi, yi, 0.12);
        emitConsole(sink, "verbosity.level2", capture.text());
    }
    ad.setVerbosityLevel(0);
    {
        ConsoleCapture capture;
        (void) ad.applyThickness(ThicknessJob::ScaleOnly, xi, yi, 0.12);
        emitConsole(sink, "verbosity.level0", capture.text());
    }
}

static void scenarioDesiredPoints(HarnessSink& sink)
{
    AirfoilData ad;
    emitBool(sink, "desired.set255", ad.setNumberOfDesiredAirfoilPoints(255));
    emitInt(sink, "desired.got255", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
    emitBool(sink, "desired.set100", ad.setNumberOfDesiredAirfoilPoints(100));
    emitInt(sink, "desired.got100", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
    emitBool(sink, "desired.set50", ad.setNumberOfDesiredAirfoilPoints(50));   // too small -> false
    emitInt(sink, "desired.got50", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
    emitBool(sink, "desired.set600", ad.setNumberOfDesiredAirfoilPoints(600)); // too large -> false
    emitInt(sink, "desired.got600", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
    emitBool(sink, "desired.set101even", ad.setNumberOfDesiredAirfoilPoints(101));
    emitInt(sink, "desired.got101", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
}

static void scenarioCreateXDistribution(HarnessSink& sink)
{
    const std::vector<double> cos = AirfoilData::createXDistribution(101, XDistributionMode::Cosine);
    const std::vector<double> half = AirfoilData::createXDistribution(101, XDistributionMode::HalfCosine);
    const std::vector<double> def = AirfoilData::createXDistribution(101);
    emitVector(sink, "xdist.cosine", cos);
    emitVector(sink, "xdist.halfcosine", half);
    emitVector(sink, "xdist.default", def);
    // Out of the allowed 99..511 interval -> empty vector.
    emitVectorSize(sink, "xdist.zero", AirfoilData::createXDistribution(0));
    emitVectorSize(sink, "xdist.one", AirfoilData::createXDistribution(1));
    emitVectorSize(sink, "xdist.below99", AirfoilData::createXDistribution(41));
    emitVectorSize(sink, "xdist.above511", AirfoilData::createXDistribution(600));
    // Interval boundaries; the generator itself is not odd-adjusted
    // (100 -> 100 points).
    emitVectorSize(sink, "xdist.min99", AirfoilData::createXDistribution(99));
    emitVectorSize(sink, "xdist.even100", AirfoilData::createXDistribution(100));
    emitVectorSize(sink, "xdist.max511", AirfoilData::createXDistribution(511));
    const std::vector<double> d = AirfoilData::createXDistribution(99);
    emitDouble(sink, "xdist.min99.first", d.front());
    emitDouble(sink, "xdist.min99.last", d.back());
    emitVectorSize(sink, "xdist.cosine511",
                   AirfoilData::createXDistribution(511, XDistributionMode::Cosine));
}

static void scenarioInitClear(HarnessSink& sink)
{
    AirfoilData ad;
    ad.initAirfoil(101);
    emitVectorSize(sink, "init101.airfoilX", ad.getAirfoilX());
    emitVectorSize(sink, "init101.camberX", ad.getCamberLineX());
    emitVectorSize(sink, "init101.thickDistX", ad.getThicknessDistributionX());
    emitInt(sink, "init101.desired", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
    ad.clearAirfoilData();
    emitVectorSize(sink, "clear.airfoilX", ad.getAirfoilX());
    // initAirfoil() uses the same 99..511 interval as the other point count
    // APIs: out of range -> rejected (data cleared, desired count unchanged),
    // even counts adjusted to odd.
    ad.initAirfoil(1);
    emitVectorSize(sink, "init1.airfoilX", ad.getAirfoilX());
    emitInt(sink, "init1.desired", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
    ad.initAirfoil(41);
    emitVectorSize(sink, "init41.airfoilX", ad.getAirfoilX());
    emitInt(sink, "init41.desired", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
    ad.initAirfoil(100);
    emitVectorSize(sink, "init100.airfoilX", ad.getAirfoilX());
    emitInt(sink, "init100.desired", static_cast<int>(ad.getNumberOfDesiredAirfoilPoints()));
}

static void scenarioVectorSetters(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> v(4, 0.0);
    v[0] = 1.0; v[1] = 2.0; v[2] = 3.0; v[3] = 4.0;
    ad.setPointsX(v);
    ad.setPointsY(v);
    ad.setCamberLineX(v);
    ad.setCamberLineY(v);
    ad.setThicknessX(v);
    ad.setThicknessY(v);
    ad.setPointsUpperX(v);
    ad.setPointsUpperY(v);
    ad.setPointsLowerX(v);
    ad.setPointsLowerY(v);
    ad.setPointsCamberDerivativeX(v);
    ad.setPointsCamberDerivativeY(v);
    ad.setPointsThicknessDerivativeX(v);
    ad.setPointsThicknessDerivativeY(v);
    ad.setPointsConformalFunctionX(v);
    ad.setPointsConformalFunctionY(v);
    ad.setPointsConformalCircleX(v);
    ad.setPointsConformalCircleY(v);
    ad.setCpUpperX(v);
    ad.setCpLowerX(v);
    std::vector<std::vector<double>> m(2, v);
    ad.setCpUpper(m);
    ad.setCpLower(m);
    emitVector(sink, "vset.airfoilX", ad.getAirfoilX());
    emitVector(sink, "vset.camberX", ad.getCamberLineX());
    emitVector(sink, "vset.thickDistX", ad.getThicknessDistributionX());
    emitVector(sink, "vset.upperX", ad.getUpperSurfaceX());
    emitVector(sink, "vset.lowerY", ad.getLowerSurfaceY());
    emitVector(sink, "vset.camberDerivY", ad.getCamberDerivativeY());
    emitVector(sink, "vset.thickDerivY", ad.getThicknessDerivativeY());
    emitVector(sink, "vset.confFuncY", ad.getConformalFunctionY());
    emitVector(sink, "vset.confCircleY", ad.getConformalCircleY());
    emitVector(sink, "vset.cpUpperX", ad.getCpUpperX());
    emitVector2d(sink, "vset.upperCp", ad.getUpperCp());
    emitVector2d(sink, "vset.lowerCp", ad.getLowerCp());
    emitInt(sink, "vset.nCamberDeriv", ad.getNumberOfCamberDerivativePoints());
    emitInt(sink, "vset.nThickDeriv", ad.getNumberOfThicknessDerivativePoints());
    emitInt(sink, "vset.nConfFunc", ad.getNumberOfConformalFunctionPoints());
    emitInt(sink, "vset.nConfCircle", ad.getNumberOfConformalCirclePoints());
    // Combined setter setAirfoilPoints(): consistent sizes -> true, assigned
    // and all derived data cleared; mismatch -> false with everything left
    // unchanged.
    std::vector<double> w(3, 0.0);
    w[0] = 5.0; w[1] = 6.0; w[2] = 7.0;
    ad.setIsCreated(true);
    emitBool(sink, "vset.afpoints.ok", ad.setAirfoilPoints(v, v));
    // The derived vectors set earlier must be gone after the new contour.
    emitInt(sink, "vset.afpoints.nCamberAfter", ad.getNumberOfCamberPoints());
    emitVectorSize(sink, "vset.afpoints.thickDistAfter", ad.getThicknessDistributionX());
    emitDouble(sink, "vset.afpoints.thicknessAfter", ad.getThickness());
    emitBool(sink, "vset.afpoints.createdAfter", ad.isCreated());
    {
        ConsoleCapture capture;
        emitBool(sink, "vset.afpoints.mismatch", ad.setAirfoilPoints(v, w));
        emitConsole(sink, "vset.afpoints", capture.text());
    }
    emitVector(sink, "vset.afpoints.x", ad.getAirfoilX());
    emitVector(sink, "vset.afpoints.y", ad.getAirfoilY());
}

static void scenarioNoseIndex(HarnessSink& sink)
{
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    AirfoilData ad;
    std::size_t nose = 0;
    ConsoleCapture capture;
    emitBool(sink, "nose.circle.secant.ok", ad.getNoseIndex(xi, yi, NoseDetectionMode::LongestSecant, nose));
    emitInt(sink, "nose.circle.secant", static_cast<int>(nose));
    emitBool(sink, "nose.circle.min.ok", ad.getNoseIndex(xi, yi, NoseDetectionMode::LocalMinimum, nose));
    emitInt(sink, "nose.circle.min", static_cast<int>(nose));
    // Open, asymmetric trailing edge (corners moved apart vertically): the
    // secant origin is the midpoint of the two contour endpoints, so the
    // winner is the x-minimum point and not an upper-surface point biased
    // by the lower trailing edge corner.
    std::vector<double> ox;
    std::vector<double> oy;
    circleProfile(41, ox, oy);
    oy[0] = -0.3;
    oy[ox.size() - 1] = 0.3;
    emitBool(sink, "nose.opente.secant.ok", ad.getNoseIndex(ox, oy, NoseDetectionMode::LongestSecant, nose));
    emitInt(sink, "nose.opente.secant", static_cast<int>(nose));
    // Too few points -> error path.
    std::vector<double> tx;
    std::vector<double> ty;
    circleProfile(3, tx, ty);
    emitBool(sink, "nose.tiny.secant.ok", ad.getNoseIndex(tx, ty, NoseDetectionMode::LongestSecant, nose));
    emitBool(sink, "nose.tiny.min.ok", ad.getNoseIndex(tx, ty, NoseDetectionMode::LocalMinimum, nose));
    emitConsole(sink, "nose", capture.text());
}

static void scenarioReorderSort(HarnessSink& sink)
{
    // reorderData (uses sortDataClockwise/removeConsecutiveDuplicates indirectly).
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    // deliberately passed in reversed order.
    std::vector<double> rx(xi.rbegin(), xi.rend());
    std::vector<double> ry(yi.rbegin(), yi.rend());
    ad.reorderData(rx, ry);
    emitVector(sink, "reorder.x", rx);
    emitVector(sink, "reorder.y", ry);
}

static void scenarioScaleRotate(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    std::vector<double> xo(xi.size());
    std::vector<double> yo(xi.size());
    ConsoleCapture capture;
    emitBool(sink, "scale.ok", ad.scale(xi, yi, xo, yo, 0.0, 1.0));
    emitVector(sink, "scale.x", xo);
    emitBool(sink, "scale.bad", ad.scale(xi, yi, xo, yo, 1.0, 0.0));
    // rotate() takes degrees.
    emitBool(sink, "rotate.ok", ad.rotate(xi, yi, xo, yo, 30.0));
    emitVector(sink, "rotate.x", xo);
    emitVector(sink, "rotate.y", yo);
    emitBool(sink, "rotate.zero", ad.rotate(xi, yi, xo, yo, 0.0));
    emitConsole(sink, "scalerotate", capture.text());
}

static void scenarioNormalize(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    ConsoleCapture capture;
    std::vector<double> xo = xi;
    std::vector<double> yo(xi.size());
    double angle = 0.0;
    emitBool(sink, "norm.none.none", ad.normalizeAirfoil(xi, yi, xo, yo, SortMode::None, InterpolationMode::None, angle));
    emitDouble(sink, "norm.none.none.angle", angle);
    emitVector(sink, "norm.none.none.x", xo);
    emitVector(sink, "norm.none.none.y", yo);
    // sort + map onto target support points
    std::vector<double> xtarget = AirfoilData::createXDistribution(21);
    std::vector<double> ytarget(xtarget.size());
    emitBool(sink, "norm.sort.map", ad.normalizeAirfoil(xi, yi, xtarget, ytarget, SortMode::Sort, InterpolationMode::Map, angle));
    emitDouble(sink, "norm.sort.map.angle", angle);
    emitVector(sink, "norm.sort.map.x", xtarget);
    emitVector(sink, "norm.sort.map.y", ytarget);
    // Inconsistent sizes without interpolation -> error.
    std::vector<double> xbad(5, 0.0);
    std::vector<double> ybad;
    emitBool(sink, "norm.sizemismatch", ad.normalizeAirfoil(xi, yi, xbad, ybad, SortMode::None, InterpolationMode::None, angle));
    // Verbosity > 1 with MORE target points than input points (15 -> 21):
    // the per-point debug lines must not read the source buffers beyond
    // their nInSize elements (regression guard for the xBuffer2 OOB read).
    {
        std::vector<double> sx;
        std::vector<double> sy;
        circleProfile(15, sx, sy);
        std::vector<double> xbig = AirfoilData::createXDistribution(21);
        std::vector<double> ybig(xbig.size());
        ad.setVerbosityLevel(2);
        emitBool(sink, "norm.bigout", ad.normalizeAirfoil(sx, sy, xbig, ybig, SortMode::Sort, InterpolationMode::Map, angle));
        ad.setVerbosityLevel(0);
        emitVectorSize(sink, "norm.bigout.x", xbig);
    }
    emitConsole(sink, "normalize", capture.text());
}

static void scenarioRepairXHooks(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    std::size_t removed = 0;
    ConsoleCapture capture;
    emitBool(sink, "hooks.clean", ad.repairXHooks(xi, yi, removed));
    emitInt(sink, "hooks.clean.removed", static_cast<int>(removed));
    // Inconsistent sizes -> error.
    std::vector<double> xb(4, 0.0);
    std::vector<double> yb(3, 0.0);
    emitBool(sink, "hooks.mismatch", ad.repairXHooks(xb, yb, removed));
    emitInt(sink, "hooks.mismatch.removed", static_cast<int>(removed));
    emitConsole(sink, "hooks", capture.text());
}

static void scenarioMapToSupport(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    std::vector<double> rx;
    std::vector<double> ry;
    ConsoleCapture capture;
    emitBool(sink, "map.ok", ad.mapAirfoilToSupportPoints(xi, yi, rx, ry, 21));
    emitVector(sink, "map.x", rx);
    emitVector(sink, "map.y", ry);
    emitBool(sink, "map.bad", ad.mapAirfoilToSupportPoints(xi, yi, rx, ry, 0));
    emitConsole(sink, "map", capture.text());
}

static void scenarioRedistribute(HarnessSink& sink, const std::vector<double>& profileX, const std::vector<double>& profileY)
{
    {
        AirfoilData ad;
        loadInto(ad, profileX, profileY);
        ConsoleCapture capture;
        emitBool(sink, "redist.zero", ad.redistributeAirfoil(0, RedistributeMode::AirfoilOnly));
        emitConsole(sink, "redist.zero", capture.text());
    }
    {
        AirfoilData ad;
        loadInto(ad, profileX, profileY);
        ConsoleCapture capture;
        emitBool(sink, "redist.101", ad.redistributeAirfoil(101, RedistributeMode::AirfoilOnly));
        emitVectorSize(sink, "redist.101.x", ad.getAirfoilX());
        emitConsole(sink, "redist.101", capture.text());
    }
    {
        AirfoilData ad;
        loadInto(ad, profileX, profileY);
        ConsoleCapture capture;
        emitBool(sink, "redist.101.all", ad.redistributeAirfoil(101, RedistributeMode::AllData));
        emitVectorSize(sink, "redist.101.all.x", ad.getAirfoilX());
        emitConsole(sink, "redist.101.all", capture.text());
    }
    {
        // Even point count is decremented to odd (100 -> 99).
        AirfoilData ad;
        loadInto(ad, profileX, profileY);
        ConsoleCapture capture;
        emitBool(sink, "redist.even", ad.redistributeAirfoil(100, RedistributeMode::AirfoilOnly));
        emitVectorSize(sink, "redist.even.x", ad.getAirfoilX());
        emitConsole(sink, "redist.even", capture.text());
    }
    {
        // Outside 99..511 -> error.
        AirfoilData ad;
        loadInto(ad, profileX, profileY);
        ConsoleCapture capture;
        emitBool(sink, "redist.600", ad.redistributeAirfoil(600, RedistributeMode::AirfoilOnly));
        emitConsole(sink, "redist.600", capture.text());
    }
}

static void scenarioThickness(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    double thick = 0.0;
    double atLoc = 0.0;
    double loc = 0.0;
    ConsoleCapture capture;
    emitBool(sink, "thick.abs.two", ad.calculateThickness(xi, yi, ThicknessMode::Absolute, 0.5, thick, atLoc, loc, SideMode::TwoSided));
    emitDouble(sink, "thick.abs.two.thick", thick);
    emitDouble(sink, "thick.abs.two.atLoc", atLoc);
    emitDouble(sink, "thick.abs.two.loc", loc);
    emitBool(sink, "thick.proj.two", ad.calculateThickness(xi, yi, ThicknessMode::Projected, 0.5, thick, atLoc, loc, SideMode::TwoSided));
    emitDouble(sink, "thick.proj.two.thick", thick);
    emitBool(sink, "thick.abs.one", ad.calculateThickness(xi, yi, ThicknessMode::Absolute, 0.5, thick, atLoc, loc, SideMode::OneSided));
    emitDouble(sink, "thick.abs.one.thick", thick);
    // Too few points -> error.
    std::vector<double> tx;
    std::vector<double> ty;
    circleProfile(3, tx, ty);
    emitBool(sink, "thick.tiny", ad.calculateThickness(tx, ty, ThicknessMode::Absolute, 0.5, thick, atLoc, loc, SideMode::TwoSided));
    emitConsole(sink, "thickness", capture.text());
}

static void scenarioApplyThickness(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    emitBool(sink, "apply.scale", ad.applyThickness(ThicknessJob::ScaleOnly, xi, yi, 0.12));
    emitVector(sink, "apply.scale.y", yi);
    std::vector<double> xj;
    std::vector<double> yj;
    circleProfile(41, xj, yj);
    emitBool(sink, "apply.fixed", ad.applyThickness(ThicknessJob::FixedThickness, xj, yj, 0.12));
    emitVector(sink, "apply.fixed.y", yj);
    emitBool(sink, "apply.zero", ad.applyThickness(ThicknessJob::FixedThickness, xj, yj, 0.0));
}

static void scenarioDyNose(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    double noseHeight = 0.0;
    ConsoleCapture capture;
    emitBool(sink, "dynose.ok", ad.calculateDyNose(xi, yi, noseHeight));
    emitDouble(sink, "dynose.value", noseHeight);
    std::vector<double> tx;
    std::vector<double> ty;
    circleProfile(3, tx, ty);
    emitBool(sink, "dynose.tiny", ad.calculateDyNose(tx, ty, noseHeight));
    emitConsole(sink, "dynose", capture.text());
}

static void scenarioArea(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    double area = 0.0;
    ConsoleCapture capture;
    emitBool(sink, "area.ok", ad.calculateArea(xi, yi, area));
    emitDouble(sink, "area.value", area);
    std::vector<double> tx;
    std::vector<double> ty;
    circleProfile(3, tx, ty);
    emitBool(sink, "area.tiny", ad.calculateArea(tx, ty, area));
    emitConsole(sink, "area", capture.text());
}

static void scenarioCamberAndThickness(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    loadInto(ad, xi, yi);
    ConsoleCapture capture;
    emitBool(sink, "camthick.circle", ad.calculateAirfoilCamberAndThickness());
    emitVectorSize(sink, "camthick.camberY", ad.getCamberLineY());
    emitVectorSize(sink, "camthick.thickDistY", ad.getThicknessDistributionY());
    emitConsole(sink, "camthick.circle", capture.text());
}

static void scenarioTeThickness(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    loadInto(ad, xi, yi);
    ad.calculateTeThickness();
    emitDouble(sink, "te.circle", ad.getTrailingEdgeThickness());
}

static void scenarioExtractCamberLine(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    std::vector<double> xthick = AirfoilData::createXDistribution(41);
    std::vector<double> ythick;
    std::vector<double> ycamber;
    ConsoleCapture capture;
    emitBool(sink, "extract.simple", ad.extractCamberLine(xi, yi, xthick, ythick, ycamber, CamberDifferenceMode::Simple));
    emitVector(sink, "extract.simple.ythick", ythick);
    emitVector(sink, "extract.simple.ycamber", ycamber);
    std::vector<double> ythick2;
    std::vector<double> ycamber2;
    emitBool(sink, "extract.correct", ad.extractCamberLine(xi, yi, xthick, ythick2, ycamber2, CamberDifferenceMode::Correct));
    // Inconsistent sizes -> error.
    std::vector<double> xbad = AirfoilData::createXDistribution(21);
    std::vector<double> ythick3;
    std::vector<double> ycamber3;
    emitBool(sink, "extract.sizemismatch", ad.extractCamberLine(xi, yi, xbad, ythick3, ycamber3, CamberDifferenceMode::Simple));
    emitConsole(sink, "extract", capture.text());
}

static void scenarioDerivatives(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    std::vector<double> fLow, sLow, tLow, fUp, sUp, tUp;
    ConsoleCapture capture;
    emitBool(sink, "deriv.two", ad.calculateDerivatives(xi, yi, fLow, sLow, tLow, fUp, sUp, tUp, SideMode::TwoSided));
    emitVector(sink, "deriv.two.fLow", fLow);
    emitVector(sink, "deriv.two.sUp", sUp);
    emitVectorSize(sink, "deriv.two.tUp", tUp);
    std::vector<double> fLow2, sLow2, tLow2, fUp2, sUp2, tUp2;
    emitBool(sink, "deriv.one", ad.calculateDerivatives(xi, yi, fLow2, sLow2, tLow2, fUp2, sUp2, tUp2, SideMode::OneSided));
    emitVector(sink, "deriv.one.fUp", fUp2);
    emitVectorSize(sink, "deriv.one.fLow", fLow2);
    // Too few points -> error.
    std::vector<double> tx;
    std::vector<double> ty;
    circleProfile(3, tx, ty);
    std::vector<double> fL3, sL3, tL3, fU3, sU3, tU3;
    emitBool(sink, "deriv.tiny", ad.calculateDerivatives(tx, ty, fL3, sL3, tL3, fU3, sU3, tU3, SideMode::TwoSided));
    emitConsole(sink, "derivatives", capture.text());
}

static void scenarioCirculation(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    loadInto(ad, xi, yi);
    (void) ad.calculateAirfoilCamberAndThickness();
    const std::vector<double> camberX = ad.getCamberLineX();
    const std::vector<double> camberY = ad.getCamberLineY();
    std::vector<double> cx;
    std::vector<double> cy;
    ConsoleCapture capture;
    emitBool(sink, "circ.alpha0", ad.calculateCirculation(camberX, camberY, cx, cy, 0.0));
    emitVector(sink, "circ.x", cx);
    emitVector(sink, "circ.y", cy);
    emitDouble(sink, "circ.alpha0Geo", ad.getAlpha0Geo());
    emitDouble(sink, "circ.cm0Geo", ad.getCm0Geo());
    std::vector<double> cx2;
    std::vector<double> cy2;
    emitBool(sink, "circ.cl05", ad.calculateCirculationCl05(camberX, camberY, cx2, cy2));
    emitVector(sink, "circ.cl05.x", cx2);
    emitVector(sink, "circ.cl05.y", cy2);
    // Invalid camber line -> error.
    std::vector<double> tinyX;
    std::vector<double> tinyY;
    circleProfile(3, tinyX, tinyY);
    std::vector<double> cx3;
    std::vector<double> cy3;
    emitBool(sink, "circ.tiny", ad.calculateCirculation(tinyX, tinyY, cx3, cy3, 0.0));
    // Negative x (outside [0,1]) -> rejected before acos could produce NaN.
    std::vector<double> negX = {0.0, -0.5, 0.5, 1.0};
    std::vector<double> negY = {0.0, 0.1, 0.1, 0.0};
    std::vector<double> cx4;
    std::vector<double> cy4;
    emitBool(sink, "circ.negx", ad.calculateCirculation(negX, negY, cx4, cy4, 0.0));
    emitConsole(sink, "circulation", capture.text());
}

static void scenarioSourceStrength(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    loadInto(ad, xi, yi);
    (void) ad.calculateAirfoilCamberAndThickness();
    const std::vector<double> thickX = ad.getThicknessDistributionX();
    const std::vector<double> thickY = ad.getThicknessDistributionY();
    std::vector<double> xq;
    std::vector<double> q;
    ConsoleCapture capture;
    emitBool(sink, "source.ok", ad.calculateSourceStrength(thickX, thickY, xq, q));
    emitVector(sink, "source.xq", xq);
    emitVector(sink, "source.q", q);
    std::vector<double> tinyX;
    std::vector<double> tinyY;
    circleProfile(3, tinyX, tinyY);
    std::vector<double> xq2;
    std::vector<double> q2;
    emitBool(sink, "source.tiny", ad.calculateSourceStrength(tinyX, tinyY, xq2, q2));
    emitConsole(sink, "source", capture.text());
}

static void scenarioUpdates(HarnessSink& sink, const std::vector<double>& profileX, const std::vector<double>& profileY)
{
    {
        AirfoilData ad;
        loadInto(ad, profileX, profileY);
        (void) ad.calculateAirfoilCamberAndThickness();
        ConsoleCapture capture;
        emitBool(sink, "upd.circ", ad.updateCirculation(RecalcMode::Recompute));
        emitBool(sink, "upd.source", ad.updateSourceStrength(RecalcMode::Recompute));
        emitBool(sink, "upd.both", ad.updateCirculationAndSourceStrength());
        emitVectorSize(sink, "upd.circX", ad.getCirculationX());
        emitVectorSize(sink, "upd.sourceX", ad.getSourceX());
        emitConsole(sink, "upd", capture.text());
    }
    {
        // update without prior computation (reCalc=false) -> error path possible.
        AirfoilData ad;
        ConsoleCapture capture;
        emitBool(sink, "upd.circ.empty", ad.updateCirculation(RecalcMode::Keep));
        emitBool(sink, "upd.source.empty", ad.updateSourceStrength(RecalcMode::Keep));
        emitConsole(sink, "upd.empty", capture.text());
    }
}

static void scenarioUpperLowerGradient(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    loadInto(ad, xi, yi);
    ConsoleCapture capture;
    emitBool(sink, "ulc.ok", ad.calculateUpperLowerCoordinates());
    emitVectorSize(sink, "ulc.upperX", ad.getUpperSurfaceX());
    emitVectorSize(sink, "ulc.lowerX", ad.getLowerSurfaceX());
    emitBool(sink, "grad.ok", ad.calculateGradientCurvature());
    emitVector(sink, "grad.upper", ad.getGradientDataUpper());
    emitVector(sink, "grad.lower", ad.getGradientDataLower());
    emitVector(sink, "grad.upperX", ad.getGradientDataUpperX());
    emitVector(sink, "grad.lowerX", ad.getGradientDataLowerX());
    emitVector(sink, "curv.upper", ad.getCurvatureDataUpper());
    emitVector(sink, "curv.lower", ad.getCurvatureDataLower());
    const GradientData gd = ad.getGradientData();
    emitBool(sink, "gdata.valid", gd.valid);
    emitVectorSize(sink, "gdata.upperX", gd.upperX);
    emitVectorSize(sink, "gdata.lowerGradient", gd.lowerGradient);
    emitVectorSize(sink, "gdata.upperCurvature", gd.upperCurvature);
    emitInt(sink, "grad.nUpper", ad.getNumberOfUpperGradientPoints());
    emitInt(sink, "grad.nLower", ad.getNumberOfLowerGradientPoints());
    emitConsole(sink, "ulc", capture.text());
}

static void scenarioGeometryData(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> xi;
    std::vector<double> yi;
    circleProfile(41, xi, yi);
    loadInto(ad, xi, yi);
    // calculateAirfoilGeometryData requires existing camber points.
    (void) ad.calculateAirfoilCamberAndThickness();
    ConsoleCapture capture;
    emitBool(sink, "geom.ok", ad.calculateAirfoilGeometryData());
    emitDouble(sink, "geom.thickness", ad.getThickness());
    emitDouble(sink, "geom.area", ad.getArea());
    emitDouble(sink, "geom.dyNose", ad.getDyNose());
    emitConsole(sink, "geom", capture.text());
}

static void scenarioAeroData(HarnessSink& sink)
{
    AirfoilData ad;
    std::vector<double> alphas;
    alphas.push_back(-2.0);
    alphas.push_back(0.0);
    alphas.push_back(2.0);
    std::vector<double> cd = {0.01, 0.009, 0.011};
    std::vector<double> cdf = {0.007, 0.006, 0.007};
    std::vector<double> cdp = {0.003, 0.003, 0.004};
    std::vector<double> cl = {0.0, 0.2, 0.4};
    std::vector<double> cm = {-0.05, -0.04, -0.03};
    std::vector<double> eCur = {10.0, 15.0, 12.0};
    std::vector<bool> conv = {true, true, true};
    AeroScalars scalars;
    scalars.alpha0 = -1.5;
    scalars.cl0 = 0.11;
    scalars.cd0 = 0.009;
    scalars.cm0 = -0.05;
    scalars.clMax = 1.2;
    scalars.eMax = 35.0;
    ConsoleCapture capture;
    emitBool(sink, "aero.ok", ad.setAerodynamicData(alphas, cd, cdf, cdp, cl, cm, scalars, eCur, conv));
    emitVector(sink, "aero.alphas", ad.getAlphas());
    emitVector(sink, "aero.cl", ad.getCl());
    emitVector(sink, "aero.cd", ad.getCd());
    emitVector(sink, "aero.cdf", ad.getCdf());
    emitVector(sink, "aero.cdp", ad.getCdp());
    emitVector(sink, "aero.cm", ad.getCm());
    emitVector(sink, "aero.e", ad.getE());
    emitVectorBool(sink, "aero.converged", ad.getConverged());
    emitDouble(sink, "aero.alpha0", ad.getAlpha0());
    emitDouble(sink, "aero.cm0", ad.getCm0());
    emitDouble(sink, "aero.cl0", ad.getCl0());
    emitDouble(sink, "aero.cd0", ad.getCd0());
    emitDouble(sink, "aero.clMax", ad.getClMax());
    emitDouble(sink, "aero.eMax", ad.getEMax());
    // Size mismatch -> error.
    std::vector<double> badCl = {0.0, 0.2};
    emitBool(sink, "aero.mismatch", ad.setAerodynamicData(alphas, cd, cdf, cdp, badCl, cm, scalars, eCur, conv));
    // Non-finite scalar (cm0) -> error, stored data unchanged.
    AeroScalars badScalars = scalars;
    badScalars.cm0 = std::numeric_limits<double>::quiet_NaN();
    emitBool(sink, "aero.nonFinite", ad.setAerodynamicData(alphas, cd, cdf, cdp, cl, cm, badScalars, eCur, conv));
    emitDouble(sink, "aero.cm0AfterReject", ad.getCm0());
    emitConsole(sink, "aero", capture.text());
}

static void scenarioAirfoilOK(HarnessSink& sink, const std::vector<double>& profileX, const std::vector<double>& profileY)
{
    {
        AirfoilData ad;
        loadInto(ad, profileX, profileY);
        ConsoleCapture capture;
        emitBool(sink, "ok.naca", ad.airfoilOK(TrailingEdgeCheck::Full));
        emitConsole(sink, "ok.naca", capture.text());
    }
    {
        AirfoilData ad;
        loadInto(ad, profileX, profileY);
        ConsoleCapture capture;
        emitBool(sink, "ok.naca.avoidTE", ad.airfoilOK(TrailingEdgeCheck::SkipNearEdge));
        emitConsole(sink, "ok.naca.avoidTE", capture.text());
    }
    {
        // Empty profile -> error.
        AirfoilData ad;
        ConsoleCapture capture;
        emitBool(sink, "ok.empty", ad.airfoilOK(TrailingEdgeCheck::Full));
        emitConsole(sink, "ok.empty", capture.text());
    }
}

static void scenarioProperties(HarnessSink& sink, const std::vector<double>& profileX, const std::vector<double>& profileY)
{
    {
        AirfoilData ad;
        loadInto(ad, profileX, profileY);
        ad.setAirfoilName("NACA4618");
        ad.setIsCreated(true);
        ConsoleCapture capture;
        emitBool(sink, "props.naca", ad.calculateAirfoilProperties());
        emitDouble(sink, "props.naca.thickness", ad.getThickness());
        emitDouble(sink, "props.naca.thickLoc", ad.getThicknessLocation());
        emitDouble(sink, "props.naca.camber", ad.getCamber());
        emitDouble(sink, "props.naca.camberLoc", ad.getCamberLocation());
        emitDouble(sink, "props.naca.area", ad.getArea());
        emitDouble(sink, "props.naca.dyNose", ad.getDyNose());
        emitDouble(sink, "props.naca.teThickness", ad.getTrailingEdgeThickness());
        emitDouble(sink, "props.naca.alpha0Geo", ad.getAlpha0Geo());
        emitDouble(sink, "props.naca.cm0Geo", ad.getCm0Geo());
        emitVector(sink, "props.naca.circX", ad.getCirculationX());
        emitVector(sink, "props.naca.circY", ad.getCirculationY());
        emitVector(sink, "props.naca.sourceX", ad.getSourceX());
        emitVector(sink, "props.naca.sourceY", ad.getSourceY());
        emitVector(sink, "props.naca.camberY", ad.getCamberLineY());
        emitVector(sink, "props.naca.thickDistY", ad.getThicknessDistributionY());
        emitVector(sink, "props.naca.gradUpper", ad.getGradientDataUpper());
        emitVector(sink, "props.naca.curvLower", ad.getCurvatureDataLower());
        emitInt(sink, "props.naca.nCamber", ad.getNumberOfCamberPoints());
        emitInt(sink, "props.naca.nCirc", ad.getNumberOfCirculationPoints());
        emitInt(sink, "props.naca.nSource", ad.getNumberOfSourcePoints());
        emitConsole(sink, "props.naca", capture.text());
    }
    {
        AirfoilData ad;
        std::vector<double> xi;
        std::vector<double> yi;
        circleProfile(15, xi, yi);
        loadInto(ad, xi, yi);
        ad.setIsCreated(true);
        ConsoleCapture capture;
        emitBool(sink, "props.small15", ad.calculateAirfoilProperties());
        emitDouble(sink, "props.small15.thickness", ad.getThickness());
        emitDouble(sink, "props.small15.area", ad.getArea());
        emitVectorSize(sink, "props.small15.circX", ad.getCirculationX());
        emitConsole(sink, "props.small15", capture.text());
    }
}

static void scenarioSave(HarnessSink& sink, const std::vector<double>& profileX, const std::vector<double>& profileY)
{
    AirfoilData ad;
    loadInto(ad, profileX, profileY);
    ad.setAirfoilName("NACA4618");
    // Success path: saveAirfoilCoordinates() requires created == true.
    ad.setIsCreated(true);
    ConsoleCapture capture;
    emitBool(sink, "save.ok", ad.saveAirfoilCoordinates("harness_saved.dat", "harness"));
    emitConsole(sink, "save.ok", capture.text());
    {
        // Not created -> error path, no file is written.
        ConsoleCapture capture2;
        ad.setIsCreated(false);
        emitBool(sink, "save.notCreated", ad.saveAirfoilCoordinates("harness_should_not_exist.dat", "harness"));
        emitConsole(sink, "save.notCreated", capture2.text());
    }
}

static void scenarioAkima(HarnessSink& sink)
{
    AkimaInterpolation ak;
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> y = {0.0, 1.0, 4.0, 9.0, 16.0};
    emitBool(sink, "akima.init", ak.init(x, y));
    double value = 0.0;
    double d1 = 0.0;
    double d2 = 0.0;
    emitBool(sink, "akima.value.ok", ak.interpolate(2.5, value));
    emitDouble(sink, "akima.value", value);
    emitBool(sink, "akima.first.ok", ak.interpolate(2.5, value, d1));
    emitDouble(sink, "akima.first.value", value);
    emitDouble(sink, "akima.first.d1", d1);
    emitBool(sink, "akima.second.ok", ak.interpolate(2.5, value, d1, d2));
    emitDouble(sink, "akima.second.value", value);
    emitDouble(sink, "akima.second.d1", d1);
    emitDouble(sink, "akima.second.d2", d2);
    // Extrapolation outside the support range.
    emitBool(sink, "akima.extrap.ok", ak.interpolate(-1.0, value));
    emitDouble(sink, "akima.extrap", value);
    // Not initialized -> false.
    AkimaInterpolation fresh;
    emitBool(sink, "akima.uninit", fresh.interpolate(1.0, value));
    // Invalid input (not ascending) -> init false.
    std::vector<double> badX = {0.0, 2.0, 1.0};
    std::vector<double> badY = {0.0, 1.0, 2.0};
    emitBool(sink, "akima.badinit", ak.init(badX, badY));
    // Two-point special case.
    AkimaInterpolation two;
    std::vector<double> x2 = {0.0, 1.0};
    std::vector<double> y2 = {0.0, 2.0};
    emitBool(sink, "akima.two.init", two.init(x2, y2));
    emitBool(sink, "akima.two.value.ok", two.interpolate(0.5, value));
    emitDouble(sink, "akima.two.value", value);
    // Huge y values (secant differences overflow to infinity): valid finite,
    // strictly ascending input, but the computed slopes are not finite ->
    // init must reject and leave the object unusable (two and three points).
    AkimaInterpolation hugeTwo;
    std::vector<double> hugeX2 = {0.0, 1.0};
    std::vector<double> hugeY2 = {1.0e308, -1.0e308};
    emitBool(sink, "akima.hugeslope.two.init", hugeTwo.init(hugeX2, hugeY2));
    emitBool(sink, "akima.hugeslope.two.interp", hugeTwo.interpolate(0.5, value));
    AkimaInterpolation hugeThree;
    std::vector<double> hugeX3 = {0.0, 1.0, 2.0};
    std::vector<double> hugeY3 = {1.0e308, -1.0e308, 1.0e308};
    emitBool(sink, "akima.hugeslope.three.init", hugeThree.init(hugeX3, hugeY3));
    emitBool(sink, "akima.hugeslope.three.interp", hugeThree.interpolate(1.5, value));
}

static void scenarioProfile(HarnessSink& sink, const std::vector<double>& profileX, const std::vector<double>& profileY)
{
    AirfoilData ad;
    loadInto(ad, profileX, profileY);
    ad.setAirfoilName("NACA4618");
    ad.setIsCreated(true);
    ConsoleCapture capture;
    (void) ad.calculateAirfoilProperties();
    emitVector(sink, "profile.airfoilX", ad.getAirfoilX());
    emitVector(sink, "profile.airfoilY", ad.getAirfoilY());
    emitVector(sink, "profile.upperX", ad.getUpperSurfaceX());
    emitVector(sink, "profile.upperY", ad.getUpperSurfaceY());
    emitVector(sink, "profile.lowerX", ad.getLowerSurfaceX());
    emitVector(sink, "profile.lowerY", ad.getLowerSurfaceY());
    emitVector(sink, "profile.camberX", ad.getCamberLineX());
    emitVector(sink, "profile.camberY", ad.getCamberLineY());
    emitVector(sink, "profile.thickDistX", ad.getThicknessDistributionX());
    emitVector(sink, "profile.thickDistY", ad.getThicknessDistributionY());
    emitVector(sink, "profile.camberDerivX", ad.getCamberDerivativeX());
    emitVector(sink, "profile.thickDerivX", ad.getThicknessDerivativeX());
    emitVector(sink, "profile.confFuncX", ad.getConformalFunctionX());
    emitVector(sink, "profile.confCircleX", ad.getConformalCircleX());
    emitConsole(sink, "profile", capture.text());
}

// ------------------------------------------------------------------
// Corpus
// ------------------------------------------------------------------

static void scenarioEdgeCases(HarnessSink& sink)
{
    ConsoleCapture capture;

    // --- Empty input: all methods should cleanly reject with false. ---
    {
        AirfoilData ad;
        std::vector<double> e;
        loadInto(ad, e, e);
        std::size_t nose = 0;
        std::size_t removed = 0;
        double thick = 0.0;
        double atLoc = 0.0;
        double loc = 0.0;
        double area = 0.0;
        double dy = 0.0;
        double angle = 0.0;
        std::vector<double> rx;
        std::vector<double> ry;
        std::vector<double> fL;
        std::vector<double> sL;
        std::vector<double> tL;
        std::vector<double> fU;
        std::vector<double> sU;
        std::vector<double> tU;
        std::vector<double> cx;
        std::vector<double> cy;
        std::vector<double> xq;
        std::vector<double> q;
        std::vector<double> yth;
        std::vector<double> ycm;
        emitBool(sink, "edge.empty.camber", ad.calculateAirfoilCamberAndThickness());
        emitBool(sink, "edge.empty.geom", ad.calculateAirfoilGeometryData());
        emitBool(sink, "edge.empty.props", ad.calculateAirfoilProperties());
        emitBool(sink, "edge.empty.airfoilOK", ad.airfoilOK(TrailingEdgeCheck::Full));
        emitBool(sink, "edge.empty.nose", ad.getNoseIndex(e, e, NoseDetectionMode::LocalMinimum, nose));
        emitBool(sink, "edge.empty.thick", ad.calculateThickness(e, e, ThicknessMode::Absolute, 0.5, thick, atLoc, loc, SideMode::TwoSided));
        emitBool(sink, "edge.empty.area", ad.calculateArea(e, e, area));
        emitBool(sink, "edge.empty.dynose", ad.calculateDyNose(e, e, dy));
        emitBool(sink, "edge.empty.deriv", ad.calculateDerivatives(e, e, fL, sL, tL, fU, sU, tU, SideMode::TwoSided));
        emitBool(sink, "edge.empty.circ", ad.calculateCirculation(e, e, cx, cy, 0.0));
        emitBool(sink, "edge.empty.source", ad.calculateSourceStrength(e, e, xq, q));
        emitBool(sink, "edge.empty.extract", ad.extractCamberLine(e, e, e, yth, ycm, CamberDifferenceMode::Simple));
        emitBool(sink, "edge.empty.scale", ad.scale(e, e, rx, ry, 0.0, 1.0));
        emitBool(sink, "edge.empty.rotate", ad.rotate(e, e, rx, ry, 10.0));
        emitBool(sink, "edge.empty.hooks", ad.repairXHooks(e, e, removed));
        emitBool(sink, "edge.empty.map", ad.mapAirfoilToSupportPoints(e, e, rx, ry, 21));
        emitBool(sink, "edge.empty.norm", ad.normalizeAirfoil(e, e, rx, ry, SortMode::None, InterpolationMode::None, angle));
        emitBool(sink, "edge.empty.redist", ad.redistributeAirfoil(21, RedistributeMode::AllData));
    }

    // --- Single point. ---
    {
        AirfoilData ad;
        std::vector<double> sx = {0.5};
        std::vector<double> sy = {0.0};
        loadInto(ad, sx, sy);
        std::size_t nose = 0;
        std::size_t removed = 0;
        double thick = 0.0;
        double atLoc = 0.0;
        double loc = 0.0;
        double area = 0.0;
        double angle = 0.0;
        std::vector<double> rx;
        std::vector<double> ry;
        emitBool(sink, "edge.one.camber", ad.calculateAirfoilCamberAndThickness());
        emitBool(sink, "edge.one.airfoilOK", ad.airfoilOK(TrailingEdgeCheck::Full));
        emitBool(sink, "edge.one.nose", ad.getNoseIndex(sx, sy, NoseDetectionMode::LocalMinimum, nose));
        emitBool(sink, "edge.one.thick", ad.calculateThickness(sx, sy, ThicknessMode::Absolute, 0.5, thick, atLoc, loc, SideMode::TwoSided));
        emitBool(sink, "edge.one.area", ad.calculateArea(sx, sy, area));
        emitBool(sink, "edge.one.hooks", ad.repairXHooks(sx, sy, removed));
        emitBool(sink, "edge.one.norm", ad.normalizeAirfoil(sx, sy, rx, ry, SortMode::None, InterpolationMode::None, angle));
    }

    // --- Non-monotone / duplicated x (x-hooks). ---
    {
        AirfoilData ad;
        std::vector<double> dx = {0.0, 0.5, 0.5, 1.0, 0.5, 0.0};
        std::vector<double> dyv = {0.0, 0.1, -0.1, 0.0, 0.05, -0.05};
        std::size_t nose = 0;
        std::size_t removed = 0;
        double angle = 0.0;
        std::vector<double> rx;
        std::vector<double> ry;
        emitBool(sink, "edge.dup.nose", ad.getNoseIndex(dx, dyv, NoseDetectionMode::LocalMinimum, nose));
        emitInt(sink, "edge.dup.nose.idx", static_cast<int>(nose));
        emitBool(sink, "edge.dup.hooks", ad.repairXHooks(dx, dyv, removed));
        emitInt(sink, "edge.dup.hooks.removed", static_cast<int>(removed));
        emitBool(sink, "edge.dup.norm", ad.normalizeAirfoil(dx, dyv, rx, ry, SortMode::Sort, InterpolationMode::None, angle));
    }

    // --- Very many points. ---
    {
        AirfoilData ad;
        std::vector<double> xi;
        std::vector<double> yi;
        circleProfile(2001, xi, yi);
        loadInto(ad, xi, yi);
        std::vector<double> rx;
        std::vector<double> ry;
        emitBool(sink, "edge.big.map", ad.mapAirfoilToSupportPoints(xi, yi, rx, ry, 201));
        emitVectorSize(sink, "edge.big.map.x", rx);
        emitBool(sink, "edge.big.camber", ad.calculateAirfoilCamberAndThickness());
        emitVectorSize(sink, "edge.big.camberX", ad.getCamberLineX());
    }

    // --- NaN / Inf in the coordinates (validation boundary). ---
    {
        AirfoilData ad;
        std::vector<double> nx;
        std::vector<double> ny;
        nx.push_back(0.0);
        nx.push_back(std::numeric_limits<double>::quiet_NaN());
        nx.push_back(1.0);
        ny.push_back(0.0);
        ny.push_back(0.1);
        ny.push_back(std::numeric_limits<double>::infinity());
        loadInto(ad, nx, ny);
        std::vector<double> rx;
        std::vector<double> ry;
        emitBool(sink, "edge.nan.airfoilOK", ad.airfoilOK(TrailingEdgeCheck::Full));
        emitBool(sink, "edge.nan.map", ad.mapAirfoilToSupportPoints(nx, ny, rx, ry, 21));
    }

    emitConsole(sink, "edge", capture.text());
}

void runAllScenarios(HarnessSink& sink, const std::string& profilePath)
{
    sink.emit("harness|version|" + std::string(AirfoilData::versionString()));

    std::vector<double> profileX;
    std::vector<double> profileY;
    const bool loaded = loadProfile(profilePath, profileX, profileY);
    sink.emit("profile|loaded|" + harnessBool(loaded));
    sink.emit("profile|size|" + std::to_string(profileX.size()));

    scenarioVersion(sink);
    scenarioDefaultState(sink);
    scenarioScalarSetters(sink);
    scenarioNameAndCreated(sink);
    scenarioOutputMethod(sink);
    scenarioVerbosity(sink);
    scenarioDesiredPoints(sink);
    scenarioCreateXDistribution(sink);
    scenarioInitClear(sink);
    scenarioVectorSetters(sink);
    scenarioNoseIndex(sink);
    scenarioReorderSort(sink);
    scenarioScaleRotate(sink);
    scenarioNormalize(sink);
    scenarioRepairXHooks(sink);
    scenarioMapToSupport(sink);
    scenarioThickness(sink);
    scenarioApplyThickness(sink);
    scenarioDyNose(sink);
    scenarioArea(sink);
    scenarioCamberAndThickness(sink);
    scenarioTeThickness(sink);
    scenarioExtractCamberLine(sink);
    scenarioDerivatives(sink);
    scenarioCirculation(sink);
    scenarioSourceStrength(sink);
    scenarioUpperLowerGradient(sink);
    scenarioGeometryData(sink);
    scenarioAeroData(sink);
    scenarioAkima(sink);
    scenarioEdgeCases(sink);

    if (loaded)
    {
        scenarioRedistribute(sink, profileX, profileY);
        scenarioUpdates(sink, profileX, profileY);
        scenarioAirfoilOK(sink, profileX, profileY);
        scenarioProperties(sink, profileX, profileY);
        scenarioSave(sink, profileX, profileY);
        scenarioProfile(sink, profileX, profileY);
    }
}
