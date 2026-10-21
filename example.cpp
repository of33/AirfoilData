// Minimal example for the library AirfoilData.
// Shows the most important public functions on a real airfoil.
//
// Usage:
//   ./example [airfoil.dat]        default: tests/data/NACA2312.dat

#include "AirfoilData.h"
#include "AkimaInterpolation.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// Reads an airfoil (comment lines '#', data lines: x y [yt]).
static bool loadAirfoil(const std::string& path,
                        std::vector<double>& x,
                        std::vector<double>& y)
{
    std::ifstream in(path);
    if (!in)
    {
        return false;
    }
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream ls(line);
        double xi = 0.0;
        double yi = 0.0;
        if (ls >> xi >> yi)
        {
            x.push_back(xi);
            y.push_back(yi);
        }
    }
    return x.size() >= 2;
}

// Writes a column table (whitespace-separated, scientific notation).
// Missing entries of shorter columns are filled with 'nan' so that every
// row has the same number of columns (gnuplot-friendly).
static bool writeColumns(const std::string& path,
                         const std::string& header,
                         const std::vector<std::vector<double>>& cols)
{
    std::ofstream out(path, std::ios::out);
    if (!out)
    {
        std::cerr << "Could not write output file: " << path << "\n";
        return false;
    }
    out << header;
    out << std::scientific << std::setprecision(8);
    std::size_t rows = 0;
    for (const std::vector<double>& c : cols)
    {
        rows = std::max(rows, c.size());
    }
    for (std::size_t i = 0; i < rows; i++)
    {
        for (std::size_t j = 0; j < cols.size(); j++)
        {
            if (j != 0)
            {
                out << ' ';
            }
            if (i < cols[j].size())
            {
                out << cols[j][i];
            }
            else
            {
                out << "nan";
            }
        }
        out << "\n";
    }
    return true;
}

// Writes two independent two-column blocks (separated by a blank line)
// into one file, so that distributions of different length produce no 'nan'
// fillers (gnuplot block format, readable via 'index 0' / 'index 1').
static bool writeTwoBlocks(const std::string& path,
                           const std::string& headerA,
                           const std::vector<double>& xA,
                           const std::vector<double>& yA,
                           const std::string& headerB,
                           const std::vector<double>& xB,
                           const std::vector<double>& yB)
{
    std::ofstream out(path, std::ios::out);
    if (!out)
    {
        std::cerr << "Could not write output file: " << path << "\n";
        return false;
    }
    out << std::scientific << std::setprecision(8);
    out << headerA;
    for (std::size_t i = 0; i < xA.size(); i++)
    {
        out << xA[i] << ' ' << yA[i] << "\n";
    }
    out << "\n\n";
    out << headerB;
    for (std::size_t i = 0; i < xB.size(); i++)
    {
        out << xB[i] << ' ' << yB[i] << "\n";
    }
    return true;
}

int main(int argc, char** argv)
{
    const std::string airfoilPath = (argc > 1) ? argv[1] : "tests/data/NACA2312.dat";

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\nAirfoilData "
              << AirfoilData::versionMajor() << "."
              << AirfoilData::versionMinor() << "."
              << AirfoilData::versionPatch()
              << " (" << AirfoilData::versionString() << ")\n\n";

    std::vector<double> x;
    std::vector<double> y;
    if (!loadAirfoil(airfoilPath, x, y))
    {
        std::cerr << "Could not read airfoil: " << airfoilPath << "\n";
        return 1;
    }
    std::cout << "Airfoil: " << airfoilPath << "  (" << x.size() << " points)\n";

    AirfoilData ad;
    ad.setAirfoilName("Example");
    if (!ad.setAirfoilPoints(x, y))
    {
        std::cerr << "Could not assign airfoil coordinates.\n";
        return 1;
    }

    if (!ad.airfoilOK(TrailingEdgeCheck::Full))
    {
        std::cout << "Note: airfoilOK() reports problems with the airfoil.\n";
    }

    // Full pipeline: camber/thickness + geometry + alpha0/cm0.
    if (!ad.calculateAirfoilProperties())
    {
        std::cerr << "calculateAirfoilProperties() failed\n";
        return 1;
    }

    std::cout << "\nGeometry:\n"
              << "  Thickness               = " << ad.getThickness() << "\n"
              << "  Thickness location      = " << ad.getThicknessLocation() << "\n"
              << "  Camber                  = " << ad.getCamber() << "\n"
              << "  Camber location         = " << ad.getCamberLocation() << "\n"
              << "  TE thickness            = " << ad.getTrailingEdgeThickness() << "\n"
              << "  Area                    = " << ad.getArea() << "\n"
              << "  Nose thickness at 1.25% = " << ad.getDyNose() << "\n"
              << "  alpha0 (inviscid)       = " << ad.getAlpha0Geo() << "\n"
              << "  cm0    (inviscid)       = " << ad.getCm0Geo() << "\n";

    // Akima interpolation on the computed camber line.
    const std::vector<double>& cx = ad.getCamberLineX();
    const std::vector<double>& cy = ad.getCamberLineY();
    AkimaInterpolation akima;
    if (akima.init(cx, cy))
    {
        std::cout << "\nCamber line (Akima):";
        for (double t : {0.25, 0.5, 0.75})
        {
            double yc = 0.0;
            if (akima.interpolate(t, yc))
            {
                std::cout << "  x=" << t << " y=" << yc;
            }
        }
        std::cout << "\n";
    }

    // Circulation and source strength: the update method takes care of the
    // internal cutting of the thickness distribution from the leading edge
    // (correct API usage).
    if (ad.updateCirculationAndSourceStrength())
    {
        std::cout << "Circulation:     " << ad.getCirculationX().size() << " support points\n"
                  << "Source strength: " << ad.getSourceX().size() << " support points\n";
    }

    // Redistribute the airfoil to 333 support points; AllData recomputes
    // camber/thickness, geometry, circulation, source strength, upper/lower
    // surface and slope/curvature.
    if (ad.redistributeAirfoil(333, RedistributeMode::AllData))
    {
        std::cout << "\nRedistribution to 333 points -> "
                  << ad.getAirfoilX().size() << " points active\n";
    }

    // Output files for gnuplot (base name 'example').
    const std::string base = "example";

    // (a) Airfoil coordinates + thickness distribution (full teardrop TE->nose->TE,
    //     symmetric, same number of points as the airfoil).
    writeColumns(base + "_airfoil.dat",
                 "#  x_airfoil       y_airfoil       x_thick         y_thick\n",
                 {ad.getAirfoilX(), ad.getAirfoilY(),
                  ad.getThicknessDistributionX(), ad.getThicknessDistributionY()});

    // (b) Camber line (one side, nose->TE).
    writeColumns(base + "_camber.dat",
                 "#  x               y\n",
                 {ad.getCamberLineX(), ad.getCamberLineY()});

    // (c) Circulation and source distribution: two separate blocks
    //     (different number of support points -> no nan fillers).
    writeTwoBlocks(base + "_circ_source.dat",
                   "#  x_circ       k_over_Uinf\n",
                   ad.getCirculationX(), ad.getCirculationY(),
                   "#  x_source  scaled_source_dzdx\n",
                   ad.getSourceX(), ad.getSourceY());

    // (d) Slope and curvature distribution (upper/lower).
    writeColumns(base + "_grad_curv.dat",
                 "#  x_upper         slope_upper     curv_upper      x_lower         slope_lower     curv_lower\n",
                 {ad.getGradientDataUpperX(), ad.getGradientDataUpper(), ad.getCurvatureDataUpper(),
                  ad.getGradientDataLowerX(), ad.getGradientDataLower(), ad.getCurvatureDataLower()});

    std::cout << "Output files written: "
              << base << "_airfoil.dat, " << base << "_camber.dat, "
              << base << "_circ_source.dat, " << base << "_grad_curv.dat\n";

    return 0;
}
