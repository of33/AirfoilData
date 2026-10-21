// Test program: runs the same scenario corpus as the golden generator
// and compares every emitted record against the golden reference.
//
// Records are first compared exactly. If they differ, the complete prefix
// before the final '|' must still match exactly. A final field that can be
// parsed as a floating-point number is compared with the configured
// absolute and relative tolerances.
//
// This accommodates small differences caused by compilers, mathematical
// libraries, floating-point evaluation order, and target platforms. The
// tolerances are intentionally above typical platform-level rounding
// differences while remaining small enough to detect relevant numerical
// changes in the covered scenarios.
//
// Captured console records remain subject to exact textual comparison.
//
// Invocation:
//   ./test_airfoildata [goldenFile] [profileFile]
//
// Defaults:
//   goldenFile  = golden.txt
//   profileFile = NACA4618.dat
//
// Exit code:
//   0 = all records match according to the comparison rules
//   1 = deviations were found

#include "harness_common.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace
{

// Absolute and relative tolerances for finite floating-point values:
//
//     |golden - actual|
//         <= kAbsTolerance
//          + kRelTolerance * max(|golden|, |actual|)
constexpr double kAbsTolerance = 1.0e-12;
constexpr double kRelTolerance = 1.0e-9;


bool parseNumber(const std::string& text, double& value)
{
    if (text.empty())
    {
        return false;
    }

    const char* begin = text.c_str();
    char* end = nullptr;

    value = std::strtod(begin, &end);

    return (end != begin) &&
           (end == begin + text.size());
}


bool lineMatches(const std::string& golden,
                 const std::string& actual)
{
    // Most records, including identical numeric records, terminate here.
    if (golden == actual)
    {
        return true;
    }

    // Captured console output is textual and must match exactly.
    if ((golden.find("|console|") != std::string::npos) ||
        (actual.find("|console|") != std::string::npos))
    {
        return false;
    }

    const std::size_t goldenSeparator = golden.rfind('|');
    const std::size_t actualSeparator = actual.rfind('|');

    if ((std::string::npos == goldenSeparator) ||
        (std::string::npos == actualSeparator))
    {
        return false;
    }

    // The complete record prefix remains subject to exact comparison.
    if (golden.substr(0, goldenSeparator) !=
        actual.substr(0, actualSeparator))
    {
        return false;
    }

    double goldenValue = 0.0;
    double actualValue = 0.0;

    if (!parseNumber(
            golden.substr(goldenSeparator + 1),
            goldenValue) ||
        !parseNumber(
            actual.substr(actualSeparator + 1),
            actualValue))
    {
        return false;
    }

    // NaN values are accepted only if their complete serialized records
    // were identical. That case was handled by the first comparison.
    if (std::isnan(goldenValue) ||
        std::isnan(actualValue))
    {
        return false;
    }

    // Infinite values must agree in classification and sign.
    if (std::isinf(goldenValue) ||
        std::isinf(actualValue))
    {
        return std::isinf(goldenValue) &&
               std::isinf(actualValue) &&
               (std::signbit(goldenValue) ==
                std::signbit(actualValue));
    }

    const double goldenMagnitude = std::fabs(goldenValue);
    const double actualMagnitude = std::fabs(actualValue);

    const double scale =
        (goldenMagnitude > actualMagnitude)
            ? goldenMagnitude
            : actualMagnitude;

    const double difference =
        std::fabs(goldenValue - actualValue);

    const double tolerance =
        kAbsTolerance + kRelTolerance * scale;

    return difference <= tolerance;
}

} // namespace


class CollectSink : public HarnessSink
{
public:
    void emit(const std::string& line) override
    {
        lines_.push_back(line);
    }

    const std::vector<std::string>& lines() const
    {
        return lines_;
    }

private:
    std::vector<std::string> lines_;
};


int main(int argc, char** argv)
{
    const std::string goldenPath =
        (argc > 1) ? argv[1] : "golden.txt";

    const std::string profilePath =
        (argc > 2) ? argv[2] : "NACA4618.dat";

    std::ifstream goldenFile(goldenPath);

    if (false == goldenFile.is_open())
    {
        std::cerr
            << "Cannot read golden file "
            << goldenPath
            << std::endl;

        return 1;
    }

    std::vector<std::string> goldenLines;
    std::string line;

    while (std::getline(goldenFile, line))
    {
        // Accept CRLF checkouts on Windows. Emitted scenario records use
        // '\n' and do not contain the trailing carriage return.
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }

        goldenLines.push_back(line);
    }

    CollectSink sink;
    runAllScenarios(sink, profilePath);

    const std::vector<std::string>& actualLines =
        sink.lines();

    std::size_t mismatches = 0;

    const std::size_t commonLineCount =
        (goldenLines.size() < actualLines.size())
            ? goldenLines.size()
            : actualLines.size();

    for (std::size_t i = 0;
         i < commonLineCount;
         ++i)
    {
        if (!lineMatches(goldenLines[i], actualLines[i]))
        {
            ++mismatches;

            if (mismatches <= 40)
            {
                std::cout
                    << "Line " << (i + 1) << ":\n"
                    << "  golden: " << goldenLines[i] << "\n"
                    << "  actual: " << actualLines[i]
                    << std::endl;
            }
        }
    }

    if (goldenLines.size() != actualLines.size())
    {
        std::cout
            << "Line count differs: golden "
            << goldenLines.size()
            << ", actual "
            << actualLines.size()
            << std::endl;

        mismatches +=
            (goldenLines.size() > actualLines.size())
                ? goldenLines.size() - actualLines.size()
                : actualLines.size() - goldenLines.size();

        const std::size_t greaterLineCount =
            (goldenLines.size() > actualLines.size())
                ? goldenLines.size()
                : actualLines.size();

        for (std::size_t i = commonLineCount;
             (i < greaterLineCount) &&
             (i < commonLineCount + 10);
             ++i)
        {
            if (i < goldenLines.size())
            {
                std::cout
                    << "  golden[" << (i + 1) << "]: "
                    << goldenLines[i]
                    << std::endl;
            }

            if (i < actualLines.size())
            {
                std::cout
                    << "  actual[" << (i + 1) << "]: "
                    << actualLines[i]
                    << std::endl;
            }
        }
    }

    if (0 == mismatches)
    {
        std::cout
            << "PASS: "
            << actualLines.size()
            << " records match the golden reference "
               "(abs tol 1e-12, rel tol 1e-9)."
            << std::endl;

        return 0;
    }

    std::cout
        << "FAIL: "
        << mismatches
        << " deviations."
        << std::endl;

    return 1;
}
