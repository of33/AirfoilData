// Test program: runs the same scenario corpus as the golden generator
// and compares every emitted line against the golden reference.
//
// Invocation: ./test_airfoildata [goldenFile] [profileFile]
//             (default: golden.txt, NACA4618.dat)
// Exit code 0 = all lines match, 1 = deviations.

#include "harness_common.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class CollectSink : public HarnessSink
{
public:
    void emit(const std::string& line) override
    {
        lines_.push_back(line);
    }

    const std::vector<std::string>& lines() const { return lines_; }

private:
    std::vector<std::string> lines_;
};

int main(int argc, char** argv)
{
    const std::string goldenPath = (argc > 1) ? argv[1] : "golden.txt";
    const std::string profilePath = (argc > 2) ? argv[2] : "NACA4618.dat";

    std::ifstream goldenFile(goldenPath);
    if (false == goldenFile.is_open())
    {
        std::cerr << "Cannot read golden file " << goldenPath << std::endl;
        return 1;
    }
    std::vector<std::string> goldenLines;
    std::string line;
    while (std::getline(goldenFile, line))
        goldenLines.push_back(line);

    CollectSink sink;
    runAllScenarios(sink, profilePath);
    const std::vector<std::string>& actualLines = sink.lines();

    std::size_t mismatches = 0;
    const std::size_t common = (goldenLines.size() < actualLines.size())
                                   ? goldenLines.size() : actualLines.size();
    for (std::size_t i = 0; i < common; i++)
    {
        if (goldenLines[i] != actualLines[i])
        {
            mismatches++;
            if (mismatches <= 40)
            {
                std::cout << "Line " << (i + 1) << ":\n"
                          << "  golden: " << goldenLines[i] << "\n"
                          << "  actual: " << actualLines[i] << std::endl;
            }
        }
    }
    if (goldenLines.size() != actualLines.size())
    {
        std::cout << "Line count differs: golden " << goldenLines.size()
                  << ", actual " << actualLines.size() << std::endl;
        mismatches += (goldenLines.size() > actualLines.size())
                          ? goldenLines.size() - actualLines.size()
                          : actualLines.size() - goldenLines.size();
        const std::size_t show = (goldenLines.size() > actualLines.size())
                                     ? goldenLines.size() : actualLines.size();
        for (std::size_t i = common; i < show && i < common + 10; i++)
        {
            if (i < goldenLines.size())
                std::cout << "  golden[" << (i + 1) << "]: " << goldenLines[i] << std::endl;
            if (i < actualLines.size())
                std::cout << "  actual[" << (i + 1) << "]: " << actualLines[i] << std::endl;
        }
    }

    if (0 == mismatches)
    {
        std::cout << "PASS: " << actualLines.size() << " lines match the golden reference." << std::endl;
        return 0;
    }
    std::cout << "FAIL: " << mismatches << " deviations." << std::endl;
    return 1;
}
