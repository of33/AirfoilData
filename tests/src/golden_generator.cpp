// Golden generator: runs the scenario corpus and writes the
// emitted lines into the golden reference file.
//
// Invocation: ./golden_generator [goldenFile] [profileFile]
//             (default: golden.txt, NACA4618.dat)

#include "harness_common.h"

#include <fstream>
#include <iostream>
#include <string>

class FileSink : public HarnessSink
{
public:
    FileSink(const std::string& path)
        : file_(path)
    {
    }

    bool ok() const { return file_.is_open(); }

    void emit(const std::string& line) override
    {
        file_ << line << "\n";
    }

private:
    std::ofstream file_;
};

int main(int argc, char** argv)
{
    const std::string goldenPath = (argc > 1) ? argv[1] : "golden.txt";
    const std::string profilePath = (argc > 2) ? argv[2] : "NACA4618.dat";

    FileSink sink(goldenPath);
    if (false == sink.ok())
    {
        std::cerr << "Cannot write golden file " << goldenPath << std::endl;
        return 1;
    }

    runAllScenarios(sink, profilePath);
    return 0;
}
