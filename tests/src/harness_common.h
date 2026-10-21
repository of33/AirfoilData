// Common scenario basis for the golden generator and the test program.
// Both programs run exactly the same scenarios (runAllScenarios);
// they differ only in how the emitted lines are handled
// (writing into the golden reference vs. comparing against it).
//
// Serialization: doubles are printed with printf("%a").
// This preserves the exact binary double value without decimal
// formatting ambiguity. Numerical results may nevertheless differ
// between compilers, platforms, libraries, or floating-point settings.

#ifndef HARNESS_COMMON_H
#define HARNESS_COMMON_H

#include <sstream>
#include <streambuf>
#include <string>
#include <vector>

class HarnessSink
{
public:
    virtual ~HarnessSink() {}
    virtual void emit(const std::string& line) = 0;
};

// ---- deterministic serialization ----
std::string harnessDouble(double value);
std::string harnessBool(bool value);

void emitDouble(HarnessSink& sink, const std::string& tag, double value);
void emitInt(HarnessSink& sink, const std::string& tag, int value);
void emitBool(HarnessSink& sink, const std::string& tag, bool value);
void emitString(HarnessSink& sink, const std::string& tag, const std::string& value);
// Vectors: size always; for more than 12 elements only the first/last 5.
void emitVector(HarnessSink& sink, const std::string& tag, const std::vector<double>& v);
void emitVectorSize(HarnessSink& sink, const std::string& tag, const std::vector<double>& v);
void emitVector2d(HarnessSink& sink, const std::string& tag, const std::vector<std::vector<double>>& m);
void emitVectorBool(HarnessSink& sink, const std::string& tag, const std::vector<bool>& v);

// RAII: redirects the core's cout and cerr (emitOutput fallback) into a buffer.
class ConsoleCapture
{
public:
    ConsoleCapture();
    ~ConsoleCapture();
    std::string text() const;

private:
    std::ostringstream buffer_;
    std::streambuf* oldCout_;
    std::streambuf* oldCerr_;
};

// Splits the core text captured during recording into lines.
void emitConsole(HarnessSink& sink, const std::string& tag, const std::string& captured);

// Reads a .dat profile (comment lines '#', then x y [yt]).
bool loadProfile(const std::string& path, std::vector<double>& x, std::vector<double>& y);

// The scenario corpus: all public methods + edge cases.
void runAllScenarios(HarnessSink& sink, const std::string& profilePath);

#endif // HARNESS_COMMON_H
