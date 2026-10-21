# Contributing to AirfoilData

Contributions to AirfoilData are welcome.

Please use GitHub issues for:

- bug reports,
- questions,
- feature suggestions,
- portability problems,
- discussion of larger numerical or API changes.

Pull requests are welcome for:

- bug fixes,
- documentation improvements,
- additional regression scenarios,
- numerical robustness improvements,
- portability improvements,
- carefully discussed extensions of the public API.

For larger changes, please open an issue first to discuss the motivation,
numerical implications, and possible impact on the public API.

---

## Project Philosophy

AirfoilData is a practical C++17 engineering library whose design reflects
several decades of professional experience with numerical, aerodynamic, and
engineering software.

Contributions must preserve the following goals:

- simple use and integration,
- a core consisting of only a few source files,
- no mandatory dependencies beyond the C++ standard library,
- readable numerical procedures,
- explicit units, normalizations, and physical meanings,
- defensive checks for dimensions, ranges, NaN, infinity, and invalid state,
- transactional updates where operations can fail,
- reproducible behavior supported by focused regression tests.

Abstraction and modern C++ features are welcome when they clearly improve
safety, correctness, or maintainability. They should not obscure the
engineering procedure or make ordinary use more difficult.

A technically correct implementation may still be unsuitable if it
unnecessarily increases integration effort, hides the numerical method behind
several abstraction layers, or weakens existing validation and state
guarantees.

---

## Requirements

The core library requires:

- a C++17-compatible compiler,
- the C++ standard library.

The complete developer test set may additionally require:

- GCC,
- Clang,
- GNU Make,
- AddressSanitizer,
- UndefinedBehaviorSanitizer,
- the libstdc++ debug mode,
- `gcov`,
- the GCC static analyzer,
- `clang-tidy`,
- Valgrind.

Not every optional tool is required for every contribution, but relevant tests
should be run whenever the required tools are available.

---

## Repository Layout

The principal files are organized as follows:

```text
src/
    AirfoilData.h
    AirfoilData.cpp
    AkimaInterpolation.h
    AkimaInterpolation.cpp

tests/
      Makefile
      data/
           golden.txt
           NACA4618.dat
           NACA2312.dat
      src/
          harness_common.h
          harness_common.cpp
          golden_generator.cpp
          test_airfoildata.cpp

example.cpp
Makefile
plot/
     plot.gnu
```
The core library consists of four files and has no mandatory external
dependencies.

---

## Building the Example

Build the example with:

```bash
make
```

or:

```bash
make example
```

The executable is written to:

```text
bin/example
```

On Windows with MSYS2, the executable is normally:

```text
bin/example.exe
```

---

## Building the Core Object Files

AirfoilData normally links the two core object files directly into an
application. Build them with:

```bash
make objects
```

This creates:

```text
build/AirfoilData.o
build/AkimaInterpolation.o
```

The object files must be built with a toolchain, ABI, target architecture, and
relevant compiler options compatible with the calling application.

On systems providing a compatible `ar` tool, an optional static archive can be
created with:

```bash
make lib
```

This creates:

```text
lib/libAirfoilData.a
```

The archive is a convenience only. It contains the same two object modules used
by the default direct-object linking process.

---

## Running the Standard Test Set

Run the complete standard test set from the repository root with:

```bash
make test
```

This invokes:

```bash
make -C tests check
```

A successful run ends with:

```text
TEST OVERALL: PASSED
```

The complete `check` target may require GCC, Clang, `clang-tidy`, `gcov`, and
sanitizer support. Individual test targets can be used if not all tools are
available.

---

## Individual Test Targets

Run the GCC golden-reference comparison with:

```bash
make -C tests test-gcc
```

Run the Clang comparison with:

```bash
make -C tests test-clang
```

Run AddressSanitizer and UndefinedBehaviorSanitizer with GCC:

```bash
make -C tests san-gcc
```

Run the Clang sanitizer build, including implicit-conversion checks:

```bash
make -C tests san-clang
```

Run the expanded GCC undefined-behavior checks:

```bash
make -C tests san-full-gcc
```

Run the libstdc++ debug and assertion modes:

```bash
make -C tests dbg-gcc
```

Generate coverage information:

```bash
make -C tests coverage
```

Run the GCC static analyzer:

```bash
make -C tests analyzer
```

Run Clang analyzer checks through `clang-tidy`:

```bash
make -C tests tidy
```

Run Valgrind Memcheck:

```bash
make -C tests memcheck
```

Run the extended set of developer checks:

```bash
make -C tests sharp
```

Build the GCC regression test with warnings treated as errors:

```bash
make -C tests strict
```

---

## Test-Harness Architecture

The regression harness uses a common scenario corpus implemented in:

```text
tests/src/harness_common.cpp
```

Both the golden generator and the regression test call:

```cpp
void runAllScenarios(
    HarnessSink& sink,
    const std::string& profilePath);
```

The two programs differ only in how the emitted lines are handled:

- `golden_generator.cpp` writes them to the golden-reference file,
- `test_airfoildata.cpp` compares them with the existing reference.

Floating-point values are serialized with the C `%a` format. This preserves
the exact stored binary `double` value without decimal-formatting ambiguity.

Hexadecimal serialization does not guarantee identical results on every
platform. Compiler optimizations, mathematical-library implementations,
processor behavior, and floating-point evaluation order may still produce
differences. The GCC and Clang comparisons are therefore important portability
checks.

---

## Floating-Point Regression Comparison

Scenario values are serialized with the C `%a` format. This preserves the
exact binary `double` value without decimal-formatting ambiguity.

Finite floating-point values are compared using:

```text
abs(golden - actual)
<= 1e-12
+ 1e-9 * max(abs(golden), abs(actual))
```

The complete record prefix before the final `|` separator must match exactly.
Captured console records, tags, dimensions, integer values, Boolean values,
diagnostic text, record order, and record count are compared exactly.

NaN values are accepted only if the complete serialized records match.
Infinite values must agree in classification and sign.

The tolerance accommodates small differences between compilers,
mathematical libraries, floating-point evaluation orders, and platforms. It
must not be increased merely to make a failing test pass.

When a numerical test fails:

1. inspect the affected scenario and value,
2. determine the absolute and relative difference,
3. compare the result with another supported compiler or platform,
4. determine whether the change is expected,
5. update the golden reference only for an intended behavior change,
6. change the tolerances only with documented numerical justification.

---

## Skipped Optional Checks

Some sanitizer and analysis targets require runtime components that are not
available in every compiler installation.

If a required optional runtime is unavailable, the test Makefile may print a
warning such as:

```text
WARNING: san-gcc NOT RUN - the address/undefined sanitizer runtime
(libasan/libubsan) is not installed for g++.
```

The remaining available checks continue to run.

`TEST OVERALL: PASSED` means that every check actually executed passed. It
does not mean that every optional check was executed.

A skipped sanitizer target provides no sanitizer result and must not be
reported as a successful sanitizer run.

Before a release, sanitizer checks should pass on at least one supported
environment where the required runtimes are available.

---

## Optional MSYS2 Developer Tools

Ordinary use, object-file builds, the optional static archive, and the example
program do not require Clang.

For the Clang compiler and clang-tidy targets in MSYS2 UCRT64, install:

```bash
pacman -S --needed \
mingw-w64-ucrt-x86_64-clang \
mingw-w64-ucrt-x86_64-clang-tools-extra
```

Check the installation with:

```bash
clang++ --version
clang-tidy --version
```

These packages do not necessarily provide every sanitizer runtime used by the
test Makefile. If `compiler-rt`, `libasan`, or `libubsan` is unavailable, the
affected sanitizer target is reported as `NOT RUN`.

---

## Adding a Regression Scenario

A behavioral change, defect correction, or new public method should normally
be accompanied by a scenario in:

```text
tests/src/harness_common.cpp
```

A scenario is usually implemented as a file-local function:

```cpp
static void scenarioNewFeature(HarnessSink& sink)
{
    AirfoilData airfoil;

    std::vector<double> x = {1.0, 0.0, 1.0};
    std::vector<double> y = {-0.01, 0.0, 0.01};

    const bool result =
    airfoil.setAirfoilPoints(x, y);

    emitBool(
        sink,
        "newfeature.setpoints.ok",
        result);

    emitVector(
        sink,
        "newfeature.airfoilX",
        airfoil.getAirfoilX());
}
```

Call the scenario from `runAllScenarios()`:

```cpp
scenarioNewFeature(sink);
```

Use stable and descriptive tags. A recommended pattern is:

```text
<scenario>.<operation>.<result>
```

Examples:

```text
rotate.positive90.ok
rotate.positive90.x
rotate.positive90.y
akima.nonascending.init
aero.scalars.cm0
```

A new scenario should normally include:

- at least one successful call,
- the principal numerical or structural result,
- relevant invalid or boundary inputs,
- confirmation of documented transactional behavior,
- confirmation that outputs remain unchanged when this is part of the API
contract.

Every defect correction should include a scenario that would have detected the
original defect.

---

## Updating the Golden Reference

The committed file:

```text
tests/data/golden.txt
```

defines the expected numerical and textual behavior of the documented
version.

Do not regenerate the golden reference merely to make a failing test pass.
First determine whether a difference is:

1. an unintended regression,
2. an intended numerical correction,
3. an intended API or diagnostic-message change,
4. a platform-specific difference requiring investigation.

Only intended and reviewed changes should be incorporated into the reference.

Generate a new reference intentionally with:

```bash
make -C tests golden-gcc
```

Inspect the complete change:

```bash
git diff -- tests/data/golden.txt
```

Then rerun at least:

```bash
make -C tests test-gcc
make -C tests test-clang
make -C tests san-gcc
make -C tests san-clang
```

The normal regression target must compare against the existing committed
reference. It must not regenerate the reference automatically before the
comparison.

---

## Floating-Point Regression Comparison

Floating-point scenario values are serialized with the C `%a` format. This
preserves the exact stored binary `double` value without decimal formatting
ambiguity.

The regression comparison is intentionally tolerant across compilers and
platforms. Finite values are compared using the configured absolute and
relative tolerances. Structural data, tags, sizes, integer values, Boolean
values, and message text remain subject to exact comparison.

The tolerance exists to accommodate small differences between platforms such
as Linux/GCC and Windows/MSYS2/GCC. It must not be used to hide an unexplained
algorithmic change.

When a numerical test fails:

1. inspect the affected scenario and value,
2. determine the absolute and relative difference,
3. compare the result on another supported compiler or platform,
4. decide whether the change is expected,
5. update the reference only for an intended behavioral change,
6. change the tolerances only when there is a documented numerical reason.

---

## Numerical Changes

Algebraically equivalent floating-point expressions can produce different
binary results because operation order affects rounding.

When changing numerical code:

- determine whether bitwise compatibility is required,
- preserve documented operation order where relevant,
- explain deliberate numerical changes,
- add a focused regression scenario,
- compare GCC and Clang results,
- run relevant sanitizer and debug-library targets,
- inspect the complete golden-reference difference,
- update the mathematical documentation if semantics changed.

Do not simplify or regroup expressions merely to reduce source length when the
existing order is numerically significant.

Changes involving the following areas require particular care:

- interpolation endpoints,
- support-point ordering,
- leading-edge detection,
- finite-difference stencils,
- Fourier coefficients,
- singularity-distribution normalization,
- geometric tolerances,
- transactional object updates.

---

## Public API Changes

When adding or changing a public method, type, enumeration, or constant, update
all relevant project components:

1. public header,
2. implementation,
3. regression scenarios,
4. golden reference, if behavior changed intentionally,
5. User Manual,
6. example program, if the preferred workflow changed,
7. `README.md`,
8. `CHANGELOG.md`,
9. version information, when required by the release policy.

Assign each new public method to one documentation level:

- **Basic API** for the preferred ordinary workflow,
- **Advanced API** for individual numerical processing steps,
- **Integration API** for application adapters and low-level data management.

Public documentation should state, where relevant:

- parameter units,
- coordinate and sign conventions,
- vector-size requirements,
- support-point ordering,
- finite-value requirements,
- output aliasing restrictions,
- behavior on failure,
- transactional state behavior,
- invalidation of derived data,
- exact normalization and physical meaning.

---

## Coding Conventions

AirfoilData uses C++17 and the C++ standard library.

Please follow the style of the surrounding source file:

- use explicit and descriptive variable types in numerical code,
- use `std::size_t` for vector sizes and indices where appropriate,
- use `enum class` instead of undocumented integer flags,
- mark important status-returning methods with `[[nodiscard]]`,
- validate finite numerical input where required,
- avoid exposing partially updated object state,
- calculate results in temporary storage and commit after success where
practical,
- preserve the correspondence between related coordinate vectors,
- explain numerical and aerodynamic reasons in comments,
- prefer straightforward control flow.

Modern C++ features are welcome when they clearly improve safety or
maintainability. They should not make the numerical procedure harder to
understand.

Comments in public headers should describe the API contract. Comments in
implementation files should explain formulas, tolerances, special cases, and
non-obvious implementation decisions.

---

## Documentation and Physical Meaning

If a contribution changes the physical interpretation of a stored quantity,
the documentation must be updated together with the source code.

This is particularly important for:

- angle units,
- normalized coordinate systems,
- scaled source--sink distributions,
- normalized vortex-sheet distributions,
- derivative and curvature terminology,
- fixed aerodynamic reference conditions,
- distinctions between imported and internally calculated values.

The User Manual should explain the API meaning without duplicating complete
theoretical derivations. Detailed derivations should be referenced from the
appropriate companion reports.

---

## Recommended Contribution Workflow

A typical workflow is:

```bash
git checkout -b feature/my-change

make clean
make example
make -C tests test-gcc
make -C tests test-clang
make -C tests san-gcc
make -C tests san-clang
make -C tests coverage
```

If all developer tools are installed, run:

```bash
make test
```

before submitting the pull request.

---

## Contribution Checklist

Before submitting a change, verify:

- [ ] The code compiles as C++17.
- [ ] GCC and Clang warnings have been reviewed.
- [ ] The ordinary Basic API workflow remains simple.
- [ ] No unnecessary mandatory dependency has been introduced.
- [ ] The implementation remains readable to engineering and scientific users.
- [ ] Units, normalizations, signs, and reference conditions are explicit.
- [ ] Input validation has not been weakened.
- [ ] NaN, infinity, vector dimensions, point counts, and ordering are handled
      where relevant.
- [ ] Important status values are checked.
- [ ] Failure paths do not expose unintended partial results.
- [ ] Invalidated derived data are cleared or recomputed.
- [ ] A focused regression scenario covers the change.
- [ ] Relevant invalid and boundary inputs are tested.
- [ ] GCC and Clang regression comparisons pass where available.
- [ ] Linux and Windows results were compared where practical.
- [ ] Floating-point differences satisfy
      `abs(golden - actual) <= 1e-12 + 1e-9 * max(abs(golden), abs(actual))`.
- [ ] Test tolerances were not increased without documented numerical
      justification.
- [ ] NaN, infinity, signed infinity, and near-zero comparison behavior were
      considered.
- [ ] Skipped optional checks were reviewed and reported.
- [ ] Sanitizer checks ran successfully on at least one suitable environment
      for release-critical changes.
- [ ] Relevant static-analysis checks pass.
- [ ] Golden-reference differences have been reviewed.
- [ ] API classification is clear.
- [ ] Public API documentation matches the current header.
- [ ] Example and plot labels remain consistent with the API.
- [ ] `CHANGELOG.md` has been updated where appropriate.
- [ ] License and attribution requirements have been observed.

---

## License and Attribution

By submitting a contribution, you agree that it may be distributed under the
license and additional attribution terms of the AirfoilData project.

Do not submit code or data that cannot legally be distributed under those
terms. Identify third-party sources and their licenses clearly in the pull
request.
