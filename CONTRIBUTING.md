# Contributing

Thanks for your interest in the project. This document covers the local
development loop.

## Prerequisites

- CMake >= 3.24
- A C++20 compiler (GCC >= 11 or Clang >= 14)
- Ninja
- Optional, for the quality gates: `clang-format`, `clang-tidy`, `cppcheck`,
  `gcovr`, `prettier` (via Node/`npx`, for non-code files), and `actionlint`
  (on `PATH`, for the workflows)

GoogleTest is fetched automatically at configure time; no system install is
required.

## Build and test

The project uses CMake presets. The thin `Makefile` wraps the common ones:

```bash
make configure        # cmake --preset dev
make build            # cmake --build build/dev
make test             # ctest --preset dev
make simulate         # build and run the software-in-the-loop simulator
```

Or drive CMake directly:

```bash
cmake --preset dev
cmake --build build/dev
ctest --preset dev --output-on-failure
```

## Quality gates

Run the same checks the pipeline runs before opening a change:

```bash
make format           # clang-format, in place
make lint             # clang-format --dry-run + clang-tidy + cppcheck + prettier + actionlint
make coverage         # build with coverage, run tests, emit an HTML report
```

Sanitizers:

```bash
cmake --preset asan && cmake --build build/asan && ctest --preset asan
```

## Style

- Four-space indentation, no tabs, LF line endings (enforced by
  `.editorconfig` and `.clang-format`).
- ASCII only in source and documentation.
- Types and methods are `PascalCase`; private members carry a trailing
  underscore.
- The numeric warning set (`-Wconversion`, `-Wsign-conversion`, ...) is on by
  default. Make narrowing conversions explicit rather than silencing the
  warning.
- Every new module ships with a matching test in `tests/`.

## Commits

- Small, focused commits with imperative subject lines.
- A commit must build and pass `ctest` on its own.
