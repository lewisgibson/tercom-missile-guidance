# Thin wrapper around the CMake presets for the common development tasks.
# The presets in CMakePresets.json are the source of truth; this file just gives
# them short, memorable names. Override a tool name on the command line if your
# distribution installs it unversioned, for example:
#   make format-check CLANG_FORMAT=clang-format
# (clang-tidy is located by CMake's find_program, preferring clang-tidy-18.)

CLANG_FORMAT ?= clang-format-18
GCOVR        ?= gcovr
# prettier (needs Node/npx) checks non-code files; actionlint checks the workflows.
# Both must be on PATH for the full `lint` gate; override if installed differently.
PRETTIER     ?= npx --yes prettier@3.4.2
ACTIONLINT   ?= actionlint

# All first-party C++ sources and headers (never the fetched dependencies).
SOURCES := $(shell find include src examples tools tests benchmarks scenarios \( -name '*.h' -o -name '*.cpp' \) 2>/dev/null)

.PHONY: all configure build test simulate benchmark format format-check tidy cppcheck prettier actionlint lint coverage clean help

all: build

help:
	@echo "Targets:"
	@echo "  configure     Configure the Debug developer build (preset: dev)"
	@echo "  build         Build the developer build"
	@echo "  test          Run the unit tests with ctest"
	@echo "  simulate      Build and run the software-in-the-loop simulation example"
	@echo "  benchmark     Build (optimized) and run the performance benchmarks"
	@echo "  format        Reformat all sources in place with clang-format"
	@echo "  lint          format-check + clang-tidy + cppcheck + prettier + actionlint (the strict quality gates)"
	@echo "  coverage      Build instrumented, run tests, and emit an HTML coverage report"
	@echo "  clean         Remove the build tree"

configure:
	cmake --preset dev

build: configure
	cmake --build build/dev

test: build
	ctest --preset dev

simulate: build
	./build/dev/bin/sitl-simulation

benchmark:
	cmake --preset release -DTERCOM_BUILD_BENCHMARKS=ON
	cmake --build build/release --target tercom-benchmark
	./build/release/bin/tercom-benchmark

format:
	@if [ -z "$(strip $(SOURCES))" ]; then echo "No source files found -- the find directory list is stale" >&2; exit 1; fi
	@echo "Formatting $(words $(SOURCES)) files"
	$(CLANG_FORMAT) -i $(SOURCES)

format-check:
	@if [ -z "$(strip $(SOURCES))" ]; then echo "No source files found -- the find directory list is stale" >&2; exit 1; fi
	$(CLANG_FORMAT) --dry-run --Werror $(SOURCES)

tidy:
	cmake --preset tidy -DTERCOM_BUILD_BENCHMARKS=ON
	cmake --build build/tidy

cppcheck:
	cmake --preset cppcheck -DTERCOM_BUILD_BENCHMARKS=ON
	cmake --build build/cppcheck

prettier:
	$(PRETTIER) --check '**/*.{yml,yaml,json,md}'

actionlint:
	$(ACTIONLINT) -no-color

lint: format-check tidy cppcheck prettier actionlint

coverage:
	cmake --preset coverage
	cmake --build build/coverage
	ctest --test-dir build/coverage --output-on-failure
	mkdir -p coverage
	$(GCOVR) --root . --filter 'src/' --filter 'include/' \
		--html-details coverage/index.html --print-summary \
		--fail-under-line 90 build/coverage
	@echo "Coverage report: coverage/index.html"

clean:
	rm -rf build coverage
