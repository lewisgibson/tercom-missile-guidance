# Wires clang-tidy and cppcheck through CMake's built-in per-language hooks so
# they run as part of compilation. Enabling either is opt-in and fails loudly
# if the tool is missing -- a requested check must never silently no-op.
include_guard(GLOBAL)

function(tercom_enable_static_analysis)
    if(TERCOM_ENABLE_CLANG_TIDY)
        # Prefer the versioned binary the project is developed against, falling back
        # to an unversioned clang-tidy.
        find_program(CLANG_TIDY_EXE NAMES clang-tidy-18 clang-tidy)
        if(NOT CLANG_TIDY_EXE)
            message(FATAL_ERROR "TERCOM_ENABLE_CLANG_TIDY is ON but clang-tidy was not found on PATH")
        endif()
        set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE};--warnings-as-errors=*" PARENT_SCOPE)
        message(STATUS "clang-tidy enabled: ${CLANG_TIDY_EXE}")
    endif()

    if(TERCOM_ENABLE_CPPCHECK)
        find_program(CPPCHECK_EXE NAMES cppcheck)
        if(NOT CPPCHECK_EXE)
            message(FATAL_ERROR "TERCOM_ENABLE_CPPCHECK is ON but cppcheck was not found on PATH")
        endif()
        # useStlAlgorithm is a style preference, not a defect: the numerics keep
        # deliberate explicit loops for readability. The include suppressions silence
        # cppcheck's inability to see the full include graph in unity-free builds.
        set(CMAKE_CXX_CPPCHECK
            "${CPPCHECK_EXE};--enable=warning,style,performance,portability;--inline-suppr;--std=c++20;--language=c++;--suppress=missingInclude;--suppress=missingIncludeSystem;--suppress=useStlAlgorithm;--error-exitcode=1"
            PARENT_SCOPE)
        message(STATUS "cppcheck enabled: ${CPPCHECK_EXE}")
    endif()
endfunction()
