# Per-target gcov/llvm-cov instrumentation, gated behind TERCOM_ENABLE_COVERAGE.
include_guard(GLOBAL)

function(tercom_target_coverage target)
    if(NOT TERCOM_ENABLE_COVERAGE)
        return()
    endif()

    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(WARNING "Coverage instrumentation only supported for GCC/Clang; skipping ${target}.")
        return()
    endif()

    target_compile_options(${target} PRIVATE --coverage -O0 -g)
    target_link_options(${target} PRIVATE --coverage)
endfunction()
