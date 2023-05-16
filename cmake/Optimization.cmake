# Aggressive optimization for first-party targets, applied only in optimized build
# types (everything except Debug). It is set per target so the fetched test
# dependency keeps its own defaults.
#
# The navigation filter is built from small dense matrices. At -O3 the compiler
# vectorises those products and the covariance prediction runs about three times
# faster than at -O2, which keeps the code readable (a plain dense Kalman predict)
# while still being fast. Coverage and sanitizer builds are Debug, so they keep
# their own optimization level and are unaffected (coverage in particular stays at
# -O0).
include_guard(GLOBAL)

function(tercom_target_optimization target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE $<$<NOT:$<CONFIG:Debug>>:-O3>)
    elseif(MSVC)
        target_compile_options(${target} PRIVATE $<$<NOT:$<CONFIG:Debug>>:/O2>)
    endif()
endfunction()
