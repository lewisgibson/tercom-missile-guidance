# Strict warning flags for first-party targets.
# Navigation maths must not rely on implicit conversions, so the conversion
# warnings are deliberately enabled. Third-party code (fetched dependencies)
# is never passed through here.
include_guard(GLOBAL)

function(tercom_target_warnings target)
    set(gcc_clang_warnings
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wconversion
        -Wsign-conversion
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wnon-virtual-dtor
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
        -Wimplicit-fallthrough
    )
    set(msvc_warnings /W4 /permissive-)

    if(TERCOM_WARNINGS_AS_ERRORS)
        list(APPEND gcc_clang_warnings -Werror)
        list(APPEND msvc_warnings /WX)
    endif()

    if(MSVC)
        target_compile_options(${target} PRIVATE ${msvc_warnings})
    else()
        target_compile_options(${target} PRIVATE ${gcc_clang_warnings})
    endif()
endfunction()
