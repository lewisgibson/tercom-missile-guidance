# AddressSanitizer + UndefinedBehaviorSanitizer, gated behind TERCOM_ENABLE_SANITIZERS.
# Applied per target so fetched dependencies stay uninstrumented.
include_guard(GLOBAL)

function(tercom_target_sanitizers target)
    if(NOT TERCOM_ENABLE_SANITIZERS)
        return()
    endif()

    if(MSVC)
        message(WARNING "TERCOM_ENABLE_SANITIZERS has no effect with MSVC; skipping ${target}.")
        return()
    endif()

    set(flags -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
    target_compile_options(${target} PRIVATE ${flags})
    target_link_options(${target} PRIVATE ${flags})
endfunction()
