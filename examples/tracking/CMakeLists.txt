include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(tracking src/main.cpp)
target_link_libraries(tracking PRIVATE lib-tercom)
set_property(TARGET tracking PROPERTY FOLDER "tracking")

add_custom_command(
    TARGET tracking POST_BUILD
    COMMAND sha256sum tracking > sha256.txt
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
add_custom_command(
    TARGET tracking POST_BUILD
    COMMAND sha256sum tracking > sha256.txt
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
