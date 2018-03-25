add_library(aig_fmt INTERFACE)
target_include_directories(aig_fmt INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/fmt)
target_compile_definitions(aig_fmt INTERFACE FMT_HEADER_ONLY)

add_library(aig_cli11 INTERFACE)
target_include_directories(aig_cli11 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/cli11)
