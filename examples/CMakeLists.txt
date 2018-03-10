# No debugging and optimization for examples
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")

function(add_example name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} aig aig_fmt aig_cli11 aig_lorina)
endfunction()

add_example(main main.cpp)
