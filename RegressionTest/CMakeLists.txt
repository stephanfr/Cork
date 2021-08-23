
FetchContent_Declare(
  Xoshiro256PlusSIMD
  GIT_REPOSITORY https://github.com/stephanfr/Xoshiro256PlusSIMD.git
  GIT_TAG        main)

FetchContent_MakeAvailable(Xoshiro256PlusSIMD)

include_directories(
  ../src
  ${xoshiro256plussimd_SOURCE_DIR}/include
  )

add_executable(CorkCLI RegressionTest.cpp)

target_link_libraries(CorkCLI  Corklib boost_program_options boost_filesystem boost_system boost_timer boost_thread pthread)