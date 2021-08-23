
include_directories(
  ../src
  )



add_executable(CorkCLI RegressionTest.cpp)

target_link_libraries(CorkCLI  Corklib tbb mpir boost_program_options boost_filesystem boost_system boost_timer boost_thread pthread)