
include_directories(
  ../src
  ../thirdparty/boost/boost_1_65_1
  ../thirdparty/tbb/tbb2017_20170807oss/include
  ../thirdparty/mpir/mpir-2.7.2
  )

link_directories(/home/steve/dev/Cork/thirdparty/tbb/tbb2017_20170807oss/lib/intel64/gcc4.7)

add_executable(CorkCLI RegressionTest.cpp)

target_link_libraries(CorkCLI  Corklib tbb mpir boost_program_options boost_filesystem boost_system boost_timer boost_thread pthread)