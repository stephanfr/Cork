
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.0.0-preview3)

FetchContent_MakeAvailable(Catch2)

include_directories(
    ../include
  )

add_executable( tests
  CachingFactoryTest.cpp
  ConstructOnceResizeableVectorTest.cpp
  Vector3DTest.cpp
)

target_link_libraries(tests PRIVATE Catch2::Catch2WithMain Corklib tbb mpir boost_program_options boost_filesystem boost_system boost_timer boost_thread pthread)

add_executable( benchmarks
#  UnionFindBenchmark.cpp
  Vector3DBenchmark.cpp
)

target_link_libraries(benchmarks PRIVATE Catch2::Catch2WithMain Corklib tbb mpir boost_program_options boost_filesystem boost_system boost_timer boost_thread pthread)


list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)

include(CTest)
include(Catch)

catch_discover_tests( tests )
catch_discover_tests( benchmarks )