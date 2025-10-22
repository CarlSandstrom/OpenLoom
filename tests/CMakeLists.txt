# CMakeLists.txt for tests
enable_testing()



add_executable(runTests test_main.cpp)

target_link_libraries(runTests PRIVATE GTest::gtest GTest::gtest_main pthread)

add_test(NAME AllTests COMMAND runTests)
