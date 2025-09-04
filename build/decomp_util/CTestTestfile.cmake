# CMake generated Testfile for 
# Source directory: /home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil
# Build directory: /home/zhicheng/Project/FRGraph/build/decomp_util
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_seed_decomp "test_seed_decomp")
set_tests_properties(test_seed_decomp PROPERTIES  _BACKTRACE_TRIPLES "/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/CMakeLists.txt;19;add_test;/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/CMakeLists.txt;0;")
add_test(test_line_segment "test_line_segment" "/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/data/obstacles.txt")
set_tests_properties(test_line_segment PROPERTIES  _BACKTRACE_TRIPLES "/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/CMakeLists.txt;23;add_test;/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/CMakeLists.txt;0;")
add_test(test_ellipsoid_decomp "test_ellipsoid_decomp" "/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/data/obstacles.txt")
set_tests_properties(test_ellipsoid_decomp PROPERTIES  _BACKTRACE_TRIPLES "/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/CMakeLists.txt;27;add_test;/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/CMakeLists.txt;0;")
add_test(test_iterative_decomp "test_iterative_decomp" "/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/data/obstacles.txt")
set_tests_properties(test_iterative_decomp PROPERTIES  _BACKTRACE_TRIPLES "/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/CMakeLists.txt;31;add_test;/home/zhicheng/Project/FRGraph/src/third_party/DecompROS/DecompUtil/CMakeLists.txt;0;")
