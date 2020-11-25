
set(SOURCES elmo_cst_example.cpp)
add_executable(elmo_cst_example ${SOURCES})
target_link_libraries(elmo_cst_example soem m)
install(TARGETS elmo_cst_example DESTINATION bin)