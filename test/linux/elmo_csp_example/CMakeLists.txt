
set(SOURCES elmo_csp_example.cpp)
add_executable(elmo_csp_example ${SOURCES})
target_link_libraries(elmo_csp_example soem m)
install(TARGETS elmo_csp_example DESTINATION bin)