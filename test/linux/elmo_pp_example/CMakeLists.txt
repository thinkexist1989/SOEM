
set(SOURCES elmo_pp_example.cpp)
add_executable(elmo_pp_example ${SOURCES})
target_link_libraries(elmo_pp_example soem m)
install(TARGETS elmo_pp_example DESTINATION bin)