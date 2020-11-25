
set(SOURCES torque_examples_2drivers.c)
add_executable(torque_examples_2drivers ${SOURCES})
target_link_libraries(torque_examples_2drivers soem m)
install(TARGETS torque_examples_2drivers DESTINATION bin)
