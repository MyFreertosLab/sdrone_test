set(motors_srcs "src/motors.c")

idf_component_register(SRCS "${motors_srcs}"
                       INCLUDE_DIRS "include")

