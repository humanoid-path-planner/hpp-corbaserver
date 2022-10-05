add_python_unit_test(py-robot tests/robot.py src)
add_python_unit_test(py-quat tests/quat.py src)
add_python_unit_test(py-utils tests/utils.py src)

# robot & utils start a hppcorbaserver in a separate process. They shouldn't be
# launched at the same time.
set_tests_properties("py-robot" PROPERTIES RUN_SERIAL "ON")
set_tests_properties("py-utils" PROPERTIES RUN_SERIAL "ON")
