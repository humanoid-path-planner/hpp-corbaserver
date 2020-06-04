ADD_PYTHON_UNIT_TEST(py-robot tests/robot.py src)
ADD_PYTHON_UNIT_TEST(py-quat tests/quat.py src)
ADD_PYTHON_UNIT_TEST(py-utils tests/utils.py src)

# robot & utils start a hppcorbaserver in a separate process.
# They shouldn't be launched at the same time.
SET_TESTS_PROPERTIES("py-robot" PROPERTIES RUN_SERIAL "ON")
SET_TESTS_PROPERTIES("py-utils" PROPERTIES RUN_SERIAL "ON")
