# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# (Not part of the boilerplate)
# This example uses an extra component for common functions such as Wi-Fi and Ethernet connection.
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

list(APPEND EXTRA_COMPONENT_DIRS components/esp32_AS7262)
list(APPEND EXTRA_COMPONENT_DIRS components/esp32_SGP30)
list(APPEND EXTRA_COMPONENT_DIRS components/BME280)
list(APPEND EXTRA_COMPONENT_DIRS components/Microfone)
list(APPEND EXTRA_COMPONENT_DIRS components/ble-mesh-model)
list(APPEND EXTRA_COMPONENT_DIRS components/tarablessd1306)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mqtt_tcp)