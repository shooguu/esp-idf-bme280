idf_component_register(SRCS "spi.cpp" "bme280.cpp"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++14)
