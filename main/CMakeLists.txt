idf_component_register(SRCS main.cpp PID_v1_0_0.cpp xv_lib.cpp esp32_serial_transport.c 
                    INCLUDE_DIRS ""
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)