# marv_gcs: the ground-control backend. HTTP (/api/schema, /api/link, the gcs/web build) and WebSocket /ws on
# 127.0.0.1, one io_context thread owning the link to the flight controller (UDP to the bridge, or serial).
find_package(Threads REQUIRED)

add_library(marv_gcs_core STATIC src/json.cpp src/schema.cpp src/mission.cpp src/link.cpp src/server.cpp
            ${PROJECT_SOURCE_DIR}/ground/src/transport.cpp)
target_include_directories(marv_gcs_core PUBLIC src ${PROJECT_SOURCE_DIR}/ground/src)
target_compile_definitions(marv_gcs_core PUBLIC MARV_PARAMS_TEXT
                           MARV_GCS_WEB_DIR="${PROJECT_SOURCE_DIR}/gcs/web/build"
                           MARV_GCS_REPO_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(marv_gcs_core PUBLIC marv_link Boost::headers Threads::Threads)

add_executable(marv_gcs src/main.cpp)
target_link_libraries(marv_gcs PRIVATE marv_gcs_core)

foreach(name test_gcs_schema test_gcs_link test_gcs_mission)
    add_executable(${name} src/${name}.cpp)
    target_link_libraries(${name} PRIVATE marv_gcs_core)
    add_test(NAME ${name} COMMAND ${name})
endforeach()
