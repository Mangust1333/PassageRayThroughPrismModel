cmake_minimum_required(VERSION 3.14)
project(RayTracingApp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

# ───────────────────────────────────────────────────────────────────────────────
# 1) nlohmann/json (header-only)
# ───────────────────────────────────────────────────────────────────────────────
FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.12.0
)
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(nlohmann_json)

# ───────────────────────────────────────────────────────────────────────────────
# 2) Boost.DI (header-only DI из репозитория boost-ext/di)
# ───────────────────────────────────────────────────────────────────────────────
FetchContent_Declare(
        boost_di
        GIT_REPOSITORY https://github.com/boost-ext/di.git
        GIT_TAG v1.3.2
)
FetchContent_MakeAvailable(boost_di)

# ───────────────────────────────────────────────────────────────────────────────
# 3) SFML (2.5.1) через FetchContent, отключаем примеры, тесты, документацию
# ───────────────────────────────────────────────────────────────────────────────
FetchContent_Declare(
        SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 3.0.1
)
set(SFML_BUILD_EXAMPLES OFF CACHE INTERNAL "")
set(SFML_BUILD_TESTS OFF    CACHE INTERNAL "")
set(SFML_BUILD_DOC OFF      CACHE INTERNAL "")
set(SFML_BUILD_SHARED_LIBS ON CACHE INTERNAL "")
FetchContent_MakeAvailable(SFML)

# ───────────────────────────────────────────────────────────────────────────────
# 4) Пути для include
# ───────────────────────────────────────────────────────────────────────────────
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${boost_di_SOURCE_DIR}/include
        ${nlohmann_json_SOURCE_DIR}/include
)

# ───────────────────────────────────────────────────────────────────────────────
# 5) Сборка исходников проекта (src/core + src/adapters)
# ───────────────────────────────────────────────────────────────────────────────
file(GLOB_RECURSE CORE_SRC    "${CMAKE_SOURCE_DIR}/src/core/*.cpp")
file(GLOB_RECURSE ADAPTERS_SRC "${CMAKE_SOURCE_DIR}/src/adapters/*.cpp")

add_executable(RayTracingApp
        src/main.cpp
        ${CORE_SRC}
        ${ADAPTERS_SRC}
)

# ───────────────────────────────────────────────────────────────────────────────
# 6) Линковка библиотек: SFML (graphics, window, system) + nlohmann/json + boost.di
# ───────────────────────────────────────────────────────────────────────────────
target_link_libraries(RayTracingApp
        PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
        nlohmann_json::nlohmann_json
        # Boost::di
)

# ───────────────────────────────────────────────────────────────────────────────
# 7) Передача пути к config.json через макрос
# ───────────────────────────────────────────────────────────────────────────────
target_compile_definitions(RayTracingApp PRIVATE
    CONFIG_JSON_PATH="${CMAKE_SOURCE_DIR}/config.json"
)
