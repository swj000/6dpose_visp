cmake_minimum_required(VERSION 3.10)
project(PoseVisp)

set(CMAKE_CXX_STANDARD 11)

# ViSP 찾기 (VISP_DIR 환경변수 자동 사용)
find_package(VISP REQUIRED)

# OpenCV 찾기 (OpenCV_DIR 환경변수 자동 사용)
find_package(OpenCV REQUIRED)

add_executable(main_exec main.cpp)

# ViSP 및 OpenCV 라이브러리 링크
target_link_libraries(main_exec
    ${VISP_LIBRARIES}
    ${OpenCV_LIBS}
)

# include 디렉토리 자동 추가
target_include_directories(main_exec PRIVATE
    ${VISP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)