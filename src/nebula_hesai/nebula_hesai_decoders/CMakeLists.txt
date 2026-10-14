cmake_minimum_required(VERSION 3.20)
project(nebula_hesai_decoders)

find_package(autoware_cmake REQUIRED)
find_package(Boost REQUIRED)
autoware_package()

# Option to enable CUDA support
option(BUILD_CUDA "Build with CUDA support" ON)

# CUDA Configuration
if(BUILD_CUDA)
  find_package(CUDAToolkit QUIET)
  if(NOT CUDAToolkit_FOUND)
    message(WARNING "CUDA Toolkit not found, building without CUDA support")
    set(BUILD_CUDA OFF)
  else()
    message(STATUS "CUDA Toolkit found: ${CUDAToolkit_VERSION}")

    # GPU architectures — set via -DCMAKE_CUDA_ARCHITECTURES on the command line,
    # or fall back to a default set covering Turing through Ada + PTX for forward
    # compatibility (e.g. Blackwell GPUs JIT from compute_89 PTX).
    # Example: -DCMAKE_CUDA_ARCHITECTURES=89  (single-arch for fast builds)
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
      set(CMAKE_CUDA_ARCHITECTURES "75;86;87;89" CACHE STRING "CUDA architectures" FORCE)
    endif()

  endif()
endif()

# Hesai Decoder library (CPU)
add_library(nebula_hesai_decoders SHARED src/hesai_driver.cpp)

target_include_directories(
  nebula_hesai_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_hesai_decoders
  PUBLIC nebula_core_common::nebula_core_common
         nebula_hesai_common::nebula_hesai_common
         nebula_core_decoders::nebula_core_decoders rclcpp::rclcpp
         Boost::headers)

# CUDA Decoder library
if(BUILD_CUDA)
  enable_language(CUDA)

  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 -diag-suppress 20012")

  add_library(nebula_hesai_decoders_cuda SHARED
    src/cuda/hesai_cuda_kernels.cu
  )

  set_target_properties(nebula_hesai_decoders_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
  )

  # Clear any inherited compile options that might be incompatible with CUDA
  set_property(TARGET nebula_hesai_decoders_cuda PROPERTY COMPILE_OPTIONS "")

  target_include_directories(
    nebula_hesai_decoders_cuda
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  target_link_libraries(nebula_hesai_decoders_cuda
    PRIVATE CUDA::cudart_static)

  target_compile_definitions(nebula_hesai_decoders_cuda PUBLIC NEBULA_CUDA_ENABLED)
  target_compile_definitions(nebula_hesai_decoders PUBLIC NEBULA_CUDA_ENABLED)

  # Link CUDA library to main library
  target_link_libraries(nebula_hesai_decoders
    PUBLIC nebula_hesai_decoders_cuda
    PRIVATE CUDA::cudart)

  install(TARGETS nebula_hesai_decoders_cuda EXPORT export_nebula_hesai_decoders_cuda)
  ament_export_targets(export_nebula_hesai_decoders_cuda)
endif()

install(TARGETS nebula_hesai_decoders EXPORT export_nebula_hesai_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY calibration DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_hesai_decoders)

ament_export_dependencies(Boost nebula_core_common nebula_hesai_common
                          nebula_core_decoders rclcpp)

ament_package()
