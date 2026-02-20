cmake_minimum_required(VERSION 3.10) # Bumped to 3.10 for better Modern CMake target support
project(collision_test)

# Set C++ Standard globally (Modern FCL and Eigen heavily benefit from C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==========================================================
# Core Physics & Math Dependencies
# ==========================================================
find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED) # Made REQUIRED because the benchmark needs it for threading

add_executable(collision_test main.cpp)

# Modern CMake targets (Eigen3::Eigen, fcl) automatically handle their own include directories.
# We use PRIVATE because this is an executable, not a library being linked elsewhere.
target_link_libraries(collision_test PRIVATE 
    fcl 
    Eigen3::Eigen 
    OpenMP::OpenMP_CXX
)

# ==========================================================
# Optional Visualization Dependencies (OpenGL, GLFW, GLEW, GLM)
# ==========================================================
find_package(OpenGL QUIET)
find_package(PkgConfig QUIET)

# Try to find GLFW3
set(GLFW3_FOUND FALSE)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(GLFW3 glfw3)
endif()
if(NOT GLFW3_FOUND)
    find_package(glfw3 QUIET)
    if(glfw3_FOUND)
        set(GLFW3_FOUND TRUE)
        set(GLFW3_LIBRARIES glfw)
    endif()
endif()

# Try to find GLEW
find_package(GLEW QUIET)
if(NOT GLEW_FOUND AND PKG_CONFIG_FOUND)
    pkg_check_modules(GLEW glew)
endif()

# Try to find GLM
find_package(glm QUIET)

# Check if ALL visualization dependencies are met (Fixed: Added GLM to the check)
if(OPENGL_FOUND AND GLFW3_FOUND AND GLEW_FOUND AND (glm_FOUND OR GLM_FOUND))
    message(STATUS "OpenGL visualization dependencies FOUND. Enabling visualization.")
    target_compile_definitions(collision_test PRIVATE ENABLE_OPENGL)
    
    target_link_libraries(collision_test PRIVATE
        OpenGL::GL
        ${GLFW3_LIBRARIES}
        ${GLEW_LIBRARIES}
    )
    
    if(GLFW3_INCLUDE_DIRS)
        target_include_directories(collision_test PRIVATE ${GLFW3_INCLUDE_DIRS})
    endif()
    
    if(GLEW_INCLUDE_DIRS)
        target_include_directories(collision_test PRIVATE ${GLEW_INCLUDE_DIRS})
    endif()
    
    # Fixed: Actually include GLM if found (GLM is header-only)
    if(GLM_INCLUDE_DIRS)
        target_include_directories(collision_test PRIVATE ${GLM_INCLUDE_DIRS})
    endif()
else()
    message(WARNING "OpenGL visualization disabled - missing dependencies!")
    message(STATUS "  OpenGL: ${OPENGL_FOUND}")
    message(STATUS "  GLFW3:  ${GLFW3_FOUND}")
    message(STATUS "  GLEW:   ${GLEW_FOUND}")
    message(STATUS "  GLM:    ${glm_FOUND}")
    message(STATUS "Running in console-only mode (Benchmarks only)")
endif()