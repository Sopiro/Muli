add_library(glad STATIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/glad.c
    ${CMAKE_CURRENT_SOURCE_DIR}/include/glad/glad.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KHR/khrplatform.h
)

target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
