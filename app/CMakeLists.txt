add_library(APP INTERFACE)

target_include_directories(APP INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(APP INTERFACE   
    pico_stdlib    
)