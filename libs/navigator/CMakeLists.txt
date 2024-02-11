add_library(navigator STATIC
        src/navigator.cpp
)
target_link_libraries(navigator PUBLIC
        config
        common
        pico_cppm
        receiver
        statemanager
        config
        communicator
        waypoint_navigation
)
target_include_directories(navigator PUBLIC
        include
)