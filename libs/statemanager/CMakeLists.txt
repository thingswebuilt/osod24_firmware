add_library(statemanager STATIC
        src/statemanager.cpp
)

target_link_libraries(statemanager PUBLIC
        receiver
        state_estimator
        stoker
        mixer
        servo
        common
)

target_include_directories(statemanager PUBLIC
        include
        servo
        receiver
        state_estimator
        stoker
        mixer
        common
)