// drivetrain_config.h
#ifndef DRIVETRAIN_CONFIG_H
#define DRIVETRAIN_CONFIG_H
#include <cmath>
#include <cstdint>  

namespace CONFIG {
    #define LAVA_PALAVA 0
    #define ECO_DISASTER 1
    #define ESCAPE_ROUTE 2
    #define MINESWEEPER 3
    #define ZOMBIE_APOCALYPSE 4
    #define PI_NOON 5
    #define TEMPLE_OF_DOOM 6

    #define CURRENT_CHALLENGE ESCAPE_ROUTE

    constexpr int I2C_SDA_PIN = 20;
    constexpr int I2C_SCL_PIN = 21;
    constexpr uint8_t BNO08X_ADDR = 0x4A;
    
    enum Handedness {
        LEFT,
        RIGHT
    };

    // chassis geometry
    constexpr float WHEEL_BASE = 0.18f; // metres
    constexpr float WHEEL_TRACK = 0.15f; // metres
    constexpr float HALF_WHEEL_TRACK = WHEEL_TRACK / 2;

    //steering
    constexpr float MAX_STEERING_ANGLE = 3.14 / 4; // radians
    const float STEERING_HYPOTENUSE = std::sqrt(HALF_WHEEL_TRACK * HALF_WHEEL_TRACK + WHEEL_BASE * WHEEL_BASE);

    //left steering servo
    static constexpr float LEFT_MIN_PULSE = 1032.0f; // usec
    static constexpr float LEFT_MID_PULSE = 1599.0f; // usec
    static constexpr float LEFT_MAX_PULSE = 2200.0f; // usec
    static constexpr float LEFT_MID_VALUE = 0.0f; // radians @LEFT_MID_PULSE
    static constexpr float LEFT_MIN_VALUE = 3.14 / 4; // radians  @LEFT_MIN_PULSE (pi/4radians = 45degrees)

    //right steering servo
    static constexpr float RIGHT_MIN_PULSE = 1221.0f; // usec
    static constexpr float RIGHT_MID_PULSE = 1670.0f; // usec
    static constexpr float RIGHT_MAX_PULSE = 2200.0f; // usec
    static constexpr float RIGHT_MID_VALUE = 0.0f; // radians @RIGHT_MID_PULSE
    static constexpr float RIGHT_MIN_VALUE = 3.14 / 4; // radians  @RIGHT_MIN_PULSE (pi/4radians = 45degrees)


    //wheels and gearing
    constexpr float LARGE_WHEEL_DIAMETER = 0.0816f; // metres valid for Lego 2902 "81.6" tyres
    constexpr float SMALL_WHEEL_DIAMETER = 0.052f; // metres valid for Lego 15413 tyres
    constexpr float MECANUM_DIAMETER = 0.048f; // metres, valid for "48mm" mecanums
    constexpr float GEARMOTOR_RATIO = 19.22f; // -to-1


    // Define WHEEL_DIAMETER and GEAR_RATIO based on CURRENT_CHALLENGE
    #if (CURRENT_CHALLENGE == ECO_DISASTER)
        constexpr float WHEEL_DIAMETER = LARGE_WHEEL_DIAMETER;
        constexpr float GEAR_RATIO = 42.0 / 18.0 * GEARMOTOR_RATIO;
    #elif (CURRENT_CHALLENGE == ESCAPE_ROUTE || CURRENT_CHALLENGE == MINESWEEPER || CURRENT_CHALLENGE == ZOMBIE_APOCALYPSE)
        const float WHEEL_DIAMETER = SMALL_WHEEL_DIAMETER;
        const float GEAR_RATIO = GEARMOTOR_RATIO;
    #elif (CURRENT_CHALLENGE == PI_NOON)
        const float WHEEL_DIAMETER = MECANUM_DIAMETER;
        const float GEAR_RATIO = GEARMOTOR_RATIO;
    #elif  (CURRENT_CHALLENGE == LAVA_PALAVA || CURRENT_CHALLENGE == TEMPLE_OF_DOOM)
        const float WHEEL_DIAMETER = LARGE_WHEEL_DIAMETER;
        const float GEAR_RATIO = GEARMOTOR_RATIO;
    #else
        // Default case
        const float WHEEL_DIAMETER = SMALL_WHEEL_DIAMETER;
        const float GEAR_RATIO = GEARMOTOR_RATIO;
    #endif


    // motor properties
    // The counts per rev of the motor
    constexpr int CPR = 3;
    // The counts per revolution of the wheel
    // note that this is not constexpr because it depends on the gear ratio which is not constexpr
    // because it depends on the value of CURRENT_CHALLENGE which is not evaluated until the preprocessing stage
    const float COUNTS_PER_REV = CPR * GEAR_RATIO;
    // The scaling to apply to the motor's speed to match its real-world speed
    constexpr float SPEED_SCALE = 495.0f;

    //dynamics
    constexpr float MAX_VELOCITY = 1.28; // m/s
    constexpr float MAX_ACCELERATION = 8; // m/s^2
    constexpr float MAX_ANGULAR_VELOCITY = 17; // rad/s
    constexpr float MAX_ANGULAR_ACCELERATION = 20; // rad/s^2


    // Control constants such as PID & feedforward
    // PID values
    constexpr float VEL_KP = 3.25f; // Velocity proportional (P) gain
    constexpr float VEL_KI = 0.0f; // Velocity integral (I) gain
    constexpr float VEL_KD = 0.003f; // Velocity derivative (D) gain

// feedforward values
    constexpr float VEL_FF_GAIN = 1.0f;   // Velocity feedforward gain
    constexpr float ACC_FF_GAIN = 0.0f;    // Acceleration feedforward gain


}
#endif // DRIVETRAIN_CONFIG_H