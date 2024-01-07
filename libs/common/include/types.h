//
// Created by robbe on 31/12/2023.
//

#ifndef OSOD_MOTOR_2040_TYPES_H
#define OSOD_MOTOR_2040_TYPES_H

namespace COMMON {
    namespace MOTOR_POSITION {
        enum MotorPosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT,
            MOTOR_POSITION_COUNT // always keep this last in the enum so that we can use it to get the number of elements
        };
    }
    struct MotorSpeeds {
        float speeds[MOTOR_POSITION::MOTOR_POSITION_COUNT];

        float& operator[](const MOTOR_POSITION::MotorPosition position) {
            return speeds[position];
        }

        const float& operator[](const MOTOR_POSITION::MotorPosition position) const {
            return speeds[position];
        }
    };

    struct SteeringAngles {
        float left;
        float right;
    };

    struct DriveTrainState {
        MotorSpeeds speeds;
        SteeringAngles angles;
    };

    struct Odometry {
        float x;
        float y;
        float heading;
    };

    struct Velocity {
        float x_dot;
        float y_dot;
        float velocity;
        float angular_velocity;
    };
}

#endif //OSOD_MOTOR_2040_TYPES_H
