//
// Created by markm on 11/11/2023.
//
// code for a four wheeled, independent four wheel drive, two wheel steering platform

#include "ackermann_strategy.h"
#include "mixer_strategy.h"
#include <cmath>
#include <limits>
#include <cstdio>
#include <functional>
#include "types.h"

/**
 * @brief Returns the sign of a value
 * @tparam T The type of the value
 * @param value The value to check
 * @return 1 if the value is positive, -1 if the value is negative
 */
template<typename T>
int sign(T value) {
    return std::signbit(value) ? -1 : 1;
}

namespace MIXER {
    AckermannMixer::AckermannMixer(float track, float base, float angle) : wheelTrack(track), wheelBase(base),
                                                                           maxSteeringAngle(angle) {
        // initialise the turn radius to zero
        turnRadius = 0.0;
    }

    DriveTrainState AckermannMixer::mix(float velocity, float angularVelocity) {
        // function takes desired forward speed ("throttle") in m/s and turn rate in radians/sec
        // and outputs individual wheel speeds in m/s and turn angle of steerable wheels in radians

        if (std::fabs(angularVelocity) > 0) {
            turnRadius = velocity / angularVelocity;
        } else {
            // Handle the case when yaw rate is close to zero to avoid division by zero
            turnRadius = std::numeric_limits<float>::infinity();
        }

        DriveTrainState result{};
        if (std::isinf(turnRadius)) {
            // if the turn radius is infinite then we're not turning, so all wheels travel
            // at the desired forwards speed and the steerable wheels point forwards
            result.speeds[MOTOR_POSITION::FRONT_LEFT] = velocity;
            result.speeds[MOTOR_POSITION::FRONT_RIGHT] = -velocity;
            result.speeds[MOTOR_POSITION::REAR_LEFT] = velocity;
            result.speeds[MOTOR_POSITION::REAR_RIGHT] = -velocity;
            result.angles.left = 0;
            result.angles.right = 0;
        } else {
            // calculate the x component of the turn radius of each wheel
            // where x is left/right and y is direction of travel
            const float leftWheelTurnRadius = turnRadius - CONFIG::HALF_WHEEL_TRACK;
            const float rightWheelTurnRadius = turnRadius + CONFIG::HALF_WHEEL_TRACK;


            if (velocity == 0) {
                // if we're only turning, the speeds are symmetrical and just depends on the turn rate
                result.speeds[MOTOR_POSITION::FRONT_RIGHT] = result.speeds[MOTOR_POSITION::FRONT_LEFT] = -angularVelocity * CONFIG::STEERING_HYPOTENUSE;
                result.speeds[MOTOR_POSITION::REAR_RIGHT] = result.speeds[MOTOR_POSITION::REAR_LEFT] = -angularVelocity * CONFIG::HALF_WHEEL_TRACK;
                result.angles.left = getWheelAngle(leftWheelTurnRadius, velocity, CONFIG::Handedness::LEFT).constrained;
                result.angles.right = getWheelAngle(rightWheelTurnRadius, velocity, CONFIG::Handedness::RIGHT).constrained;
            } else {
                // if we're turning and moving forwards, the speeds are asymmetrical and depend on the turn rate and velocity

                // calculate the angles of the steerable wheels
                const SteeringAngle steeringAnglesLeft = getWheelAngle(leftWheelTurnRadius, velocity,
                                                                       CONFIG::Handedness::LEFT);
                result.angles.left = steeringAnglesLeft.constrained;
                const SteeringAngle steeringAnglesRight = getWheelAngle(rightWheelTurnRadius,
                                                                        velocity, CONFIG::Handedness::RIGHT);
                result.angles.right = steeringAnglesRight.constrained;

                // calculate the speeds of the front wheels
                result.speeds[MOTOR_POSITION::FRONT_LEFT] = getFrontWheelSpeed(
                        angularVelocity,
                        leftWheelTurnRadius,
                        steeringAnglesLeft.slip,
                        CONFIG::Handedness::LEFT
                        );
                result.speeds[MOTOR_POSITION::FRONT_RIGHT] = getFrontWheelSpeed(
                        angularVelocity,
                        rightWheelTurnRadius,
                        steeringAnglesRight.slip,
                        CONFIG::Handedness::RIGHT
                        );
                // calculate the speeds of the rear wheels
                result.speeds[MOTOR_POSITION::REAR_LEFT] = getRearWheelSpeed(velocity, leftWheelTurnRadius, CONFIG::Handedness::LEFT);
                result.speeds[MOTOR_POSITION::REAR_RIGHT] = getRearWheelSpeed(velocity, rightWheelTurnRadius, CONFIG::Handedness::RIGHT);

            }

        }

        return result;
    }

    float AckermannMixer::constrainSteeringAngle(float angle) const {
        // Clamp the value
        return std::clamp(angle, -maxSteeringAngle, maxSteeringAngle);
    }

    SteeringAngle
    AckermannMixer::getWheelAngle(float wheelTurnRadius, float velocity, CONFIG::Handedness side) const {
        SteeringAngle result{};

        if (velocity == 0) {
            // if we're stationary, the angle is calculated from the wheelbase and turn radius
            // we'll still constrain it to the maximum steering angle, but we won't apply any slip
            // TODO: is this correct?
            result.raw = std::atan2(wheelBase, wheelTurnRadius);;
            result.constrained = result.raw;
            result.slip = 0;
            return result;
        }
        // if we're moving, the angle is calculated from the wheelbase and turn radius

        result.raw = std::atan(wheelBase / wheelTurnRadius);
        if (side == CONFIG::Handedness::LEFT) {
            result.raw = -result.raw;
        }
        result.constrained = constrainSteeringAngle(result.raw);
        result.slip = result.raw - result.constrained;
        return result;
    }

    float
    AckermannMixer::getRearWheelSpeed(float velocity, const float wheelTurnRadius, CONFIG::Handedness side) const {
        float tmpSpeed = velocity * wheelTurnRadius / turnRadius;
        if (side == CONFIG::Handedness::RIGHT) {
            tmpSpeed = -tmpSpeed;
        }
        return tmpSpeed;
    }

    float AckermannMixer::getFrontWheelSpeed(float angularVelocity, const float wheelTurnRadius, const float slipAngle,
                                             CONFIG::Handedness side) const {
        float tmpSpeed = angularVelocity * std::sqrt(wheelTurnRadius * wheelTurnRadius + wheelBase * wheelBase);
        tmpSpeed = tmpSpeed * sign(wheelTurnRadius);
        if (side == CONFIG::Handedness::RIGHT) {
            tmpSpeed = -tmpSpeed;
        }
        // return modified speeds to correct for limited steering
        return tmpSpeed * std::cos(slipAngle);
    }

} // namespace MIXER