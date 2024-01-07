//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_STATEMANAGER_H
#define OSOD_MOTOR_2040_STATEMANAGER_H

#include "interfaces.h"
#include "types.h"
#include "receiver.h"
#include "state_estimator.h"
#include "stoker.h"
#include "mixer_strategy.h"
#include "servo.hpp"

namespace STATEMANAGER {
    using namespace COMMON;
    struct SteeringServos {
        servo::Servo* left;
        servo::Servo* right;
    };

    class StateManager {
    public:
        explicit StateManager(MIXER::MixerStrategy *mixerStrategy, STATE_ESTIMATOR::StateEstimator *stateEstimator);

        void initialiseServo(servo::Servo*& servo, uint pin, float minPulse, float midPulse, float maxPulse, float minValue, float midValue, float maxValue);

        void requestState(const STATE_ESTIMATOR::State& requestedState);

        void setServoSteeringAngle(const DriveTrainState& driveTrainState, CONFIG::Handedness side) const;

    private:
        MIXER::MixerStrategy *mixerStrategy;
        STATE_ESTIMATOR::StateEstimator *stateEstimator;
        DriveTrainState currentDriveTrainState{};
        STOKER::Stoker* stokers[MOTOR_POSITION::MOTOR_POSITION_COUNT] = {};
        SteeringServos steering_servos{};

        Observer* observers[MOTOR_POSITION::MOTOR_POSITION_COUNT] = {};
        int observerCount = 0;

        // max speed factor - scale the speed of the motors down to this value
        static constexpr float SPEED_EXTENT = 1.0f;

        void setDriveTrainState(const DriveTrainState& motorSpeeds);
    };

} // StateManager

#endif //OSOD_MOTOR_2040_STATEMANAGER_H
