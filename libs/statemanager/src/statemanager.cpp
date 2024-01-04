//
// Created by robbe on 02/12/2023.
//

#include <cstdio>
#include "state_estimator.h"
#include "statemanager.h"
#include <libraries/servo2040/servo2040.hpp>
#include "motor2040.hpp"
#include "servo.hpp"

namespace STATEMANAGER {
    void StateManager::initialiseServo(servo::Servo*& servo, const uint pin, const float minPulse, const float midPulse, const float maxPulse, const float minValue = -0.7854, const float midValue = 0, const float maxValue = 0.7854) {
        servo = new servo::Servo(pin);
        servo->init();
        servo->calibration().apply_three_pairs(minPulse, midPulse, maxPulse, minValue, midValue, maxValue);
        servo->enable();
        servo->to_mid();
    }

    StateManager::StateManager(MIXER::MixerStrategy *mixerStrategy, STATE_ESTIMATOR::StateEstimator *stateEstimator) : mixerStrategy(mixerStrategy), stateEstimator(stateEstimator) {
        printf("State estimator created\n");
        printf("State manager created\n");
        // set up the stokers
        stokers.FRONT_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_A, Direction::NORMAL_DIR);
        stokers.FRONT_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_B, Direction::NORMAL_DIR);
        stokers.REAR_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_C, Direction::NORMAL_DIR);
        stokers.REAR_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_D, Direction::NORMAL_DIR);

        // set up the servos
        // left - ADC2 / PWM 6 - Pin 28
        initialiseServo(steering_servos.left, motor2040::ADC2, 2200, 1599, 1032);
        // right - TX_TRIG / PWM 0 - Pin 16
        initialiseServo(steering_servos.right, motor2040::TX_TRIG, 1221, 1670, 2200);
    }

    void StateManager::requestState(const STATE_ESTIMATOR::State& requestedState) {
        //printf("Requested state...\n");
        //printf("Velocity: %f ", requestedState.velocity);
        //printf("Angular velocity: %f ", requestedState.angularVelocity);
        //printf("\n");
        const COMMON::DriveTrainState driveTrainState = mixerStrategy->mix(requestedState.velocity, requestedState.angularVelocity);
        setDriveTrainState(driveTrainState);
        currentDriveTrainState = driveTrainState;
    }

    void StateManager::setServoSteeringAngle(const COMMON::DriveTrainState& driveTrainState, const CONFIG::Handedness side) const {
        servo::Servo *servo;
        float angle;
        float speed;

        if (side == CONFIG::Handedness::LEFT) {
            servo = steering_servos.left;
            angle = driveTrainState.angles.left;
            speed = driveTrainState.speeds.frontLeft;
        } else {
            servo = steering_servos.right;
            angle = driveTrainState.angles.right;
            speed = driveTrainState.speeds.frontRight;
        }

        if (std::fabs(speed) > 0.05) {
            if (not(servo->is_enabled())){
                servo->enable();
            }
            servo->value(angle);
        } else {
            servo->disable();
        }
    }

    void StateManager::setDriveTrainState(const COMMON::DriveTrainState& motorSpeeds) {
        stokers.FRONT_LEFT->set_speed(motorSpeeds.speeds.frontLeft);
        stokers.FRONT_RIGHT->set_speed(motorSpeeds.speeds.frontRight);
        stokers.REAR_LEFT->set_speed(motorSpeeds.speeds.rearLeft);
        stokers.REAR_RIGHT->set_speed(motorSpeeds.speeds.rearRight);
        setServoSteeringAngle(motorSpeeds, CONFIG::Handedness::LEFT);
        setServoSteeringAngle(motorSpeeds, CONFIG::Handedness::RIGHT);

        // save the current state
        currentDriveTrainState = motorSpeeds;

        // update the state estimator with the current state
        stateEstimator->updateCurrentDriveTrainState(motorSpeeds);
    }
} // StateManager