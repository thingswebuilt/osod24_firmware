#include <cstdio>
#include "navigator.h"
#include "statemanager.h"
#include "state_estimator.h"
#include "types.h"
#include "drivetrain_config.h"
#include "communicator.h"
#include "waypoint_navigation.h"
#include "types.h"

Navigator::Navigator(const Receiver* receiver,
                     STATEMANAGER::StateManager* stateManager,
                     STATE_ESTIMATOR::StateEstimator* stateEstimator,
                     CONFIG::SteeringStyle direction) {
    this->receiver = receiver;
    this->pStateManager = stateManager;
    this->communicator_ = &Communicator::getInstance();
    this->requestedStatePayload  = PAYLOADS::StatePayload();
    this->pStateEstimator = stateEstimator;
    driveDirection = direction;
    navigationMode = NAVIGATION_MODE::REMOTE_CONTROL;
}

void Navigator::navigate() {
    if (receiver->get_receiver_data()) {
        // send payload to the communicator
        PAYLOADS::SerialTransferAvailableStatus serial_transfer_available_status(true);
        communicator_->sendPacket(serial_transfer_available_status);

        ReceiverChannelValues values = receiver->get_channel_values();

        NAVIGATION_MODE::Mode newMode;
        //check if the extra Tx channels should trigger anything
        newMode = parseTxSignals(values);
        if (newMode != navigationMode){
            printf("changing mode to mode %d, where 1=RC, 2=waypoint, 3=Pi\n", newMode);
            navigationMode = newMode;
        }
        STATE_ESTIMATOR::VehicleState requestedState{};
        switch (navigationMode) {
        case NAVIGATION_MODE::WAYPOINT:
            waypointNavigator.navigate(current_state);
            requestedState.velocity.velocity = driveDirection * waypointNavigator.desiredV;
            requestedState.velocity.angular_velocity = waypointNavigator.desiredW;
            break;
        default: //includes REMOTE_CONTROL, which is the default
            // Apply expo function to AIL and ELE
            float expoAIL = expo(values.AIL, steeringExpoValue);
            float expoELE = expo(values.ELE, velocityExpoValue);

            // send the receiver data to the state manager
            // TODO: use a queue to send the receiver data to the state manager
            requestedState.velocity.velocity = driveDirection * expoELE * CONFIG::MAX_VELOCITY;
            requestedState.velocity.angular_velocity = expoAIL * CONFIG::MAX_ANGULAR_VELOCITY;
            pStateManager->requestState(requestedState);
            break;
        }
        // TODO: use a queue to send the receiver data to the state manager
        pStateManager->requestState(requestedState);

        // send the requested state via the communicator
        requestedStatePayload.velocity = requestedState.velocity.velocity;
        requestedStatePayload.angular_velocity = requestedState.velocity.angular_velocity;
        communicator_->sendPacket(requestedStatePayload);
    } else {
        // printf("No receiver data available\n");
        PAYLOADS::SerialTransferAvailableStatus serial_transfer_available_status(false);
        communicator_->sendPacket(serial_transfer_available_status);
    }
}

NAVIGATION_MODE::Mode Navigator::determineMode(float signal){
    NAVIGATION_MODE::Mode mode;
    if (signal > waypointModeThreshold){
        mode = NAVIGATION_MODE::WAYPOINT;
    } else {
        mode = NAVIGATION_MODE::REMOTE_CONTROL;
    }
    return mode;
}

bool Navigator::shouldResetWaypointIndex(float signal){
    return (signal > waypointIndexThreshold);
}

void Navigator::update(const VehicleState newState) {
    current_state = newState;
}

bool Navigator::shouldSetHeading(float signal){
    return (signal < setHeadingThreshold);
}

bool Navigator::shouldSetOdometryOrigin(float signal){
    return (signal > setOriginThreshold);
}

void Navigator::setHeading(){
    pStateEstimator->zeroHeading();
}

void Navigator::setOrigin(){
    pStateEstimator->requestOdometryOffset(current_state.odometry.x, current_state.odometry.y, 0);
}

NAVIGATION_MODE::Mode Navigator::parseTxSignals(const ReceiverChannelValues& signals){
    // function to use "spare" transmitter channels as auxiliary inputs
    // currently can set (zero) odoemtry heading and and origin
        if (shouldResetWaypointIndex(signals.THR)){
            printf("resetting waypoint index to 0.\n");
            waypointNavigator.targetWaypointIndex = 0;
        }
        if (shouldSetHeading(signals.RUD)){
            printf("setting current heading to 0.\n");
            setHeading();
        }
        if (shouldSetOdometryOrigin(signals.RUD)){
            printf("setting current position as zero for odometry.\n");
            setOrigin();
        }
        return determineMode(signals.AUX);
}

float Navigator::expo(float input, float expoValue) {
    return input * (abs(input) * expoValue + (1.0f - expoValue));
}

Navigator::~Navigator() = default;
