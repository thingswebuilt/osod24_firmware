//
// Created by robbe on 03/12/2023.
//
#include <cstdio>
#include "state_estimator.h"
#include "drivetrain_config.h"
#include "encoder.hpp"

namespace STATE_ESTIMATOR {
    StateEstimator *StateEstimator::instancePtr = nullptr;

    StateEstimator::StateEstimator() : encoders{
            [MOTOR_POSITION::FRONT_LEFT] =new Encoder(pio0, 0, motor2040::ENCODER_A, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            [MOTOR_POSITION::FRONT_RIGHT] =new Encoder(pio0, 1, motor2040::ENCODER_B, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            [MOTOR_POSITION::REAR_LEFT] = new Encoder(pio0, 2, motor2040::ENCODER_C, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            [MOTOR_POSITION::REAR_RIGHT] = new Encoder(pio0, 3, motor2040::ENCODER_D, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV)
    }, timer(new repeating_timer_t), estimatedState(), previousState(), currentDriveTrainState() {
        encoders[MOTOR_POSITION::FRONT_LEFT]->init();
        encoders[MOTOR_POSITION::FRONT_RIGHT]->init();
        encoders[MOTOR_POSITION::REAR_LEFT]->init();
        encoders[MOTOR_POSITION::REAR_RIGHT]->init();

        // Initialize the State struct member variables
        estimatedState.odometry.x = 0.0f;
        estimatedState.velocity.x_dot = 0.0f;
        estimatedState.odometry.y = 0.0f;
        estimatedState.velocity.y_dot = 0.0f;
        estimatedState.velocity.velocity = 0.0f;
        estimatedState.odometry.heading = 0.0f;
        estimatedState.velocity.angular_velocity = 0.0f;
        estimatedState.driveTrainState.speeds[MOTOR_POSITION::FRONT_LEFT] = 0.0f;
        estimatedState.driveTrainState.speeds[MOTOR_POSITION::FRONT_RIGHT] = 0.0f;
        estimatedState.driveTrainState.speeds[MOTOR_POSITION::REAR_LEFT] = 0.0f;
        estimatedState.driveTrainState.speeds[MOTOR_POSITION::REAR_RIGHT] = 0.0f;
        estimatedState.driveTrainState.angles.left = 0.0f;
        estimatedState.driveTrainState.angles.right = 0.0f;
        
        instancePtr = this;
        setupTimer();
    }

    void StateEstimator::showValues() const {
        printf("FRONT_LEFT: %ld ", encoders[MOTOR_POSITION::FRONT_LEFT]->count());
        printf("FRONT_RIGHT: %ld ", encoders[MOTOR_POSITION::FRONT_RIGHT]->count());
        printf("REAR_LEFT: %ld ", encoders[MOTOR_POSITION::REAR_LEFT]->count());
        printf("REAR_RIGHT: %ld ", encoders[MOTOR_POSITION::REAR_RIGHT]->count());
        printf("\n");
        printf("X: %f, Y: %f, Velocity: %f, Heading: %f, turn rate: %f\n", 
           estimatedState.odometry.x,
           estimatedState.odometry.y,
           estimatedState.velocity.velocity,
           estimatedState.odometry.heading,
           estimatedState.velocity.angular_velocity);
    }

    void StateEstimator::publishState() const {
        showValues();
    }

    void StateEstimator::addObserver(Observer* observer) {
        if (observerCount < 10) {
            observers[observerCount++] = observer;
        }
    }

    void StateEstimator::notifyObservers(const DriveTrainState newState) {
        for (int i = 0; i < observerCount; i++) {
            observers[i]->update(newState);
        }
    }

    void StateEstimator::capture_encoders(Encoder::Capture* encoderCaptures) const {
        for(int i = 0; i < MOTOR_POSITION::MOTOR_POSITION_COUNT; i++) {
            encoderCaptures[i] = encoders[i]->capture();
        }
    }

    float StateEstimator::wrap_pi(const float heading) {
        //constrain heading to +/-pi

        const double wrapped = heading > M_PI ? heading - M_TWOPI : heading < -M_PI ? heading + M_TWOPI : heading;
        return static_cast<float>(wrapped);
    }

    void StateEstimator::calculate_bilateral_speeds(const MotorSpeeds& motor_speeds, const SteeringAngles steering_angles, float& left_speed, float& right_speed) {
        left_speed = (motor_speeds[MOTOR_POSITION::FRONT_LEFT] * cos(steering_angles.left)
                      + motor_speeds[MOTOR_POSITION::REAR_LEFT]) / 2;

        // convert average wheel rotation speed to linear speed
        left_speed = left_speed * CONFIG::WHEEL_DIAMETER / 2;

        right_speed = (motor_speeds[MOTOR_POSITION::FRONT_RIGHT] * cos(steering_angles.right)
                       + motor_speeds[MOTOR_POSITION::REAR_RIGHT]) / 2;
        right_speed = right_speed * CONFIG::WHEEL_DIAMETER / 2;
    }

    void StateEstimator::get_position_deltas(Encoder::Capture encoderCaptures[4], float& distance_travelled, float& heading_change) const {
        // Calculate average wheel rotation delta for left and right sides
        // for the front wheels we only use the forward component of the movement
        //this should give a more accurate estimate for distance_travelled
        // but less accurate for heading_change.
        // In future, the heading will be taken entirely from the IMU though
        float left_travel = (encoderCaptures[MOTOR_POSITION::FRONT_LEFT].radians_delta() * cos(estimatedState.driveTrainState.angles.left)
                             + encoderCaptures[MOTOR_POSITION::REAR_LEFT].radians_delta()) / 2;
        float right_travel = (encoderCaptures[MOTOR_POSITION::FRONT_RIGHT].radians_delta() * cos(estimatedState.driveTrainState.angles.right)
                              + encoderCaptures[MOTOR_POSITION::REAR_RIGHT].radians_delta()) / 2;

        // convert wheel rotation to distance travelled in meters
        left_travel = left_travel * CONFIG::WHEEL_DIAMETER / 2;
        right_travel = right_travel * CONFIG::WHEEL_DIAMETER / 2;

        distance_travelled = (left_travel - right_travel) / 2;
        heading_change = (left_travel + right_travel) / CONFIG::WHEEL_TRACK;
    }

    void StateEstimator::calculate_new_position_orientation(State& tmpState, const float distance_travelled, const float heading_change) {
        //calc a temp heading halfway between old heading and new
        //assumed to be representative of heading during distance_travelled
        const float tempHeading = tmpState.odometry.heading + heading_change / 2;
        tmpState.odometry.x += distance_travelled * sin(tempHeading);
        tmpState.odometry.y += distance_travelled * cos(tempHeading);

        //now actually update heading
        tmpState.odometry.heading += heading_change;

        //constrain heading to +/-pi
        tmpState.odometry.heading = wrap_pi(tmpState.odometry.heading);
    }

    Velocity StateEstimator::calculate_velocities(const float heading, const float left_speed, const float right_speed) {
        // TODO return a velocities struct instead of setting individual values
        Velocity tmpVelocity{};
        tmpVelocity.velocity = (left_speed - right_speed) / 2;
        tmpVelocity.x_dot = tmpVelocity.velocity * sin(heading);
        tmpVelocity.y_dot = tmpVelocity.velocity * cos(heading);

        //now less accurate as we're taking the wrong component of the front wheel speeds.
        // TODO: fix this by using the IMU for heading
        tmpVelocity.angular_velocity = (left_speed + right_speed) / CONFIG::WHEEL_TRACK;
        return tmpVelocity;
    }

    MotorSpeeds StateEstimator::get_wheel_speeds(const Encoder::Capture* encoderCaptures) {
        MotorSpeeds wheelSpeeds{};
        for(int i = 0; i < MOTOR_POSITION::MOTOR_POSITION_COUNT; i++) {
            wheelSpeeds.speeds[static_cast<MOTOR_POSITION::MotorPosition>(i)] = encoderCaptures[i].radians_per_second();
        }
        return wheelSpeeds;
    }

    void StateEstimator::estimateState() {
        // instantiate a copy of the current state
        State tmpState = estimatedState;
        
        //get current encoder state
        Encoder::Capture encoderCaptures[MOTOR_POSITION::MOTOR_POSITION_COUNT];
        capture_encoders(encoderCaptures);

        // calculate position deltas

        float distance_travelled = 0.0f;
        float heading_change = 0.0f;
        get_position_deltas(encoderCaptures, distance_travelled, heading_change);

        //calculate new position and orientation
        calculate_new_position_orientation(tmpState, distance_travelled, heading_change);

        //calculate speeds

        //get wheel speeds
        tmpState.driveTrainState.speeds = get_wheel_speeds(encoderCaptures);

        // calculate left and right speeds
        float left_speed;
        float right_speed;
        calculate_bilateral_speeds(tmpState.driveTrainState.speeds, tmpState.driveTrainState.angles, left_speed, right_speed);

        //calc all velocities
        tmpState.velocity = calculate_velocities(tmpState.odometry.heading, left_speed, right_speed);

        // update the estimated state
        estimatedState = tmpState;

        // notify observers of the new state
        notifyObservers(estimatedState.driveTrainState);

    }

    void StateEstimator::setupTimer() const {
        // Example configuration (adjust as needed)
        constexpr uint32_t timerInterval = 50;  // Interval in milliseconds

        // Set up the repeating timer with the callback
        if (!add_repeating_timer_ms(timerInterval,
                                    reinterpret_cast<repeating_timer_callback_t>(&StateEstimator::timerCallback),
                                    nullptr, timer)) {
            // Handle error if timer creation fails
        }
    }

    void StateEstimator::timerCallback(repeating_timer_t *timer) {
        if (instancePtr != nullptr) {
            instancePtr->estimateState();
            instancePtr->publishState();
        }
    }

    void StateEstimator::updateCurrentDriveTrainState(const DriveTrainState& newDriveTrainState) {
        currentDriveTrainState = newDriveTrainState;
    }

    StateEstimator::~StateEstimator() {
        delete encoders[MOTOR_POSITION::FRONT_LEFT];
        delete encoders[MOTOR_POSITION::FRONT_RIGHT];
        delete encoders[MOTOR_POSITION::REAR_LEFT];
        delete encoders[MOTOR_POSITION::REAR_RIGHT];
        // Cancel the timer in the destructor
        cancel_repeating_timer(timer);

    }
}
// STATE_ESTIMATOR

