//
// Created by robbe on 03/12/2023.
//

#ifndef OSOD_MOTOR_2040_STATE_ESTIMATOR_H
#define OSOD_MOTOR_2040_STATE_ESTIMATOR_H

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "motor2040.hpp"
#include "drivetrain_config.h"
#include "interfaces.h"
#include "types.h"

using namespace motor;
using namespace encoder;

struct Encoders {
    Encoder* FRONT_LEFT;
    Encoder* FRONT_RIGHT;
    Encoder* REAR_LEFT;
    Encoder* REAR_RIGHT;
};

namespace STATE_ESTIMATOR {
    using namespace COMMON;

    // define a State struct containing the state parameters that can be requested or tracked
    struct State {
        Velocity velocity;
        Odometry odometry;
        DriveTrainState driveTrainState;
    };

    class StateEstimator : public Subject {
    public:
        explicit StateEstimator();

    protected:
        ~StateEstimator(); // Destructor to cancel the timer
    public:
        void showValues() const;

        void estimateState();

        void publishState() const;

        void addObserver(Observer* observer) override;

        void notifyObservers(DriveTrainState newState) override;

        void updateCurrentDriveTrainState(const DriveTrainState& newDriveTrainState);

        static float wrap_pi(float heading);

        void calculate_bilateral_speeds(const MotorSpeeds& motor_speeds, SteeringAngles steering_angles,
                                        float& left_speed, float& right_speed);

    private:
        Encoder* encoders[MOTOR_POSITION::MOTOR_POSITION_COUNT];
        static StateEstimator* instancePtr;
        repeating_timer_t* timer;
        State estimatedState;
        State previousState;
        DriveTrainState currentDriveTrainState;

        static void timerCallback(repeating_timer_t* timer);

        void setupTimer() const;

        Observer* observers[10] = {};
        int observerCount = 0;

        void capture_encoders(Encoder::Capture* encoderCaptures) const;

        void get_position_deltas(Encoder::Capture encoderCaptures[4], float& distance_travelled,
                                                 float& heading_change) const;

        void calculate_new_position_orientation(State& tmpState, float distance_travelled, float heading_change);

        Velocity calculate_velocities(float heading, float left_speed, float right_speed);

        static MotorSpeeds get_wheel_speeds(const Encoder::Capture* encoderCaptures);
    };
} // STATE_ESTIMATOR

#endif //OSOD_MOTOR_2040_STATE_ESTIMATOR_H
