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
#include "bno080.h"

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
    class StateEstimator : public Subject {
    public:
        explicit StateEstimator(BNO08x* IMUinstance);

    protected:
        ~StateEstimator(); // Destructor to cancel the timer
    public:
        void showValues() const;

        void estimateState();

        void publishState() const;

        void addObserver(Observer* observer) override;

        void notifyObservers(VehicleState newState) override;

        void updateCurrentSteeringAngles(const SteeringAngles& newSteeringAngles);

        void calculate_bilateral_speeds(const MotorSpeeds& motor_speeds, SteeringAngles steering_angles,
                                        float& left_speed, float& right_speed);

    private:
        Encoder* encoders[MOTOR_POSITION::MOTOR_POSITION_COUNT];
        static StateEstimator* instancePtr;
        repeating_timer_t* timer;
<<<<<<< HEAD
        VehicleState estimatedState;
        VehicleState previousState;
=======
        BNO08x* IMU;
        float heading_offset;
        //TODO: (related to issue #42) actually use timer (defined above) instead of fixed interval
        const uint32_t timerInterval = 50;  // Interval in milliseconds
        State estimatedState;
        State previousState;
>>>>>>> heading_from_IMU_take2
        DriveTrainState currentDriveTrainState;
        SteeringAngles currentSteeringAngles;

        static void timerCallback(repeating_timer_t* timer);

        void setupTimer() const;

        Observer* observers[10] = {};
        int observerCount = 0;

        void capture_encoders(Encoder::Capture* encoderCaptures) const;
        
        void get_latest_heading(float& heading);

        bool initialise_heading_offset();

<<<<<<< HEAD
        void calculate_new_position_orientation(VehicleState& tmpState, float distance_travelled, float heading_change);
=======
        void get_position_delta(Encoder::Capture encoderCaptures[4], float& distance_travelled) const;
>>>>>>> heading_from_IMU_take2

        void calculate_new_position(State& tmpState, float distance_travelled, float heading);

        Velocity calculate_velocities(float new_heading, float previous_heading, float left_speed, float right_speed);

        static MotorSpeeds get_wheel_speeds(const Encoder::Capture* encoderCaptures);

        [[nodiscard]] SteeringAngles estimate_steering_angles() const;
    };
} // STATE_ESTIMATOR

#endif //OSOD_MOTOR_2040_STATE_ESTIMATOR_H
