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
    Encoder *FRONT_LEFT;
    Encoder *FRONT_RIGHT;
    Encoder *REAR_LEFT;
    Encoder *REAR_RIGHT;
};

namespace STATE_ESTIMATOR {
    using namespace COMMON;
    // define a State struct containing the state parameters that can be requested or tracked
    struct State {
        float x;
        float xdot;
        float y;
        float ydot;
        float velocity;
        float heading;
        float angularVelocity;
        DriveTrainState driveTrainState;
    };
    class StateEstimator: public Subject {
    public:
        explicit StateEstimator();

    protected:
        ~StateEstimator();  // Destructor to cancel the timer
    public:
        void showValues() const;
        void estimateState();
        void publishState() const;
        void addObserver(Observer* observer) override;
        void notifyObservers(DriveTrainState newState) override;

    private:
        Encoders encoders;
        static StateEstimator *instancePtr;
        repeating_timer_t *timer;
        State estimatedState;
        State previousState;
        DriveTrainState currentDriveTrainState;
        static void timerCallback(repeating_timer_t *timer);

    public:
        void updateCurrentDriveTrainState(const DriveTrainState& newDriveTrainState);

    private:
        void setupTimer() const;
        Observer* observers[10];
        int observerCount = 0;



    };

} // STATE_ESTIMATOR

#endif //OSOD_MOTOR_2040_STATE_ESTIMATOR_H
