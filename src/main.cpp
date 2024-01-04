#include <cstdio>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "navigator.h"
#include "receiver.h"
#include "statemanager.h"
#include "state_estimator.h"
#include "motor2040.hpp"
#include "tank_steer_strategy.h"
#include "ackermann_strategy.h"
#include "drivetrain_config.h"
#include "utils.h"

Navigator *navigator;
bool shouldNavigate = false;

extern "C" void timer_callback(repeating_timer_t *t) {
    // Call the navigate function in the interrupt handler
    shouldNavigate = true;
}

int main() {
    stdio_init_all();

    i2c_inst_t* i2c_port0;
    initI2C(i2c_port0, 100 * 1000, CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN);

    // set up the state estimator
    auto *pStateEstimator = new STATE_ESTIMATOR::StateEstimator(i2c_port0);

    // set up the state manager
    using namespace STATEMANAGER;

    auto* pAckermannSteerStrategy = new MIXER::AckermannMixer(CONFIG::WHEEL_TRACK, CONFIG::WHEEL_BASE);
    auto* pStateManager = new StateManager(pAckermannSteerStrategy, pStateEstimator);


    // set up the receiver
    // if the cmake build flag RX_PROTOCOL is CPPM, then use the CPPM receiver
    // otherwise use the SBUS receiver
    Receiver *pReceiver = getReceiver(motor::motor2040::RX_ECHO);

    // set up the navigator
    navigator = new Navigator(pReceiver, pStateManager);

    // Initialize a hardware timer
    repeating_timer_t navigationTimer;
    add_repeating_timer_ms(
            20,
            reinterpret_cast<repeating_timer_callback_t>(timer_callback),
            nullptr,
            &navigationTimer
    );

    while (true) {
        // Do nothing in the main loop
        if (shouldNavigate) {
            // Call the navigate function in the interrupt handler
            navigator->navigate();
            shouldNavigate = false;
        }
    }
}
