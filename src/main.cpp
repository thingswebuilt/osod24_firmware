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
#include "SerialTransfer.h"
#include "communicator.h"
#include "utils.h"
#include "balance_port.h"
#include "bno080.h"
#include "tf_luna.h"


Navigator *navigator;
int32_t navigationPeriodMs = 20;

// calculate the period to read the cell status - divide the time in ms by the navigation period, and floor the result
int32_t shouldReadCellCount = 2000 / navigationPeriodMs;

// create struct to pass to the timer callback function
// containing:
// shouldNavigate - bool to indicate if the navigate function should be called
// shouldReadCellStatus - bool to indicate if the cell status should be read
// shouldReadCellCount - the number of times the cell status should be read
// navigateCount - a counter to keep track of the number of times the navigate function has been called
struct TimerCallbackData {
    bool shouldNavigate;
    bool shouldReadCellStatus;
    int32_t shouldReadCellCount;
    int32_t navigateCount;
};

TimerCallbackData timerCallbackData = {
        .shouldNavigate = false,
        .shouldReadCellStatus = false,
        .shouldReadCellCount = shouldReadCellCount,
        .navigateCount = 0,
};

extern "C" void timer_callback(repeating_timer_t *t) {
    // cast t->user_data to TimerCallbackData
    auto *user_data = reinterpret_cast<TimerCallbackData *>(t->user_data);
    user_data->shouldNavigate = true;
    if (user_data->navigateCount > user_data->shouldReadCellCount) {
        user_data->shouldReadCellStatus = true;
        user_data->navigateCount = 0;
    } else {
        user_data->navigateCount++;
    }
}

int main() {
    stdio_init_all();
    i2c_inst_t* i2c_port0;
    initI2C(i2c_port0, 100 * 1000, CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN);
    bool adcPresent;
    BalancePort balancePort;
    adcPresent = balancePort.initADC(i2c_port0); // Initialize ADC

    //set up IMU
    BNO08x IMU;
    while (IMU.begin(CONFIG::BNO08X_ADDR, i2c_port0)==false) {
        printf("BNO08x not detected at default I2C address. Check wiring. Freezing\n");
        scan_i2c_bus();
        sleep_ms(1000);
    }
    IMU.enableRotationVector();

    // set up the state estimator
    auto *pStateEstimator = new STATE_ESTIMATOR::StateEstimator(&IMU, i2c_port0, CONFIG::DRIVING_STYLE);

    // set up the state manager
    using namespace STATEMANAGER;

    auto* pAckermannSteerStrategy = new MIXER::AckermannMixer(CONFIG::WHEEL_TRACK, CONFIG::WHEEL_BASE);
    auto* pStateManager = new StateManager(pAckermannSteerStrategy, pStateEstimator);

    // set up the serial transfer
    SerialTransfer myTransfer;
    configST myConfig;
    myConfig.debug = false;
    myTransfer.begin(myConfig);

    // set up the receiver
    // if the cmake build flag RX_PROTOCOL is CPPM, then use the CPPM receiver
    // otherwise use the SBUS receiver
    Receiver *pReceiver = getReceiver(motor::motor2040::RX_ECHO);

    // set up the Communicator instance, and set the serial transfer
    // Note that the Communicator instance is a singleton, so elsewhere in the code
    // you can get the instance by calling Communicator::getInstance()
    auto *pCommunicator = &Communicator::getInstance();
    pCommunicator->setSerialTransfer(&myTransfer);

    // set up the navigator
    navigator = new Navigator(pReceiver, pStateManager, pStateEstimator, CONFIG::DRIVING_STYLE);
    pStateEstimator->addObserver(navigator);

    // Initialize a hardware timer
    repeating_timer_t navigationTimer;
    add_repeating_timer_ms(
            navigationPeriodMs,
            reinterpret_cast<repeating_timer_callback_t>(timer_callback),
            &timerCallbackData,
            &navigationTimer
    );

    while (true) {
        // Do nothing in the main loop
        if (timerCallbackData.shouldNavigate) {
            navigator->navigate();
            timerCallbackData.shouldNavigate = false;

}

        if (adcPresent && timerCallbackData.shouldReadCellStatus) {
            balancePort.raiseCellStatus();
            timerCallbackData.shouldReadCellStatus = false;
        }
    }
}
