//
// Created by robbe on 02/12/2023.
//

#include "../include/stoker.h"

namespace STOKER {
    Stoker::Stoker(const pin_pair &pins, const MOTOR_POSITION::MotorPosition position, Direction direction = Direction::NORMAL_DIR) : motor(pins), motor_position_(position) {
        motor.init();
    }

    void Stoker::set_speed(const float speed) {
        motor.speed(speed);

    }

    void Stoker::update(const DriveTrainState newState) {
        current_motor_speed = newState.speeds[motor_position_];
    }

} // STOKER