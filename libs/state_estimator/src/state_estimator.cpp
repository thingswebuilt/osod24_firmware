//
// Created by robbe on 03/12/2023.
//
#include <cstdio>
#include <bitset>
#include "state_estimator.h"
#include "drivetrain_config.h"
#include "encoder.hpp"
#include "bno080.h"
#include "utils.h"

#include "tf_luna.h"
namespace STATE_ESTIMATOR {
    StateEstimator *StateEstimator::instancePtr = nullptr;

    StateEstimator::StateEstimator(BNO08x* IMUinstance, i2c_inst_t* port, CONFIG::SteeringStyle direction) : encoders{
            [MOTOR_POSITION::FRONT_LEFT] =new Encoder(pio0, 0, motor2040::ENCODER_A, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            [MOTOR_POSITION::FRONT_RIGHT] =new Encoder(pio0, 1, motor2040::ENCODER_B, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            [MOTOR_POSITION::REAR_LEFT] = new Encoder(pio0, 2, motor2040::ENCODER_C, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            [MOTOR_POSITION::REAR_RIGHT] = new Encoder(pio0, 3, motor2040::ENCODER_D, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV)
    }, timer(new repeating_timer_t), estimatedState(), previousState(), currentDriveTrainState() {
        encoders[MOTOR_POSITION::FRONT_LEFT]->init();
        encoders[MOTOR_POSITION::FRONT_RIGHT]->init();
        encoders[MOTOR_POSITION::REAR_LEFT]->init();
        encoders[MOTOR_POSITION::REAR_RIGHT]->init();
        i2c_port = port;
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
        estimatedState.tofDistances = getAllLidarDistances(i2c_port);
        IMU = IMUinstance;

        instancePtr = this;
        // check if we're going to use the ToF sensors for arena localisation
        // (a naN arena size means we're not going to use the arena for localisation):
        arenaLocalisation = !isnan(CONFIG::ARENA_SIZE);

        odometryOffsetRequest.x = odometryOffsetRequest.y = odometryOffsetRequest.heading = 0;

        driveDirection = direction;

        zeroHeading();

        setupTimer();
    }

    void StateEstimator::showValues() const {
        // printf("FRONT_LEFT: %ld ", encoders.FRONT_LEFT->count());
        // printf("FRONT_RIGHT: %ld ", encoders.FRONT_RIGHT->count());
        // printf("REAR_LEFT: %ld ", encoders.REAR_LEFT->count());
        // printf("REAR_RIGHT: %ld ", encoders.REAR_RIGHT->count());
        // printf("\n");
        // printf("X: %f, Y: %f, Velocity: %f, Heading: %f, turn rate: %f, front ToF: %f\n",
        //    estimatedState.odometry.x,
        //    estimatedState.odometry.y,
        //    estimatedState.velocity.velocity,
        //    estimatedState.odometry.heading,
        //    estimatedState.velocity.angular_velocity,
           // estimatedState.tofDistances.front);
    }

    void StateEstimator::showValuesViaCSV() const {

        printf("%i, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
           millis(),
           estimatedState.odometry.x,
           estimatedState.odometry.y,
           estimatedState.odometry.heading,
           estimatedState.tofDistances.front,
           estimatedState.tofDistances.right,
           estimatedState.tofDistances.rear,
           estimatedState.tofDistances.left);
    }

    void StateEstimator::publishState() const {
        showValuesViaCSV();
    }

    void StateEstimator::addObserver(Observer* observer) {
        if (observerCount < 10) {
            observers[observerCount++] = observer;
        }
    }

    void StateEstimator::notifyObservers(const VehicleState newState) {
        for (int i = 0; i < observerCount; i++) {
            observers[i]->update(newState);
        }
    }

    void StateEstimator::captureEncoders(Encoder::Capture* encoderCaptures) const {
        for(int i = 0; i < MOTOR_POSITION::MOTOR_POSITION_COUNT; i++) {
            encoderCaptures[i] = encoders[i]->capture();
        }
    }

    void StateEstimator::calculateBilateralSpeeds(const MotorSpeeds& motor_speeds, const SteeringAngles steering_angles, float& left_speed, float& right_speed) {
        left_speed = (motor_speeds[MOTOR_POSITION::FRONT_LEFT] * cos(steering_angles.left)
                      + motor_speeds[MOTOR_POSITION::REAR_LEFT]) / 2;

        // convert average wheel rotation speed to linear speed
        left_speed = left_speed * CONFIG::WHEEL_DIAMETER / 2;

        right_speed = (motor_speeds[MOTOR_POSITION::FRONT_RIGHT] * cos(steering_angles.right)
                       + motor_speeds[MOTOR_POSITION::REAR_RIGHT]) / 2;
        right_speed = right_speed * CONFIG::WHEEL_DIAMETER / 2;
    }

    void StateEstimator::getPositionDelta(Encoder::Capture encoderCaptures[4], float& distance_travelled) const {
        // Calculate average wheel rotation delta for left and right sides
        // for the front wheels we only use the forward component of the movement
        //this should give a more accurate estimate for distance_travelled
        float left_travel = (encoderCaptures[MOTOR_POSITION::FRONT_LEFT].radians_delta() * cos(estimatedState.driveTrainState.angles.left)
                             + encoderCaptures[MOTOR_POSITION::REAR_LEFT].radians_delta()) / 2;
        float right_travel = (encoderCaptures[MOTOR_POSITION::FRONT_RIGHT].radians_delta() * cos(estimatedState.driveTrainState.angles.right)
                              + encoderCaptures[MOTOR_POSITION::REAR_RIGHT].radians_delta()) / 2;

        // convert wheel rotation to distance travelled in meters
        distance_travelled = ((left_travel - right_travel) / 2) * CONFIG::WHEEL_DIAMETER / 2;
    }

    void StateEstimator::calculateNewPosition(VehicleState& tmpState, const float distance_travelled, const float heading) {
        //use the latest heading and distance travleled to update the estiamted position
        tmpState.odometry.x -= driveDirection * distance_travelled * sin(heading);
        tmpState.odometry.y += driveDirection * distance_travelled * cos(heading);

        //now actually update odometry's heading
        tmpState.odometry.heading = heading;

        //constrain heading to +/-pi
        tmpState.odometry.heading = wrap_pi(tmpState.odometry.heading);
    }

    Velocity StateEstimator::calculateVelocities(const float new_heading, const float previous_heading, const float left_speed, const float right_speed) {
        // TODO return a velocities struct instead of setting individual values
        Velocity tmpVelocity{};
        tmpVelocity.velocity = (left_speed - right_speed) / 2;
        tmpVelocity.x_dot = -driveDirection * tmpVelocity.velocity * sin(new_heading);
        tmpVelocity.y_dot = driveDirection * tmpVelocity.velocity * cos(new_heading);

        tmpVelocity.angular_velocity = 1000 * (wrap_pi(new_heading - previous_heading)) / timerInterval;
        return tmpVelocity;
    }

    MotorSpeeds StateEstimator::getWheelSpeeds(const Encoder::Capture* encoderCaptures) {
        MotorSpeeds wheelSpeeds{};
        for(int i = 0; i < MOTOR_POSITION::MOTOR_POSITION_COUNT; i++) {
            wheelSpeeds.speeds[static_cast<MOTOR_POSITION::MotorPosition>(i)] = encoderCaptures[i].radians_per_second();
        }
        return wheelSpeeds;
    }

    SteeringAngles StateEstimator::estimateSteeringAngles() const {
        return currentSteeringAngles;
    }

    void StateEstimator::estimateState() {

        //update odometry offsets based on external requests
        processOdometryOffsets();

        // instantiate a copy of the current state
        VehicleState tmpState = estimatedState;
        
        //get current encoder state
        Encoder::Capture encoderCaptures[MOTOR_POSITION::MOTOR_POSITION_COUNT];
        captureEncoders(encoderCaptures);

        // calculate position deltas

        float distance_travelled = 0.0f;
        getPositionDelta(encoderCaptures, distance_travelled);


        float heading = 0.0f;
        getLatestHeading(heading);

        //calculate new position and orientation
        calculateNewPosition(tmpState, distance_travelled, heading);

        //calculate speeds

        //get wheel speeds
        tmpState.driveTrainState.speeds = getWheelSpeeds(encoderCaptures);

        // estimate steering angles
        tmpState.driveTrainState.angles = estimateSteeringAngles();

        // calculate left and right speeds
        float left_speed;
        float right_speed;
        calculateBilateralSpeeds(tmpState.driveTrainState.speeds, tmpState.driveTrainState.angles, left_speed, right_speed);

        //calc all velocities
        tmpState.velocity = calculateVelocities(tmpState.odometry.heading, previousState.odometry.heading, left_speed, right_speed);

        // get ToF data
        tmpState.tofDistances = getAllLidarDistances(i2c_port);

        if (arenaLocalisation) {
            localisationEstimate = localisation(tmpState.odometry.heading, tmpState.tofDistances);
            tmpState.odometry = filterPositions(tmpState.odometry, localisationEstimate);
        }

        // update the estimated states
        previousState = estimatedState;
        estimatedState = tmpState;

        // notify observers of the new state
        notifyObservers(estimatedState);
    }

    void StateEstimator::getLatestHeading(float& heading) {
      //default latest heading is the current heading
      heading = estimatedState.odometry.heading;

      //if possible, update the heading with the latest from the IMU
        if (IMU->getSensorEvent() == true) {
            if (IMU->getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
                heading = IMU->getYaw() - IMUHeadingOffset;
            }
        }
    }

    void StateEstimator::setupTimer() const {
        // Example configuration (adjust as needed)

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

    void StateEstimator::updateCurrentSteeringAngles(const SteeringAngles& newSteeringAngles) {
        currentSteeringAngles = newSteeringAngles;
    }

    void StateEstimator::zeroHeading() {
        // function sets a heading_offset request such that the (new) heading will be zero
        odometryOffsetRequest.heading = estimatedState.odometry.heading;
    }

    void StateEstimator::requestOdometryOffset(float xOffset, float yOffset, float extraHeadingOffset){
        odometryOffsetRequest.x = xOffset;
        odometryOffsetRequest.y = yOffset;
        odometryOffsetRequest.heading = extraHeadingOffset;
    }

    void StateEstimator::processOdometryOffsets(){
        estimatedState.odometry.x = estimatedState.odometry.x - odometryOffsetRequest.x;
        estimatedState.odometry.y = estimatedState.odometry.y - odometryOffsetRequest.y;
        IMUHeadingOffset = IMUHeadingOffset + odometryOffsetRequest.heading;

        // since we've applied the requested offsets, set them back to zero
        odometryOffsetRequest.x = odometryOffsetRequest.y = odometryOffsetRequest.heading = 0;
    }

    pair<float, float> StateEstimator::calculatePossiblePositions(float angle, float distance) {
        /**
         * Calculates the potential X, Y positions of the robot based on a single distance measurement
         * and the robot's heading. This method assumes the robot is facing a square arena and uses trigonometry
         * to infer its position relative to the nearest wall.
         *
         * @param angle The current heading of the robot in radians.
         * @param distance The distance measurement from a ToF sensor to the nearest wall.
         * @return A pair of floats representing the estimated X or Y positions in the arena.
         */
        angle = wrap_pi(angle); //constrain(wrap) to our usual +/-pi range

        float x_pos = 0.0f, y_pos = 0.0f;

        if (angle < 0) {
            x_pos = CONFIG::ARENA_SIZE - distance * cos(angle + M_PI / 2);
        } else {
            x_pos = distance * cos(angle + 1.5f * M_PI);
        }

        if ((angle < (0.5f * M_PI)) && (angle > (-0.5f * M_PI))) {
            y_pos = CONFIG::ARENA_SIZE - distance * cos(angle);
        } else {
            y_pos = distance * cos(angle + M_PI);
        }

        return {x_pos, y_pos};
    }

    Pose StateEstimator::localisation(float heading, FourToFDistances tof_distances) {
        /**
         * Estimates the robot's position within the arena by aggregating distance measurements from all ToF sensors.
         * It calculates potential positions for each sensor based on the robot's current heading and the sensor readings,
         * then combines these estimates to produce a more accurate localization within the arena.
         *
         * @param heading The current heading of the robot in radians.
         * @param tof_distances A struct containing distance measurements from all ToF sensors.
         * @return A Pose struct representing the estimated position (X, Y) of the robot.
         */

        // Calculate possible positions for each sensor
        // these are inferred possible positions, depending on which arena wall each sensor
        // is measuring, the sensor could be used to infer an x position or y position
        auto [Fx, Fy] = calculatePossiblePositions(heading, tof_distances.front);
        auto [Rx, Ry] = calculatePossiblePositions(heading - M_PI_2, tof_distances.right);
        auto [Bx, By] = calculatePossiblePositions(heading - M_PI, tof_distances.rear);
        auto [Lx, Ly] = calculatePossiblePositions(heading - 3 * M_PI_2, tof_distances.left);

        // Combine the calculated positions into lists for easier manipulation
        std::array<float, NUM_TOF_SENSORS> x_positions = {Fx, Rx, Bx, Lx};
        std::array<float, NUM_TOF_SENSORS> y_positions = {Fy, Ry, By, Ly};

        // evaluate all the possible purmutation of the positions and return the best-fitting position
        return evaluatePositionPermutations(heading, x_positions, y_positions);
    }

    Pose StateEstimator::evaluatePositionPermutations(
            float heading,
            const std::array<float, NUM_TOF_SENSORS>& xPositions,
            const std::array<float, NUM_TOF_SENSORS>& yPositions){
     // evaluate all the possible purmutation of the positions and return the
     // best-fitting position. The position is based on an arena-centre origin

        float lowestVariance = numeric_limits<float>::max();
        Pose bestEstimate = {0.0f, 0.0f, 0.0f};

        for (int permutationNo = 1; permutationNo < NUM_PERMUTATIONS; ++permutationNo) {
            // create 16 permutations (all the possible combinations of the two lists of possible
            // positions), then iterate through them to check which is most self-consistent (lowest
            // variance), assume that permutation is the most likely, best estimate of our position
            PermutationResult permutation = createPermutation(permutationNo, xPositions, yPositions);
            auto [totalVariance, xMean, yMean] = calculateCoordinateVariance(permutation);

            if (totalVariance < lowestVariance) {
                lowestVariance = totalVariance;
                bestEstimate.x = driveDirection * (xMean - CONFIG::ARENA_SIZE/2);
                bestEstimate.y = driveDirection * (yMean - CONFIG::ARENA_SIZE/2);
                bestEstimate.heading = heading;
            }
        }

        return bestEstimate;
        }

    tuple<float, float, float> StateEstimator::calculateCoordinateVariance(const PermutationResult& result) {
        /**
         * Computes the variance and mean of X and Y position estimates within the limits of valid data points as specified in the PermutationResult.
         * The function calculates the mean for X and Y positions separately, then determines the variance for each set based on the deviation from their respective means.
         * The total variance is the sum of the X and Y variances. This approach ensures only valid data points are considered, aligning with the static allocation strategy in embedded systems.
         *
         * @param result The result structure from createPermutation containing X and Y positions with their valid counts.
         * @return A tuple with the total variance, mean of X positions, and mean of Y positions.
         */

        float xMean = std::accumulate(result.xList.begin(), result.xList.begin() + result.xSize, 0.0f) / result.xSize;
        float yMean = std::accumulate(result.yList.begin(), result.yList.begin() + result.ySize, 0.0f) / result.ySize;

        // Calculate variances
        auto xVariance = std::accumulate(result.xList.begin(), result.xList.begin() + result.xSize, 0.0f,
                                        [xMean](float acc, float x) { return acc + std::pow(x - xMean, 2); }) / result.xSize;
        auto yVariance = std::accumulate(result.yList.begin(), result.yList.begin() + result.ySize, 0.0f,
                                        [yMean](float acc, float y) { return acc + std::pow(y - yMean, 2); }) / result.ySize;

        float totalVariance = xVariance + yVariance;

        return {totalVariance, xMean, yMean};
    }

    Pose StateEstimator::filterPositions(Pose odometryEstimate, Pose localisationEstimate){
        /**
         * Combines the odometry and localization estimates to produce a filtered position estimate.
         * This method uses a weighted average approach to merge the estimates, potentially improving
         * the accuracy of the robot's perceived position within the arena.
         *
         * @param odometryEstimate A Pose struct representing the position estimate based on odometry.
         * @param localisationEstimate A Pose struct representing the position estimate based on localization.
         * @return A Pose struct representing the filtered position and heading of the robot.
         */

        Pose filteredPosition;
        filteredPosition.x = (1 - localisation_weighting) * odometryEstimate.x + localisation_weighting * localisationEstimate.x;
        filteredPosition.y = (1 - localisation_weighting) * odometryEstimate.y + localisation_weighting * localisationEstimate.y;
        filteredPosition.heading = odometryEstimate.heading;
        return filteredPosition;
    }


    StateEstimator::PermutationResult StateEstimator::createPermutation(
            int permutation,
            const std::array<float, NUM_TOF_SENSORS>& xPositions,
            const std::array<float, NUM_TOF_SENSORS>& yPositions) {
        /**
         * Generates a specific permutation of valid X and Y position estimates based on the given permutation number.
         * The permutation number is interpreted as a binary representation, where each bit indicates whether a position
         * from the input X or Y positions list is selected. Positions corresponding to '1' bits are added to the Y list,
         * and positions corresponding to '0' bits are added to the X list. The function returns arrays containing the
         * selected positions along with the actual count of valid positions in each array.
         *
        * @param permutation The permutation number, interpreted as a binary pattern for selecting X or Y positions.
        * @param xPositions The array of potential X positions.
        * @param yPositions The array of potential Y positions.
        * @return A structure containing the X and Y lists along with their counts of valid elements.
        */
        std::array<float, NUM_TOF_SENSORS> xList = {0.0f};
        std::array<float, NUM_TOF_SENSORS> yList = {0.0f};
        size_t xSize = 0, ySize = 0;
        bitset<4> binary(permutation);
        for (size_t sensor = 0; sensor < NUM_TOF_SENSORS; ++sensor) {
            if (binary[sensor]) {
                yList[ySize++] = yPositions[sensor];
            } else {
                xList[xSize++] = xPositions[sensor];
            }
        }
        return {xList, yList, xSize, ySize};
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

