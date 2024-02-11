#include "receiver.h"
#include "statemanager.h"
#include "communicator.h"
#include "types.h"
#include "interfaces.h"
#include "drivetrain_config.h"
#include "types.h"
#include "interfaces.h"
#include "drivetrain_config.h"
#include "waypoint_navigation.h"

using namespace COMMON;
class Navigator: public Observer {
public:
    explicit Navigator(const Receiver* receiver,
                        STATEMANAGER::StateManager *stateManager,
                        STATE_ESTIMATOR::StateEstimator* estimator,
                        CONFIG::SteeringStyle direction);
    ~Navigator();
    void navigate();
    CONFIG::SteeringStyle driveDirection; //factor to change requested motor speed direction based on what we currently consider the front

    NAVIGATION_MODE::Mode navigationMode;

    void update(const VehicleState newState) override;


private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    Communicator *communicator_;
    PAYLOADS::StatePayload requestedStatePayload;
    NAVIGATION_MODE::Mode determineMode(float signal);
    WAYPOINTS::WaypointNavigation waypointNavigator;
    STATE_ESTIMATOR::StateEstimator* pStateEstimator;

    VehicleState current_state;
    float waypointModeThreshold = 0; //if signal above this, we're move into waypoint mode
    float waypointIndexThreshold = 0.5; //if signal above this, reset the waypoint index
    float setHeadingThreshold = -0.5; //if signal below this, set the heading
    float setOriginThreshold = 0.5; //if signal above this, set the odometry origin
    float expo(float signal, float expoValue); // apply an exponential response to the channel
    float velocityExpoValue = 0.7;
    float steeringExpoValue = 0.7;
    bool shouldResetWaypointIndex(float signal);
    bool shouldSetHeading(float signal);
    bool shouldSetOdometryOrigin(float signal);
    void setHeading(); //local method that's linked to the stateEstimator set_Heading_Offset method
    void setOrigin(); //local method that's linked to the stateEstimator set_Odometry_Offset method
    NAVIGATION_MODE::Mode parseTxSignals(const ReceiverChannelValues& signals); //function to use "spare" transmitter channels as auxiliary inputs
};
