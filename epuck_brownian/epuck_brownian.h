/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example controller for obstacle avoidance with the e-puck.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/epuck_brownian.argos
 */

#ifndef EPUCK_BROWNIAN_H
#define EPUCK_BROWNIAN_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of Range and Bearing Sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the light sensor */
#include <argos3/plugins/robots/eye-bot/control_interface/ci_eyebot_light_sensor.h>
/* Definition of the CRange variable */
#include <argos3/core/utility/math/range.h>

#include <string>

using namespace argos;
using namespace std;

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEPuckBrownian : public CCI_Controller {

public:

   /* Class constructor. */
   CEPuckBrownian();

   /* Class destructor. */
   virtual ~CEPuckBrownian() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_brownian_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}



private:

   /* Init variables */
   int NumberOfRobots;
   int NumberOfDeadRobots;
   int TimeForFailureTicks;
   int OmegaTimeTicks;
   float ShortRepulsionDistance;
   float LongRepulsionDistance;
   float GoalThreshold;
   CVector3 BeaconPosition;
   string results_path;

   void flockingVector(float repulsionDistance);
   void epuckObstacleAvoidance();
   float drawFromPowerLawDistribution( float min, float max, float mu );
   bool detectedBeaconLight();
   bool flockReachedBeacon();
   void powerFailure();
   void sensorFailure();
   void motorFailure();

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the e-puck proximity sensor */
   CCI_ProximitySensor* m_pcProximity;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPosSens;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the eye-bot light sensor */
   CCI_EyeBotLightSensor* m_pcLightSens;
   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_brownian_controller> section.
    */

   /* Wheel speed. */
   Real m_fWheelVelocity;
   string results_full_path;
   CVector3 RobotPosition;
   int totalTime = 0;

   float max_time_between_turns = 5; //seconds
   float time_spent_turning = 0;
   float max_time_turning = 0;
   float time_spent_going_straight = 0;
   float time_to_turn_2pi = 15;

   float angleAccumulator;
   float repulsionDistance = 20; //Max distance that a robot repulses another
   float timeSinceLastAvoidance = 0;
   float moveTowardsFlockThreshold = 4.5;
   bool turningTowardsFlock = false;
   bool turn_left = false;
   bool goBackwards = false;

   UInt32 timeSteps = 0;

   enum State {FLOCKING, SEEN_GOAL, FOUND_BEACON, CASE_1_ERROR, CASE_2_ERROR, CASE_3_ERROR };
    State myState = SEEN_GOAL;
};

#endif
