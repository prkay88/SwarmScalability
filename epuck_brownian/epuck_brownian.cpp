/* Include the controller definition */
#include "epuck_brownian.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

#include <math.h>   // for sin(), cos(), and pow()
#include <stdlib.h> // For rand()
#include <limits>   // For max and min values
#include <iostream> // Stream class to write on files
#include <fstream> // Stream class to read from files
#include <string>
/****************************************/
/****************************************/

CEPuckBrownian::CEPuckBrownian() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcLightSens(NULL),
   m_fWheelVelocity(2.5f) {}
   //m_pcLightSens(NULL) {}

/****************************************/
/****************************************/

void CEPuckBrownian::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><epuck_brownian><actuators> and
    * <controllers><epuck_brownian><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"    );
   m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"       );
   m_pcLightSens = GetSensor  <CCI_EyeBotLightSensor           >("eyebot_light"      );
   m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   GetNodeAttribute(t_node, "NumberOfRobots",          NumberOfRobots);
   GetNodeAttribute(t_node, "NumberOfDeadRobots",      NumberOfDeadRobots);
   GetNodeAttribute(t_node, "ShortRepulsionDistance",  ShortRepulsionDistance);
   GetNodeAttribute(t_node, "LongRepulsionDistance",   LongRepulsionDistance);
   GetNodeAttribute(t_node, "TimeForFailureTicks",     TimeForFailureTicks);
   GetNodeAttribute(t_node, "OmegaTimeTicks",          OmegaTimeTicks);
   GetNodeAttribute(t_node, "BeaconPosition",          BeaconPosition);
   GetNodeAttribute(t_node, "GoalThreshold",		   GoalThreshold);
   GetNodeAttribute(t_node, "ResultsDirectoryPath",    results_path);
}

/****************************************/
/****************************************/

void CEPuckBrownian::ControlStep() {
  /*Using the state variable to determine what to do */
  switch (myState) {
    /*Main case where robots try to maintain a relative proximity to one another */
    case FLOCKING:
      flockingVector(2.5);
      /*If a robot can dected the beacon they transition state */
      if(detectedBeaconLight())
      {
        myState = SEEN_GOAL;
      }
      break;
   /*After a robot has seen the goal, they ar allowed to move farther away from the center of swarm */
    case SEEN_GOAL:
      flockingVector(5.5);
      /*If a robot has reached the beacon it transitions to another state to let the swarm know it has reached its goal */
      if(flockReachedBeacon())
      {
        myState = FOUND_BEACON;
      }
      break;
    /*Modeling where a robot loses power completely */
    case CASE_1_ERROR:
      powerFailure();
      break;
    /*Modeling where a robot loses its sensors to detect rest of swarm */
    case CASE_2_ERROR:
      sensorFailure();
      break;
    /*Modeling  where robot loses power to motor but maintains in contact with other robots */
    case CASE_3_ERROR:
      motorFailure();
      break;
  }
  stringstream ss;
  string results_file_name = ss.str();
  results_full_path = results_path;
  //LOG << "results_full_path " << results_full_path << std::endl;
  if(GetId().compare("ep1") == 0 && flockReachedBeacon())
  {
    ofstream results_output_stream;
    results_output_stream.open(results_full_path, ios::app);
    results_output_stream << "NumberOfRobots, "
                          << "NumberOfDeadRobots, "
                          << "ShortRepulsionDistance, "
                          << "LongRepulsionDistance, "
                          << "TimeForFailureTicks, "
                          << "OmegaTimeTicks, "
                          << "TotalTime" << endl
                          << NumberOfRobots << ", "
                          << NumberOfDeadRobots << ", "
                          << ShortRepulsionDistance << ", "
                          << LongRepulsionDistance << ", "
                          << TimeForFailureTicks << ", "
                          << OmegaTimeTicks << endl;
                        //  << CSimulator::GetInstance().GetRandomSeed() << endl;
    results_output_stream.close();
    //cout << "Finished Initializing the epuck_brownian" << std::endl;
  }
  /*Sending the robots state to all other robots */
  m_pcRABA->SetData(0, myState);
  totalTime++;
}

void CEPuckBrownian::flockingVector(float repulsionDistance){
  /*Getting the data from all other robots */
  const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABS->GetReadings();
  UInt32 countOFAliveBots=0;
  //argos::LOG <<"tMsgs.size = " << tMsgs.size() << std::endl;
  /*Checking to see if our packet contains data */
  if(! tMsgs.empty()) {
    UInt32 inRadiusCount=0;
    for(size_t i = 0; i < tMsgs.size(); ++i) {
       /*If a robot is in Case 1 or Case 2 error we ignore any data that they send */
       if(tMsgs[i].Data[0] != CASE_1_ERROR && tMsgs[i].Data[0] != CASE_2_ERROR){
          /*Checking to see if there are any robots that we should repulse from */
          if(tMsgs[i].Range < repulsionDistance){
            inRadiusCount++;
          }
       }
    }

    /* Preform obstacle avoidence for those robots */
    if(inRadiusCount > 0){
      //argos::LOG << "Using Epuck OA with inRadiusCount of:  " << inRadiusCount  << std::endl;
      epuckObstacleAvoidance();
    }

    /* Continue going until we've reached the threshold before turning back to the flock */
    else if(timeSinceLastAvoidance < moveTowardsFlockThreshold){
      /*We want to continue same direction we're currently going */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      timeSinceLastAvoidance++;
      //argos::LOG <<"Moving straight with timeSinceLastAvoidance = " << timeSinceLastAvoidance <<std::endl;

    }

    /* This will be the attractive behavior */
    else{
      //argos::LOG << "Inside attractive behavior" << std::endl;
      if(!turningTowardsFlock){
        for(size_t i =0; i <tMsgs.size(); i++){
          /*Move towards center of swarm */
          if(tMsgs[i].Data[0] != CASE_1_ERROR && tMsgs[i].Data[0] != CASE_2_ERROR){
            angleAccumulator += tMsgs[i].HorizontalBearing.GetValue();
            countOFAliveBots++;
          }
        }
        if(countOFAliveBots > 0){
          time_spent_turning = 0;
          angleAccumulator /= countOFAliveBots;
          float angleAccFloat = (float) angleAccumulator;
          float term =  (time_to_turn_2pi/(2*M_PI));
          max_time_turning = (angleAccFloat*term);
          if(max_time_turning >(2*M_PI) || max_time_turning < (-2*M_PI))
          {
            max_time_turning = 2*M_PI;
          }
          //argos::LOG <<"max_time_turning: " << max_time_turning << std::endl;
          if(angleAccFloat <0){
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
            turn_left = true;
          }
          else{
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
            turn_left = false;
          }
          time_spent_turning++;
          turningTowardsFlock = true;

        }
        //return averageBearing;
        timeSinceLastAvoidance = 0;
      }
      //This is where we are turning towards the flocks
      else{
        if(time_spent_turning < max_time_turning){
          if(turn_left){
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
          }
          else{
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
          }
          //argos::LOG <<"max_time_turning: " << max_time_turning << std::endl;
          //argos::LOG <<"time_spent_turning: " << time_spent_turning << std::endl;
          time_spent_turning++;
        }
        //if exceed max time we go stright.
        else{
          m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
          turningTowardsFlock = false;
          time_spent_turning = 0;
          max_time_turning = 0;
        }
      }
    }
  }
}

void CEPuckBrownian::epuckObstacleAvoidance()
{
  /* Get the highest reading in front of the robot, which corresponds to the closest object */
  Real fMaxReadVal = m_pcProximity->GetReadings()[0];
  UInt32 unMaxReadIdx = 0;
  if(fMaxReadVal < m_pcProximity->GetReadings()[1]) {
     fMaxReadVal = m_pcProximity->GetReadings()[1];
     unMaxReadIdx = 1;
  }
  if(fMaxReadVal < m_pcProximity->GetReadings()[7]) {
     fMaxReadVal = m_pcProximity->GetReadings()[7];
     unMaxReadIdx = 7;
  }
  if(fMaxReadVal < m_pcProximity->GetReadings()[6]) {
     fMaxReadVal = m_pcProximity->GetReadings()[6];
     unMaxReadIdx = 6;
  }
  /* Do we have an obstacle in front? */
  if(fMaxReadVal > 0.0f) {
    /* Yes, we do: avoid it */
    if(unMaxReadIdx == 0 || unMaxReadIdx == 1) {
      /* The obstacle is on the left, turn right */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
    }
    else {
      /* The obstacle is on the left, turn right */
      m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
    }
  }
  else {
    /* No, we don't: go straight */
     m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
  }
}

//Weisstein, Eric W. "Random Number." From MathWorld--A Wolfram Web Resource. http://mathworld.wolfram.com/RandomNumber.html
float CEPuckBrownian::drawFromPowerLawDistribution( float min, float max, float mu )
{
  float unif_var = rand()*1.0f/RAND_MAX;
  argos::LOG << "Unif Rand: " << unif_var  << std::endl;
  float min_power = pow(min, mu+1);
  argos::LOG << "Min Power: " << min_power << std::endl;
  float max_power = pow(max, mu+1);
  argos::LOG << "Max Power: " << max_power  << std::endl;
  float base_variate = (max_power-min_power)*unif_var+min_power;
  argos::LOG << "Base Variate: " << base_variate  << std::endl;
  float exponent = 1/(mu+1);
  argos::LOG << "Exponent: " << exponent  << std::endl;
  float p =  pow(base_variate, exponent);
  argos::LOG << "Result: " << p << std::endl;
  return p;
}

/* Stimulates the robots sensing the beacon using their light sensors.
   If one robot senses the beacon return true. */
bool CEPuckBrownian::detectedBeaconLight()
{
  bool detectedLight = false;
  const CCI_EyeBotLightSensor::TReadings& lightReadings = m_pcLightSens->GetReadings();

  for(size_t i =0; i < lightReadings.size(); ++i){
    if(lightReadings[i].Value > 0.0){

        float beacon_x = BeaconPosition.GetX();
        float beacon_y = BeaconPosition.GetY();
        float robot_x = RobotPosition.GetX();
        float robot_y = RobotPosition.GetY();

        float euclid_dist = (beacon_x - robot_x) * (beacon_x - robot_x) +
                            (beacon_y - robot_y) * (beacon_y - robot_y);
        if(euclid_dist < 1)
        {
          argos::LOG << "Beacon light detected " << std::endl;
          return true;
        }

    }
  }
  return false;
}

/* Checks if any rover is within the ShortRepulsionDistance using
   the QuadRotorPositionActuator to the beacon and returns a boolean */
bool CEPuckBrownian::flockReachedBeacon()
{
  RobotPosition = m_pcPosSens->GetReading().Position;

  float beacon_x = BeaconPosition.GetX();
  float beacon_y = BeaconPosition.GetY();
  float robot_x = RobotPosition.GetX();
  float robot_y = RobotPosition.GetY();

  float euclid_dist = (beacon_x - robot_x) * (beacon_x - robot_x) +
                      (beacon_y - robot_y) * (beacon_y - robot_y);

  if(euclid_dist < 0.01){
    argos::LOG << "Beacon light reached " << std::endl;
    argos::LOG << " robot_x : " << robot_x << " robot_y: " << robot_y << std::endl;
    argos::LOG << " beacon_x : " << beacon_x << " beacon_y : " << beacon_y << std::endl;
    argos::LOG << "euclid_dist: " << euclid_dist << std::endl;
    return true;
  }
  return false;
}

/* CASE 1: Failure when the robots are dead due to powerfailure */
void CEPuckBrownian::powerFailure()
{
  m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

/* CASE 2: Failure when the robots sensor fails result in the robot getting lost, just preform obstacle avoidence*/
void CEPuckBrownian::sensorFailure()
{
  epuckObstacleAvoidance();
}

/* CASE 3: Failure when the right, left, or both wheels are broken */
void CEPuckBrownian::motorFailure()
{
  const char motorErrorsTypes[4] = {'0', '1', '2'};
  int randType = rand() % 3; //generates a random number between 0 and 2
  argos::LOG << motorErrorsTypes[randType] << std::endl;

  /* spin left - right motor broken */
  if(randType == 0)
  {
    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
  }

  /* spin right - left motor broken */
  else if(randType == 1)
  {
    m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
  }

  /* stop - both motors are broken, powerfailure */
  else
  {
     m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
  }
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckBrownian, "epuck_brownian_controller")
