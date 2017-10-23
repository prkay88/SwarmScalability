/* Include the controller definition */
#include "epuck_brownian.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <math.h> // for sin(), cos(), and pow()
#include <stdlib.h> // For rand()
#include <limits> // For max and min values


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
}

/****************************************/
/****************************************/

void CEPuckBrownian::ControlStep() {
   FlockingVector();
}

void CEPuckBrownian::FlockingVector(){
  const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABS->GetReadings();
  UInt32 countOFAliveBots=0;
  if(! tMsgs.empty()) {
    UInt32 inRadiusCount=0;
    for(size_t i = 0; i < tMsgs.size(); ++i) {
       if(tMsgs[i].Data[0] != DEAD){
          if(tMsgs[i].Range < repulsionDistance){
            /*Not sure if this is the right way to calculate the vector */
            //resultVector += CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);
            angleAccumulator += tMsgs[i].HorizontalBearing.GetValue();
            inRadiusCount++;
          }
       }
    }
    if(inRadiusCount > 0){

      epuckObstacleAvoidance();
    }
    /* Continue going until we've reached the threshold before turning back to the flock */
    else if(timeSinceLastAvoidance < moveTowardsFlockThreshold){
      /*We want to continue same direction we're currently going */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      timeSinceLastAvoidance++;

    }
    /* This will be the attractive behavior */
    else{
      if(!turningTowardsFlock){
        for(size_t i =0; i <tMsgs.size(); i++){
          /*Move towards center of swarm */
          if(tMsgs[i].Data[0] != DEAD){
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
          if(angleAccFloat <0){
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
            turn_left = true;
          }
          else{
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
          }
          time_spent_turning++;
          turningTowardsFlock = true;

        }
        //return averageBearing;
        timeSinceLastAvoidance = 0;
      }
      else{
        if(time_spent_turning < max_time_turning){
          if(turn_left){
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
          }
          else{
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
          }
        }
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
