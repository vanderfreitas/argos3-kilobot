/**
 * @file <ALF.h>
 *
 * @brief This is the header file of ALF, the ARK (Augmented Reality for Kilobots) loop function. Here, we present a simple experiment
 * in which the robots move randomly in search for a special area. When a robot find the area, ALF signal him, and he stop moving.
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 * @author Vander Freitas <vandercomp@gmail.com>
 */





#ifndef ALF_H
#define ALF_H

namespace argos {
class CSpace;
//class CFloorEntity;
class CSimulator;
}

#include <math.h>


#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>


using namespace argos;



class CALF : public CLoopFunctions
{

public:
    /************************************************/
    /*       Basic ARGoS Loop function methods      */
    /************************************************/
    CALF();
    virtual ~CALF();
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
    virtual void PostStep();
    //virtual CColor GetFloorColor(const CVector2& vec_position_on_plane); // Used to plot the Virtual environment on the floor

    /************************************************/
    /*          ARK Loop function methods           */
    /************************************************/
    /** Initialization related methods */
    /* Get a Vector of all the Kilobots in the space */
    void GetKilobotsEntities();

    /* Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotsStates();

    /* Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Experiment configuration methods (From .argos files) */
    /* Set tracking type for the experiments */
    void SetTrackingType(TConfigurationNode& t_tree);

    /* Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode& t_tree);

    /* Get experiment variables */
    void GetExperimentVariables(TConfigurationNode& t_tree);

    /** Virtual environment visualization updating */
    /* Plot the environment */
    void PlotEnvironment();


    /** Kilobots' states updating methods */
    /* Get the messages to send to the Kilobots according their positions */
    void UpdateKilobotsState();

    /* Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Virtual Sensors updating methods */
    /* Get the messages to send to the Kilobots according their positions */
    void UpdateVirtualSensors();

    /* Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity& c_kilobot_entity);

    /** Virtual Envirenment Updating methods */
    /* Update the virtual environment if dynamics */
    void UpdateVirtualEnvironments();

    /* Update the virtual environment if dynamics */
    void UpdatesVirtualEnvironmentsBasedOnKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Kilobot Tracking methods */
    /* Get the position of a Kilobot */
    CVector2 GetKilobotPosition(CKilobotEntity& c_kilobot_entity);

    /* Get the orientation of a Kilobot */
    CRadians GetKilobotOrientation(CKilobotEntity& c_kilobot_entity);

    /* Get the kilobot ID */
    UInt16 GetKilobotId(CKilobotEntity& c_kilobot_entity);

    /* Get the kilobot LED color */
    CColor GetKilobotLedColor(CKilobotEntity& c_kilobot_entity);


	typedef std::map<CKilobotEntity*, std::vector<CVector3> > TWaypointMap;
   TWaypointMap m_tWaypoints;

	inline const TWaypointMap& GetWaypoints() const {
      return m_tWaypoints;
   }

private:
    /************************************************/
    /*  Basic attributes of the ARK loop function   */
    /************************************************/

    /* Random number generator */
    CRandom::CRNG* m_pcRNG;

    /* List of the Kilobots in the space */
    typedef std::vector<CKilobotEntity*> TKilobotEntitiesVector;
    TKilobotEntitiesVector m_tKilobotsEntities;

    /* List of the messages sent by communication entities */
    typedef std::vector<message_t> TKilobotsMessagesVector;
    TKilobotsMessagesVector m_tMessages;

    /* ARK kilobot message*/
    typedef struct {
        UInt8 m_sType:4;
        UInt16 m_sID:10;
        UInt16 m_sData:10;
    } m_tALFKilobotMessage;

    /* Tracking Flags*/
    bool m_bPositionTracking;
    bool m_bOrientationTracking;
    bool m_bColorTracking;

    /************************************/
    /*  Virtual Environment variables   */
    /************************************/
    /* virtual environment struct*/
    struct SVirtualArea
    {
        CVector2 Centre;
        Real Radius;
        CColor Color;
    };

   SVirtualArea m_sClustringHub;


    typedef enum
    {
        OUTSIDE_CLUSTERING_HUB=0,
        INSIDE_CLUSTERING_HUB=1,
    } SRobotState;

    std::vector<SRobotState> tKilobotsStates;
    std::vector<Real> tLastTimeMessaged;
    Real MinTimeBetweenTwoMsg;

    /************************************/
    /*       Experiment variables       */
    /************************************/

    /* Experiment time in seconds */
    Real m_fTimeInSeconds;

    /* output file for data acquizition */
    std::ofstream m_cOutput;

    /* output file name*/
    std::string m_strOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;

    /* Environment plot update frequency in ticks */
    UInt16 m_unEnvironmentPlotUpdateFrequency;

    /* Time for one kilobot message to be sent */
    Real m_fTimeForAMessage;


    /************************************/
    /*       Robots state vector        */
    /************************************/
    double *state_vector; // states
    int sv_id; // states indexes for each robot
    int N; // number of robots
};



#endif
