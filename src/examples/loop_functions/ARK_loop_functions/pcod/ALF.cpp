/**
 * @file <ALF.h>
 *
 * @brief This is the source file of ALF, the ARK (Augmented Reality for Kilobots) loop function. Here, we present a simple experiment
 * in which the robots search for a special area. When the robot find the area, ALF signal him , and he stop moving.
 * Adapted for the PCOD, by Vander Freitas.
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 * @author Vander Freitas <vandercomp@gmail.com>
 * 
 */

#include "ALF.h"




double two_pi = 2.0*M_PI;
double half_pi = M_PI / 2.0;




/****************************************/
/*     Default LoopFunction Methods     */
/****************************************/
CALF::CALF(): CLoopFunctions(),m_unDataAcquisitionFrequency(10),
    m_unEnvironmentPlotUpdateFrequency(10),m_fTimeForAMessage(0.05){}
CALF::~CALF(){
}
void CALF::Init(TConfigurationNode& t_node) {

    /* Set the tracking type from the .argos file*/
    SetTrackingType(t_node);

    /* Get experiment variables from the .argos file*/
    GetExperimentVariables(t_node);

    /* Get the initial kilobots' states */
    SetupInitialKilobotsStates();


    // Open a log file
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

void CALF::Reset() {

    /* Close data file */
    m_cOutput.close();

    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

}

void CALF::Destroy() {
    /* Close data file */
    m_cOutput.close();

	pcod_model.destroy();
}


void CALF::PreStep(){

    /* Update the time variable required for the experiment (in sec)*/
    m_fTimeInSeconds=GetSpace().GetSimulationClock()/CPhysicsEngine::GetInverseSimulationClockTick();

	if((int)m_fTimeInSeconds % 10 == 0){  // update model at each 10s.
 		pcod_model.step_forward();

    /* Update the virtual sensor of the kilobots*/
    UpdateVirtualSensors();}
}


void CALF::PostStep(){

	
	/*if(pcod_model.get_ticks() % 10 == 0){
		for(int i=0; i<N; ++i)
			m_cOutput << m_fTimeInSeconds << "\t" << i << "\t" << pcod_model.particles[i].r_x << "\t" << pcod_model.particles[i].r_y << "\t" << pcod_model.particles[i].theta << "\n";
	}*/



//    /* Log experiment's results*/
//    if(((UInt16)m_fTimeInSeconds%m_unDataAcquisitionFrequency==0)&&((m_fTimeInSeconds-(UInt16)m_fTimeInSeconds)==0)){

//        m_cOutput << (UInt16) m_fTimeInSeconds << '\t';

//        UInt16 unKilobotID;
//        CVector2 cKilobotPosition;

//        for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){

//            unKilobotID=GetKilobotId(*m_tKilobotsEntities[it]);
//            cKilobotPosition=GetKilobotPosition(*m_tKilobotsEntities[it]);

//            m_cOutput << unKilobotID << '\t' << cKilobotPosition.GetX() << '\t' << cKilobotPosition.GetY() << '\t' << (UInt16)m_vecHasFood[unKilobotID] << '\t';

//        }

//        m_cOutput << std::endl;
//    }
}
/****************************************/
/*      Kilobot Tracking Function       */
/****************************************/
CVector2 CALF::GetKilobotPosition(CKilobotEntity& c_kilobot_entity){
    CVector2 vecKilobotPosition(c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    return vecKilobotPosition;
}

CRadians CALF::GetKilobotOrientation(CKilobotEntity& c_kilobot_entity)
{

    CRadians cZAngle;
    CRadians cYAngle;
    CRadians cXAngle;

    //Calculate the orientations of the kilobot
    CQuaternion cRobotOrientations = c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation;

    cRobotOrientations.ToEulerAngles(cZAngle,cYAngle, cXAngle);

    return cZAngle;
}

UInt16 CALF::GetKilobotId(CKilobotEntity& c_kilobot_entity){
    std::string strKilobotID((c_kilobot_entity).GetControllableEntity().GetController().GetId());
    return std::stoul(strKilobotID.substr(2));
}

CColor CALF::GetKilobotLedColor(CKilobotEntity &c_kilobot_entity){
    return c_kilobot_entity.GetLEDEquippedEntity().GetLED(0).GetColor();
}

/****************************************/
/*       Initialization functions       */
/****************************************/
void CALF::GetKilobotsEntities(){
    /*
     * Go through all the robots in the environment
     * and create a vector of pointers on their entities
     */

    /* Get the map of all kilobots from the space */
    CSpace::TMapPerType& mapKilobots=GetSpace().GetEntitiesByType("kilobot");
    /* Go through them */
    for(CSpace::TMapPerType::iterator it = mapKilobots.begin();
        it != mapKilobots.end();
        ++it) {
        m_tKilobotsEntities.push_back(any_cast<CKilobotEntity*>(it->second));
    }

    /* Create Kilobots individual messages */
    m_tMessages=TKilobotsMessagesVector(m_tKilobotsEntities.size());
}

void CALF::SetupInitialKilobotsStates(){
    /* Get the Kilobots entities from the space.*/
    GetKilobotsEntities();

    tKilobotsStates.resize(m_tKilobotsEntities.size());
    tLastTimeMessaged.resize(m_tKilobotsEntities.size());
    MinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotsEntities.size()*m_fTimeForAMessage/3.0);


    // Initalize PCOD
	N = m_tKilobotsEntities.size();

	dt = 1.0 / CPhysicsEngine::GetInverseSimulationClockTick();

	printf("dt = %f \n", dt);
	printf("N = %d  M = %d  omega0 = %f \n\n", N, M, omega0);
	printf("MinTimeBetweenTwoMsg = %f  time_for_a_msg__X__N_over_3: %f \n", MinTimeBetweenTwoMsg, m_tKilobotsEntities.size()*m_fTimeForAMessage/3.0);
	
	// Initiating the pcod model
	pcod_model.init(N, M, omega0, dt);


	CVector2 cKilobotPosition;

	printf("Initial phases: ");

    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotsEntities[it]);

    	cKilobotPosition=GetKilobotPosition(*m_tKilobotsEntities[it]);
    	pcod_model.particles[it].r_x = -cKilobotPosition.GetY();
    	pcod_model.particles[it].r_y = cKilobotPosition.GetX();
    	pcod_model.particles[it].theta = GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue() + half_pi;

    	if(pcod_model.particles[it].theta < 0.0)
    		pcod_model.particles[it].theta += two_pi;
    	else if(pcod_model.particles[it].theta > two_pi)
    		pcod_model.particles[it].theta -= two_pi;
    }
	printf("\n");
	
}

void CALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins outside the clustering hub*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    tKilobotsStates[unKilobotID]=OUTSIDE_CLUSTERING_HUB;
    tLastTimeMessaged[unKilobotID]=-1000;
}

void CALF::SetTrackingType(TConfigurationNode& t_tree){

    TConfigurationNode& tTrackingNode=GetNode(t_tree,"tracking");

    GetNodeAttribute(tTrackingNode, "position", m_bPositionTracking);

    GetNodeAttribute(tTrackingNode, "orientation", m_bOrientationTracking);

    GetNodeAttribute(tTrackingNode, "color", m_bColorTracking);

}


void CALF::GetExperimentVariables(TConfigurationNode& t_tree){

    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");

    /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);

    /* Get the frequency of data saving */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);

    /* Get the frequency of updating the environment plot */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);

    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);

    /* Get the model parameters */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "omega0", omega0, omega0);

    GetNodeAttributeOrDefault(tExperimentVariablesNode, "M", M, M);


    

}

/****************************************/
/*          Updating functions          */
/****************************************/

void CALF::UpdateVirtualSensors(){

	// Update the theta vector
	for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
		pcod_model.particles[it].theta = GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue() + half_pi;

		if(pcod_model.particles[it].theta < 0.0)
			pcod_model.particles[it].theta += two_pi;
		else if(pcod_model.particles[it].theta > two_pi)
			pcod_model.particles[it].theta -= two_pi;

		// Coordinate transformation is necessary *
		CVector2 cKilobotPosition;
		cKilobotPosition=GetKilobotPosition(*m_tKilobotsEntities[it]);
		pcod_model.particles[it].r_x = -cKilobotPosition.GetY();
		pcod_model.particles[it].r_y = cKilobotPosition.GetX();
	}


	for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
		/* Update the virtual sensor of a kilobot based on its current state */
		UpdateVirtualSensor(*m_tKilobotsEntities[it]);
	}
}






void CALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){

    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tKilobotMessage;

    /* Flag for existance of message to send*/
    bool bMessageToSend=false;

    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);

    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - tLastTimeMessaged[unKilobotID]< MinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }
    else{
        /*  Prepare the inividual kilobot's message */
        tKilobotMessage.m_sID = unKilobotID;
	tKilobotMessage.m_sData = pcod_model.d_theta[unKilobotID];
	tKilobotMessage.m_sType = 0;

	if(pcod_model.d_theta[unKilobotID] < 0)
		tKilobotMessage.m_sType = 1;

        /*  Set the message sending flag to True */
        bMessageToSend=true;

        tLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
    }

    /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
    if(bMessageToSend){

        for (int i = 0; i < 9; ++i) {
            m_tMessages[unKilobotID].data[i]=0;
        }

        m_tMessages[unKilobotID].data[0] = tKilobotMessage.m_sID;
	m_tMessages[unKilobotID].data[1] = tKilobotMessage.m_sType;


	// Sending only num_digits most significant digits of the phase
	int num_digits=5;
	long long temp= fabs(pcod_model.d_theta[unKilobotID])*pow(10,num_digits-1);
	int rem;
	

	for(int i=2; i<num_digits+2; ++i){
		//remainder of the temp value
		rem=temp%10;
		temp=temp/10;

		m_tMessages[unKilobotID].data[num_digits + 3 - i] = rem;
	}

	//printf("Sending: %f\n", m_fTimeInSeconds);

        /* Sending the message */
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
    }
    else{
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
    }

}


/****************************************/
/* Here Goes the user created functions */
/****************************************/









REGISTER_LOOP_FUNCTIONS(CALF, "ALF_pcod_loop_function")
