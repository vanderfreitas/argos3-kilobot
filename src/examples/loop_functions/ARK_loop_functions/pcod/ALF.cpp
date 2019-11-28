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


#include <math.h>
#include <numeric>
#include <complex>
#include <cmath>
#include <iomanip>





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

	//pcod_model.destroy();
}


void CALF::PreStep(){
   
    /* Update the time variable required for the experiment (in sec)*/
    m_fTimeInSeconds=GetSpace().GetSimulationClock()/CPhysicsEngine::GetInverseSimulationClockTick();


    // Exporting final result
    if(m_fTimeInSeconds == 4999.0){
    	int N = m_tKilobotsEntities.size();
    	int M = m_tKilobotsEntities.size();

        double pmt[M];
        double theta[N];

        for(UInt16 it=0;it< m_tKilobotsEntities.size();it++)
            theta[it] = GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue();


        std::complex<double> res = 0.0;
		int i, m;
	    
	    // M-th moment of the order parameter
		for(m=1; m<=M; ++m){
			res = 0.0;
			for(i=0; i<N; ++i){		
				std::complex<double> aux(cos((double)m*theta[i]), sin((double)m*theta[i]));
				res += aux;
			}
			res = res / double(N);

			pmt[m-1] = sqrt(std::real(res)*std::real(res) + std::imag(res)*std::imag(res));
		}


        m_cOutput << pmt[0];
        std::cout << pmt[0];
        for(int i=1; i<M; ++i){
            m_cOutput << "\t" << pmt[i];
            std::cout << "\t" << pmt[i];
        }
        m_cOutput << "\n";
        std::cout << "\n";
    }



    /* Update the virtual sensor of the kilobots*/
    UpdateVirtualSensors();
}


void CALF::PostStep(){

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
    MinTimeBetweenTwoMsg = m_tKilobotsEntities.size()*m_fTimeForAMessage/3.0; // Max<Real>(1.0, m_tKilobotsEntities.size()*m_fTimeForAMessage/3.0);

    printf("MinTimeBetweenTwoMsg = %f   \n", MinTimeBetweenTwoMsg);

    // Initalize PCOD
	N = m_tKilobotsEntities.size();

    // The state vector stores: robot_phase, theta_1, theta_2,.., theta_N, r1(x),..,rN(x), r1(y),..,rN(y)
    state_vector = new double[3*m_tKilobotsEntities.size()]; 
    sv_id = 0;


	CVector2 cKilobotPosition;

    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotsEntities[it]);

    	cKilobotPosition=GetKilobotPosition(*m_tKilobotsEntities[it]);

        // Filling state vector
        state_vector[it] = GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue();
        state_vector[it+N] =  cKilobotPosition.GetX();
        state_vector[it+2*N] = cKilobotPosition.GetY();

    }
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
}

/****************************************/
/*          Updating functions          */
/****************************************/

void CALF::UpdateVirtualSensors(){


	// Update the theta vector
	for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        // Keep angle within the interval [-pi, pi)
        double argos_bounded_theta = atan2(sin(GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue()), cos(GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue()) );  

		// Coordinate transformation is necessary
		CVector2 cKilobotPosition;
		cKilobotPosition=GetKilobotPosition(*m_tKilobotsEntities[it]);

        // Filling state vector
        state_vector[it] = argos_bounded_theta;
        state_vector[it+N] = cKilobotPosition.GetX();
        state_vector[it+2*N] = cKilobotPosition.GetY();
	}

	for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
		/* Update the virtual sensor of a kilobot based on its current state */
		UpdateVirtualSensor(*m_tKilobotsEntities[it]);
	}

	++sv_id;
    if(sv_id > 2)
        sv_id = 0;
}



UInt8 convert_double_to_UInt8_(double angle, int num_digits){
    long temp= fabs(angle*pow(10,num_digits-1));

    if(temp > 1023)
        temp = 1023;

    return temp;
}



void CALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){

    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tMessage, tKilobotMessage_theta, tKilobotMessage_x, tKilobotMessage_y;

    /* Flag for existance of message to send*/
    bool bMessageToSend=false;

    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);

    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - tLastTimeMessaged[unKilobotID]< MinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }
    else{
    	int index_to_be_sent = 0;

    	// The id of the robot state we are sending info. (to to whom we are sending to)
    	if(sv_id == 0){ //left neighbor
			if(unKilobotID > 0)
				index_to_be_sent = unKilobotID - 1;
			else 
				index_to_be_sent = m_tKilobotsEntities.size()-1;

    	}else if(sv_id == 1){ // Own state
    		index_to_be_sent = unKilobotID;

    	}else if(sv_id == 2){ // Right neighbor
    		if(unKilobotID < m_tKilobotsEntities.size()-1)
				index_to_be_sent = unKilobotID + 1;
			else 
				index_to_be_sent = 0;
    	}


    	/*  Prepare the inividual kilobot's message */
        tKilobotMessage_theta.m_sID = index_to_be_sent;
        double data_to_be_sent = state_vector[index_to_be_sent];

        // Signal check
        if(data_to_be_sent > 0)
        	tKilobotMessage_theta.m_sType = 0;
        else
        	tKilobotMessage_theta.m_sType = 1;

        // Next 3 digits
        data_to_be_sent /= 10.0;
        long next_3_digits = fabs(data_to_be_sent*pow(10,3));
        next_3_digits = next_3_digits % (int)pow(10,3);
        tKilobotMessage_theta.m_sData = next_3_digits;




        /*  Prepare the inividual kilobot's message */
        int index = index_to_be_sent + N;
        if(index >= 3*N)
            index -= 3*N;

        tKilobotMessage_x.m_sID = index;
        data_to_be_sent = state_vector[index];

        // Signal check
        if(data_to_be_sent > 0)
            tKilobotMessage_x.m_sType = 0;
        else
            tKilobotMessage_x.m_sType = 1;

        // Next 3 digits
        data_to_be_sent /= 10.0;
        next_3_digits = fabs(data_to_be_sent*pow(10,3));
        next_3_digits = next_3_digits % (int)pow(10,3);
        tKilobotMessage_x.m_sData = next_3_digits;




        /*  Prepare the inividual kilobot's message */
        index = index_to_be_sent + 2*N;
        if(index >= 3*N)
            index -= 3*N;
        tKilobotMessage_y.m_sID = index;
        data_to_be_sent = state_vector[index];


        // Signal check
        if(data_to_be_sent > 0)
            tKilobotMessage_y.m_sType = 0;
        else
            tKilobotMessage_y.m_sType = 1;

        // Next 3 digits
        data_to_be_sent /= 10.0;
        next_3_digits = fabs(data_to_be_sent*pow(10,3));
        next_3_digits = next_3_digits % (int)pow(10,3);
        tKilobotMessage_y.m_sData = next_3_digits;



        // The problem that arise here is the sign of the theta derivative. It depends on the omega parameter and its sign follow it accordingly.
        // One strategy would be to let the robot know it already from the beginning of the simulation, since one cannot send it all the time.

        /*  Set the message sending flag to True */
        bMessageToSend=true;

        tLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
    }

    /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
    if(bMessageToSend){
        for (int i = 0; i < 9; ++i) {
            m_tMessages[unKilobotID].data[i]=0;
        }

        m_tMessages[unKilobotID].type = 0;

        // Fill the kilobot message by the ARK-type messages
        for (int i = 0; i < 3; ++i) {

            if(i==0){
                tMessage=tKilobotMessage_theta;
            }else if(i==1){
                tMessage=tKilobotMessage_x;
            }else if(i==2){
                tMessage=tKilobotMessage_y;
            }

            m_tMessages[unKilobotID].data[i*3] = (tMessage.m_sID >> 2);
            m_tMessages[unKilobotID].data[1+i*3] = (tMessage.m_sID << 6);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sType << 2);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sData >> 8);
            m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
        }

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
