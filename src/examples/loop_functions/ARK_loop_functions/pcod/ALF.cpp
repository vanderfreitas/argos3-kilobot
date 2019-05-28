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


// PÌD regulator
double keep_angle_in_interval(double phi){
	return atan2(sin(phi), cos(phi) );
}




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

	//if((int)m_fTimeInSeconds % 100 == 0){  // update model at each 10s.
 		

	//CVector2 cKilobotPosition0=GetKilobotPosition(*m_tKilobotsEntities[0]);
    //CVector2 cKilobotPosition1=GetKilobotPosition(*m_tKilobotsEntities[1]);
    //CVector2 cKilobotPosition2=GetKilobotPosition(*m_tKilobotsEntities[2]);

	//printf("time pcod=%f   time kilobot=%f   r1=(%f,%f)  r1_kilo=(%f,%f) \n", pcod_model.get_t(),m_fTimeInSeconds, 
     //                                            pcod_model.particles[0].r_x, pcod_model.particles[0].r_y, 
    //                                             cKilobotPosition0.GetX(), cKilobotPosition0.GetY());


	//printf("time pcod=%f   time kilobot=%f  r1_kilo_ori=(%f,%f) \n", pcod_model.get_t(),m_fTimeInSeconds,cKilobotPosition0.GetX(), cKilobotPosition0.GetY());
	



    pcod_model.step_forward();


        

        //printf("time pcod=%f   time kilobot=%f   c1=(%f,%f)  c2=(%f,%f)  c3=(%f,%f)\n", pcod_model.get_t(),m_fTimeInSeconds, 
        //                                                 std::real(pcod_model.cc[0]), std::imag(pcod_model.cc[0]), 
        //                                                 std::real(pcod_model.cc[1]), std::imag(pcod_model.cc[1]),
        //                                                 std::real(pcod_model.cc[2]), std::imag(pcod_model.cc[2]));


        CVector2 cKilobotPosition;
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){    
        // Heading angle transformation is necessary
        if(pcod_model.particles[it].theta < 0.0)
            pcod_model.particles[it].theta += two_pi;
        else if(pcod_model.particles[it].theta > two_pi)
            pcod_model.particles[it].theta -= two_pi;


        cKilobotPosition=GetKilobotPosition(*m_tKilobotsEntities[it]);

        //pcod_model.particles[it].r_x = cKilobotPosition.GetX();
        //pcod_model.particles[it].r_y = cKilobotPosition.GetY();

    	
        double theta = GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue();
        double v=0.00997009;
		complex<double> rk(cKilobotPosition.GetX(),cKilobotPosition.GetY());
		complex<double> vel(cos(theta),sin(theta));
		complex<double> ck( std::real(rk) - ( (std::imag(vel) * v) / (pcod_model.particles[it].w )), std::imag(rk) + ( (std::real(vel) * v) / (pcod_model.particles[it].w ) ));
	    

		printf("it=%d  time pcod=%f   time kilobot=%f   c1=(%f,%f)  c1_kilo=(%f,%f) \n", it, pcod_model.get_t(),m_fTimeInSeconds, 
	                                                 std::real(pcod_model.cc[it]), std::imag(pcod_model.cc[it]), 
	                                                 std::real(ck), std::imag(ck));
		
    }


        


    //}

    /* Update the virtual sensor of the kilobots*/
    UpdateVirtualSensors();
}


void CALF::PostStep(){

	
	/*if(pcod_model.get_ticks() % 10 == 0){
		for(int i=0; i<N; ++i)
			m_cOutput << m_fTimeInSeconds << "\t" << i << "\t" << pcod_model.particles[i].r_x << "\t" << pcod_model.particles[i].r_y << "\t" << pcod_model.particles[i].theta << "\n";
	}*/



//    /*  experiment's results*/
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
	//dt = dt / 10.0;

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
    	pcod_model.particles[it].r_x = cKilobotPosition.GetX();
    	pcod_model.particles[it].r_y = cKilobotPosition.GetY(); //-cKilobotPosition.GetY();
    	pcod_model.particles[it].theta = GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue() + 0.3*M_PI; // + half_pi; // + 0.3 * M_PI;
        //pcod_model.particles[it].theta = M_PI - 0.1;

    	if(pcod_model.particles[it].theta < 0.0)
    		pcod_model.particles[it].theta += two_pi;
    	else if(pcod_model.particles[it].theta > two_pi)
    		pcod_model.particles[it].theta -= two_pi;
    }
	printf("\n");



	// PÌD regulator initialization
    pid_reg = new pid_regulator[N];
    for(int i=0; i<N; ++i){
    	pid_reg[i].e_d = 0.0;
    	pid_reg[i].e = 0.0;
    	pid_reg[i].e_i = 0.0;
    	pid_reg[i].dt = dt;
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

    /* Get the model parameters */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "omega0", omega0, omega0);

    GetNodeAttributeOrDefault(tExperimentVariablesNode, "M", M, M);


    

}

/****************************************/
/*          Updating functions          */
/****************************************/

void CALF::UpdateVirtualSensors(){

	// One must receive the sensor readings and compare with the model's in order to correctly update the system.
	// PID controller.

	//double reference = pcod_model.particles[it].theta

    //double inc = pid.calculate(0, reference);




	// Update the theta vector
	for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        double pcod_model_bounded_theta = keep_angle_in_interval(pcod_model.particles[it].theta);
        double argos_bounded_theta = keep_angle_in_interval(GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue());

		//pid_reg[it].former_e = pid_reg[it].e;
		pid_reg[it].e = keep_angle_in_interval(pcod_model_bounded_theta - argos_bounded_theta);
		//pid_reg[it].e = keep_angle_in_interval(pid_reg[it].e);


		//if(it == 0){
			//printf("ALF:  theta=%f  theta_r=%f \n", keep_angle_in_interval(pcod_model.particles[it].theta), keep_angle_in_interval(GetKilobotOrientation(*m_tKilobotsEntities[it]).GetValue() + half_pi));
			printf("%d\t%f\t%f\n", GetSpace().GetSimulationClock(), pcod_model_bounded_theta, argos_bounded_theta);
		//}

        

		//pcod_model.particles[it].theta = keep_angle_in_interval(pcod_model.particles[it].theta);

		// Coordinate transformation is necessary
		//CVector2 cKilobotPosition;
		//cKilobotPosition=GetKilobotPosition(*m_tKilobotsEntities[it]);
		//pcod_model.particles[it].r_x = -cKilobotPosition.GetY();
		//pcod_model.particles[it].r_y = cKilobotPosition.GetX();
	}


	for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
		/* Update the virtual sensor of a kilobot based on its current state */
		UpdateVirtualSensor(*m_tKilobotsEntities[it]);
	}
}



UInt8 convert_double_to_UInt8_(double angle, int num_digits){
    long temp= fabs(angle*pow(10,num_digits-1));

    if(temp > 1023)
        temp = 1023;

    return temp;
}



void CALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){

    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;

    /* Flag for existance of message to send*/
    bool bMessageToSend=false;

    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);

    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - tLastTimeMessaged[unKilobotID]< MinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }
    else{
		// Error derivative
		//pid_reg[unKilobotID].e_d = (pid_reg[unKilobotID].e - pid_reg[unKilobotID].former_e)*pid_reg[unKilobotID].dt;
		// Error integral
		//pid_reg[unKilobotID].e_i += pid_reg[unKilobotID].e*pid_reg[unKilobotID].dt;
		// PID regulator
		//pid_reg[unKilobotID].omega = pid_reg[unKilobotID].Kp*pid_reg[unKilobotID].e 
		//                           + pid_reg[unKilobotID].Ki*pid_reg[unKilobotID].e_i 
		//                           + pid_reg[unKilobotID].Kd*pid_reg[unKilobotID].e_d;

        /*  Prepare the inividual kilobot's message */
        tKilobotMessage.m_sID = unKilobotID;

        // Convert the angle derivative (double) into a integer of 4 digits. 
        // Since the payload is composed of one type of 4 bits and a payload (data) of 10 bits, 
        // we pack one digit in the type and the other 3 in the payload

        // First digit
        //tKilobotMessage.m_sType = fabs(pid_reg[unKilobotID].e*pow(10,0)); //fabs(pid_reg[unKilobotID].omega*pow(10,0)); 




        if(pid_reg[unKilobotID].e > 0)
        	tKilobotMessage.m_sType = 0;
        else
        	tKilobotMessage.m_sType = 1;



        // Next 3 digits
        pid_reg[unKilobotID].e /= 10.0;
        long next_3_digits = fabs(pid_reg[unKilobotID].e*pow(10,3));
        next_3_digits = next_3_digits % (int)pow(10,3);
        tKilobotMessage.m_sData = next_3_digits;



        // The problem that arise here is the sign of the theta derivative. It depends on the omega parameter and its sign follow it accordingly.
        // One strategy would be to let the robot know it already from the beginning of the simulation, since one cannot send it all the time.


        //printf("time=%f  e=%f \n",m_fTimeInSeconds,pid_reg[unKilobotID].e);

        /*  Set the message sending flag to True */
        bMessageToSend=true;

        tLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
    }

    /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
    if(bMessageToSend){

        //printf("SEND: %f\n",pid_reg[unKilobotID].e);

        for (int i = 0; i < 9; ++i) {
            m_tMessages[unKilobotID].data[i]=0;
        }

        m_tMessages[unKilobotID].type = 0;
        // Prepare an empty ARK-type message to fill the gap in the full kilobot message
        tEmptyMessage.m_sID=1023;
        tEmptyMessage.m_sType=0;
        tEmptyMessage.m_sData=0;

        // Fill the kilobot message by the ARK-type messages
        for (int i = 0; i < 3; ++i) {

            if(i==0){
                tMessage=tKilobotMessage;
            } else{
                tMessage=tEmptyMessage;
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
