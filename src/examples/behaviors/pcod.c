/* Kilobot control software for the simple ALF experment : PCOD
 * author: Vander Luis de Souza Freitas (National Institute for Space Research, Brazil) vandercomp@gmail.com
 */

#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>

#include <math.h>

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

int left, right;
double omega0;

double d_theta=1.0;
double vr, vl;


double pi_over_4 = M_PI / 4.0;




/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;


int current_motion;



/* Variables for Smart Arena messages */
int sa_type = 0;
int sa_payload = 0;
bool new_sa_msg = false;


uint8_t t_left = 77;
uint8_t t_right = 77;
uint8_t forward_left = 48;
uint8_t forward_right = 48;


// Function to handle motion.
void set_motion(int new_motion)
{
   current_motion = new_motion;
     
   if (current_motion == STOP)
   {
      set_motors(0, 0);
   }
   else if (current_motion == FORWARD)
   {
      spinup_motors();
      set_motors(forward_left, forward_right);
   }
   else if (current_motion == LEFT)
   {
      spinup_motors();
      set_motors(t_left, 0);
   }
   else if (current_motion == RIGHT)
   {
      spinup_motors();
      set_motors(0, t_right);
   }
   
}



void set_omega(double omega){

	// (pi / 4) rad/s is the configured angular speed.
	// It means that when the robot is told to turn left, it will do so at (pi / 4) rad/s
	// The omega (argument) is the increment in the heading direction.
	// The robot must turn (left or right) for a certain period, until the increment is properly added.
	double angular_vel = (omega / pi_over_4);
	
	//int leftover;


	// After every turn, the particle must move straight for 10s.


	if(omega > 0){

		set_motion(LEFT);
		//if(kilo_uid == 0)
		//	printf("kilot_ticks=%d  Turning time=%f \n", kilo_ticks, angular_vel * 1000);
		delay(angular_vel * 1000);
		//leftover = 1000 - fabs(angular_vel) * 1000;

		set_motion(FORWARD);
		delay(10000*4);
		delay(10000*4);

		current_motion = LEFT;

	}else if( omega < 0){
		set_motion(RIGHT);
		delay(angular_vel * 1000);
		//leftover = 1000 - fabs(angular_vel) * 1000;

		set_motion(FORWARD);
		delay(10000*4);
		delay(10000*4);

		current_motion = RIGHT;

	}else{
		set_motion(current_motion);
		delay(angular_vel * 1000);

		//leftover = 1000 - fabs(angular_vel) * 1000;

		set_motion(FORWARD);
		delay(10000*4);
		delay(10000*4);

	}

	//printf("angular_vel=%f \n", angular_vel);

}





/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void rx_message(message_t *msg, distance_measurement_t *d) {

	//new_sa_msg = true;

	// unpack message
	sa_type = msg->data[1];  // Signal : positive 0, negative 1

	int temp=0;
	int num_digits=5;


	// Retrieving digits ----
	for(int i=2; i<num_digits+3; ++i){
		temp = temp * 10;
		temp += msg->data[i];
	}

	d_theta = (double)temp * pow(10,-(num_digits));
	
	// Check whether d_theta is negative, through the sa_type variable
	if(sa_type == 1)
		d_theta = d_theta * (-1.0);


	

	//printf("%f \n", d_theta);

	/*

	//if(sa_type == 1){
	//	printf("TYPE 1! \n");
	//	d_theta = d_theta * (1.0);
	//}

	

	// II) Considering (vr + vl) / 2 = v
	//vr = ( ( (0.033 * d_theta * dt  )  / 2.0 ) + v ) ;
	//vl = ( 2.0*v - ( ( (0.033 * d_theta * dt )  / 2.0 ) + v ) );
	

	// I) Considering vr + vl = v
	vr = ( (0.033 * d_theta * dt + v)  / 2.0  ) ;
	vl = ( v -  (0.033 * d_theta * dt + v )  / 2.0   ) ;

	
	// Update angular velocity
	

	// Negative d_theta
	if(sa_type == 1){
		left = (vr * speed); //kilo_turn_right; //kilo_straight_right;
		right = (vl * speed);
		d_theta = d_theta * (1.0);
	}else{
		left = (vl * speed); //kilo_turn_right; //kilo_straight_right;
		right = (vr * speed); // kilo_turn_left; // kilo_straight_left;
	}
	

	//printf("ticks: %d   d_th: %f  left: %d  vr: %d \n", kilo_ticks, d_theta, left, right);

	//if(abs(left - right) < 10)
	//printf("IN: %d , %d: dthetaaa=%f  l=%d  r=%d\n", kilo_uid, kilo_ticks, d_theta,left,right);

	spinup_motors();
	set_motors(left, right);

	new_sa_msg = false;*/

}





/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
	/* Initialise LED and motors */
	set_color(RGB(0,0,0));

	d_theta=0.0;

	set_motion(FORWARD);
	current_motion = FORWARD;
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
	set_omega(d_theta);
	//printf("omega=%f \n", d_theta); 
}


/*void loop() {
	//printf("OUT: %d , %d: dthetaaa=%f  l=%d  r=%d\n", kilo_uid, kilo_ticks, d_theta,left,right);
	//set_motors(left,right);

	//d_theta = 2.0;

	// I) Considering vr + vl = v
	//vl = ( (0.033 * d_theta * dt + v)  / 2.0  ) ;
	//vr = ( v -  (0.033 * d_theta * dt + v )  / 2.0   ) ;

	// II) Considering (vr + vl) / 2 = v
	//vr = ( ( (0.033 * d_theta * dt  )  / 2.0 ) + v ) ;
	//vl = ( 2.0*v - ( ( (0.033 * d_theta * dt )  / 2.0 ) + v ) );


	//d_theta = 1.0;

	// III) From Coursera - Control of Mobile Robots
	double R = 1.0; //  Wheel radius = 0.001..   kilobot diameter = 0.033.. Distance between legs = 0.025
	vr = (2.0*v + d_theta*dt * 0.025) / (2.0 * R);
	vl = (2.0*v - d_theta*dt * 0.025) / (2.0 * R);

	//vr *= 0.001;
	//vl *= 0.001;

	//vr = vr / 1000.0;
	//vl = vl / 1000.0;

	

	// The duty-cycle is assigned in the interval [0,255], which means that 1 percent equals DC=2.5.
	double factor = 25.5;

	uint8_t left = vl*speed;
	uint8_t right = vr*speed;


	printf("ticks: %d  d_theta=%f  vl=%f   vr=%f  m_l=%d  m_r=%d \n", kilo_ticks, d_theta, vl, vr, left, right);

	spinup_motors();

	//set_motors(left,right);
	set_motors(right,left); // I inverted because this is how this set_motors works
	
	//set_motors(vl*kilo_straight_right,vr*kilo_straight_left);
	//set_motors(vl*speed,vr*speed);
	//set_motors(vl,vr);
}*/

int main()
{
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);

    return 0;
}
