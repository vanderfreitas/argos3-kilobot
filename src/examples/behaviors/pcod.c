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
double omega0 = 1.0;

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


/*
void set_omega(double omega){

	// (pi / 4) rad/s is the configured angular speed.
	// It means that when the robot is told to turn left, it will do so at (pi / 4) rad/s
	// The omega (argument) is the increment in the heading direction.
	// The robot must turn (left or right) for a certain period, until the increment is properly added.
	double angular_vel = (omega / pi_over_4);

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

}*/


void set_omega(double omega){

	// (pi / 4) rad/s is the configured angular speed.
	// It means that when the robot is told to turn left, it will do so at (pi / 4) rad/s
	// The omega (argument) is the increment in the heading direction.
	// The robot must turn (left or right) for a certain period, until the increment is properly added.
	double angular_vel = fabs(omega / pi_over_4);

	// After every turn, the particle must move straight for 10s.

	if(omega > 0){

		set_motion(LEFT);
		delay(angular_vel * 1000);

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

}


/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void rx_message(message_t *msg, distance_measurement_t *d) {
	if (msg->type == 0) {
        // unpack message
        int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
        int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
        int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);
        if (id1 == kilo_uid) {
            // unpack type
            sa_type = msg->data[1] >> 2 & 0x0F;
            // unpack payload
            sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]);
            new_sa_msg = true;
        }
        if (id2 == kilo_uid) {
            // unpack type
            sa_type = msg->data[4] >> 2 & 0x0F;
            // unpack payload
            sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
            new_sa_msg = true;
        }
        if (id3 == kilo_uid) {
            // unpack type
            sa_type = msg->data[7] >> 2 & 0x0F;
            // unpack payload
            sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
            new_sa_msg = true;
        }

        // d_theta will receive 4 digits in total.
        // The first digit comes from sa_type and the remaining 3 from sa_payload.
		d_theta = (double)sa_type; 
		d_theta += (double)sa_payload  * pow(10,-3);

		if(omega0 < 0)
			d_theta *= -1.0;
		
		if(kilo_uid == 0)
			printf("PCOD: %f  ticks: %d\n", d_theta, kilo_ticks);
    }
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
	//printf("LOOP %d\n", kilo_ticks);

	set_omega(d_theta);
	//set_motion(FORWARD);

}


int main()
{
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);

    return 0;
}
