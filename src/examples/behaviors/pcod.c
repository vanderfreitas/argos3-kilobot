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





// Attempt 2: Book "Difital control of Dynamic Systems"
// Page 66, section 3.3 PID control, equation 3.17.
double e_1;
double e_2;
double T_D=0.0; //0.0008;
double T_I=0.9;// 0.003;
double T;
double K=0.5; //0.3;
double uk_1;


double dt;
double e;
double uk;



double new_e; 
double last_rcvd_e;


double d_theta=1.0;

double t=0.0;

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
void set_motion(int new_motion){

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
		//e = (double)sa_type; 

        new_e = (double)sa_payload  * pow(10,-3);

		if((double)sa_type == 1)
			new_e *= -1.0;

		new_e = new_e*10.0;
		

		//printf("id=%d  kilo_ticks=%d  t=%f  e=%f  vr=%f   vl=%f\n", kilo_uid, kilo_ticks,t,e,vr,vl);

    }
}


/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
	/* Initialise LED and motors */
	set_color(RGB(0,0,0));

	dt = 0.01; //0.3226;

	new_e = 0.0;
	last_rcvd_e = 0.0;

	t = 0.0; // dt;

	set_motion(FORWARD);


	// Approach do livro
	T = dt;
	e_1 = 0.0;
	e_2 = 0.0;
	e = 0.0;
	uk_1 = 0.0;
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
	

	if(new_e != last_rcvd_e){
		last_rcvd_e = new_e;

		e_2 = e_1;
		e_1 = e;
		e = new_e;
	}else{
		e_2 = e_1;
		e_1 = e;
		e -= uk;
	}

	uk_1 = uk;

	uk = uk_1  + K* (  (1.0 + T/T_I + T_D/T) * e - (1.0 + 2.0*(T_D/T)) * e_1  + (T_D/ T) * e_2 );
	


	double angular_vel = fabs(uk / pi_over_4);
	printf("id=%d  kilo_ticks=%d  e=%f  w=%f   ang_v=%f\n", kilo_uid, kilo_ticks,e,uk,angular_vel);

	angular_vel = fabs(angular_vel);
	if(angular_vel > 1.0){
		uk = uk / pi_over_4;
		angular_vel = 1.0;
	}


	
	if(fabs(uk) < 0.000001){
		set_motion(FORWARD);
	}else if(uk > 0){

		set_motion(LEFT);
		delay(angular_vel*1000);
		set_motion(FORWARD);
		delay((1.0-angular_vel)*1000);
	}else{

		set_motion(RIGHT);
		delay(angular_vel*1000);
		set_motion(FORWARD);
		delay((1.0-angular_vel)*1000);
	}

	

}


int main()
{
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);

    return 0;
}
