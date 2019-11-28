/* Kilobot control software for the simple ALF experment : PCOD
 * author: Vander Luis de Souza Freitas (National Institute for Space Research, Brazil) vandercomp@gmail.com
 */

#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>

#include <complex.h>
#include <math.h>







// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------- PCOD code ------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------


// -- Random number -----------
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1+(IM-1)/NTAB)
#define EPS 1.2e-7
#define RNMX (1.0-EPS)
#define PI acos(-1.0)
// ----------------------------


#define NR_END 1
#define FREE_ARG char*


#define Pi_2  2.0*M_PI


struct particle_{
	double r_x; // coordinates
	double r_y;
	double theta; // Heading angle and phase
	double w; // natural frequency
	int id;
};


long idum = -98667;
struct particle_ *particles;
double *d_theta;
double complex *cc;

double v=0.00997009; 


// ----------- RK4 attributes ------------------------------
double h=0.1;
double t;
int it;
double *x, *a, *b, *c,*df,*y;
//----------------------------------------------------------

double *L;

// ----- PCOD model parameters ------------
int N = 6; // Number of robots
double M = 2;  // Number of clusters
int NE;
double KK = 0.1;
double omega0 = 0.05;
double d_max = 2.0;


double K_m = 0.6; //0.1 * N;
double K_M = -0.7; //-0.1 * (N+1);


// ------------- Random number generator -----------------------------------

// Uniform distribution
/*Minimal random number generator of Park and Miller with Bays-Durham shuffle and added safeguards. Returns a uniform random deviate between 0.0 and 1.0 (exclusive of the endpoint values). Call with idum a negative integer to initialize; thereafter, do not alter idum
between successive deviates in a sequence. RNMX should approximate the largest floating value that is less than 1.*/
float ran1(long *idum_){


	int j;
	long k;
	static long iy=0;
	static long iv[NTAB];
	float temp;

	if(*idum_<=0 || !iy){
		if(-(*idum_)<1) 
			*idum_=1;
		else 
			*idum_ = -(*idum_);
		for(j=NTAB+7;j>=0;j--){
			k=(*idum_)/IQ;
			*idum_=IA*(*idum_-k*IQ)-IR*k;
			if(*idum_<0) 
				*idum_ +=IM;
			if(j<NTAB) 
				iv[j]=*idum_;
		}
		iy=iv[0];
	}
	k=(*idum_)/IQ;
	*idum_=IA*(*idum_-k*IQ)-IR*k;
	if(*idum_<0) 
		*idum_ += IM;
	j=iy/NDIV;
	iy=iv[j];
	iv[j]=*idum_;
	if((temp=AM*iy)>RNMX) 
		return RNMX;
	else 
		return temp;
}


// ------------------- PCOD MODEL ---------------------------------------------

// ------ Ring topology ------
void graph_laplacian(){

	L = malloc (N*N * sizeof (double));

	for(int l=0; l<N; ++l){
		for(int c=0; c<N; ++c){
			*(L + l*N + c) = 0.0;
		}

		*(L + l*N + l) = 2.0;

		if(l > 0){
			*(L + l*N + l-1) = -1.0;
			if(l==N-1){
				*(L + l*N) = -1.0;
			}else
				*(L + l*N + l+1) = -1.0;
		
		}else if(l==0){
			*(L + l*N + N-1) = -1.0;
			*(L + l*N + l+1) = -1.0;
		}
	}
}


double complex scalar_product(double *v1, double complex *v2){
	double complex dot = 0.0;

	for(int i=0; i<N; ++i){
		dot += *(v1+i) * *(v2+i);
	}
	
	return dot;
}

// Get the line l of the Projection matrix
double *matrix_line(double *matrix, int l){
	double *Pk = malloc (N * sizeof (double));


	for(int i=0; i<N; ++i)
		*(Pk+i) = *(matrix + l*N + i);

	return Pk;
}

// Compute the rotation center c_k of particle k
void rotation_center(double *state){

	double r_x, r_y, theta;
	
	int index_c = 0;
	for(int i=0; i<NE; i+=3){

		theta = state[i+1];
		r_x = state[i+2];
		r_y = state[i+3];
		
		double complex rk = r_x + r_y * I;
		double complex vel = cos(theta) + sin(theta) * I;
		double complex ck =  creal(rk)- ( (cimag(vel) * v) / (particles[index_c].w ))  + ( cimag(rk) + ( (creal(vel) * v) / (particles[index_c].w ) )) * I;

		cc[index_c] = ck;

		++index_c;
	}

}

double uk_paley_limmited(double state[], double theta, int id){

	double *Lk = matrix_line(L, id);

	double complex eitk = cos(theta) + sin(theta) * I;
	double complex Lk_c = scalar_product(Lk, cc);

	double potential_derivative = 0.0;
	double Km = 0.0;
	double theta_j=0.0;
	
	double complex *eimt = malloc (N * sizeof (double complex));  // new complex<double>[N];
	
	double complex Lk_eimt;

	int index, m, j;

	for(m=1; m<M+1; ++m){
		if(m == M)
			Km = K_M;
		else
			Km = K_m;

		index = 0;
		for(j=1; j<=NE; j+=3){
			theta_j = state[j];
			double complex aux = cos((double)m*theta_j) + sin((double)m*theta_j) * I;
			eimt[index] = aux;
			++index;
		}
		

		double complex ieimtk = I * ( cos((double)m*theta) + sin((double)m*theta) * I );   //-sin((double)m*theta) + cos( (double)m*theta) * I;
		Lk_eimt = scalar_product(Lk, eimt);

		potential_derivative += (Km/(double)m) * creal(conj(ieimtk) * Lk_eimt);
	}


	double ukk = omega0 * (1.0 + (KK/d_max) * creal(conj(eitk) * Lk_c)) + potential_derivative;

	free(Lk);

	return ukk;
}


// ------------------------ END PCOD MODEL --------------------------------------








// --------- Numerical Integration -------------------------------
void nrerror(char error_text[]){
	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);
	fprintf(stderr,"...now exiting to system...\n");
	exit(1);
}

void derivs(double y[],double df[]){

	int index=0, i;

	rotation_center(y);

	for(i=1; i<=NE; i+=3){
		df[i]=uk_paley_limmited(y, y[i], index); 

		df[i+1]=v*cos(y[i]);
		df[i+2]=v*sin(y[i]);

		++index;
	}
}

double *dvector(long nl,long nh){
	double *v;

	v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
	return v-nl+NR_END;
}

void free_dvector(double *v, long nl, long nh){
	free((FREE_ARG) (v+nl-NR_END));
}
// --------------------------------------------------------------------



void init(int N_, double M_, double omega0_, double h_){
	
	int i;	

	N = N_;
	M = M_;
	omega0 = omega0_;
	h = h_;

	NE = N_ * 3;
	

	K_m = 0.02;
	K_M = -K_m * ((double)M+1);

	KK = 50.0;

	particles = malloc (N * sizeof (struct particle_));

	y=dvector(1,NE+2);
	df=dvector(1,NE+2);

	x=dvector(1,NE+1);
	a=dvector(1,NE+2);
	b=dvector(1,NE+2);
	c=dvector(1,NE+2);

	for(i=1; i<=NE; ++i)
		x[i] = 1;


	t=0.0;
	it=0;

	//double box_side = 15.0; 

	cc = malloc (N * sizeof (double complex));
	graph_laplacian();
	d_theta = malloc (N * sizeof (double));


	for(i=0; i<N; ++i){
		particles[i].id = i;
		particles[i].theta = 0.0;
		particles[i].w = omega0;

		particles[i].r_x = 0.0; 
		particles[i].r_y = 0.0;
		cc[i] = 0.0;

		d_theta[i] = omega0;		
	}

}



void step_forward(){
	
	int j,i;
	
	int index = 0;
	for(i=1; i<=NE; i+=3){
		x[i]=particles[index].theta;
		x[i+1]=particles[index].r_x;	
		x[i+2]=particles[index].r_y;

		++index;
	}

	for(j=1;j<=NE;j++)
			y[j]=x[j];

	derivs(y,df); 

	// k1 is a
	for(j=1;j<=NE;j++){
		a[j]=h*df[j];
		y[j]=x[j]+a[j]/2.0;
	}

	derivs(y,df);

	// k2 is b
	for(j=1;j<=NE;j++){
		b[j]=h*df[j];
		y[j]=x[j]+b[j]/2.0;
	}

	derivs(y,df);

	// k3 is c
	for(j=1;j<=NE;j++){
		c[j]=h*df[j];
		y[j]=x[j]+c[j];
	}

	derivs(y,df);

	// k4 is h*df[j]
	for(j=1;j<=NE;j++)
		x[j]=x[j]+(a[j]+h*df[j])/6.0+(b[j]+c[j])/3.0;

	index = 0;
	for(i=1; i<=NE; i+=3){

		particles[index].theta = x[i];
		particles[index].r_x = x[i+1];
		particles[index].r_y = x[i+2];

		// Updating the derivative output for mobile robots
		d_theta[index] = (  (a[i]+h*df[i])/6.0+(b[i]+c[i])/3.0  ) / h;

		particles[index].theta = atan2(sin(particles[index].theta),cos(particles[index].theta));

		++index;
	}		
	
	t=t+h;
	it++;
}



void destroy(){

	// Free memmory
	free(L);
	free(cc);
	free(d_theta);
	free_dvector(y,1,NE+2);
	free_dvector(df,1,NE+2); 

	free_dvector(x,1,NE+1);
	free_dvector(a,1,NE+2);
	free_dvector(b,1,NE+2);
	free_dvector(c,1,NE+2);
}


// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------- PCOD code ------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------



















































#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3



// PÌD regulator
double keep_angle_in_interval(double phi){
	return atan2(sin(phi), cos(phi) );
}

double two_pi = 2.0*M_PI;
double half_pi = M_PI / 2.0;

double *state_vector; 
int sv_id;







// Book: G. F. Franklin, M. L. Workman, and D. Powell, Digital Control of Dynamic Systems, 3rd ed. Boston, MA, USA: Addison-Wesley Longman Publishing Co., Inc., 1997.
// Page 66, section 3.3 PID control, equation 3.17.
double e_1;
double e_2;
double T_D=0.0001; //0.005; //0.0008;
double T_I=5.0; //0.9;// 0.003;
double T;
double K=0.1; //0.5; //0.3;
double uk_1;


 


double dt;
double e;
double uk;



double new_e; 
double last_rcvd_e;


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
	//int index;


	// id1, id2 and id3 relate to state_vector positions

	if (msg->type == 0) {
        // unpack message
        int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
        int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
        int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);


        // ------------------ THETA ------------------------
        // unpack type
        sa_type = msg->data[1] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]);
        new_sa_msg = true;

        state_vector[id1] = (double)sa_payload  * pow(10,-3);
		if((double)sa_type == 1)
			state_vector[id1] *= -1.0;
		state_vector[id1] = state_vector[id1]*10.0;

		// Error
		if(id1 == kilo_uid){
			new_e = keep_angle_in_interval(particles[kilo_uid].theta - state_vector[id1]);

		}

		// ------------------ X ------------------------
        // unpack type
        sa_type = msg->data[4] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);

        state_vector[id2] = (double)sa_payload  * pow(10,-3);
		if((double)sa_type == 1)
			state_vector[id2] *= -1.0;
		state_vector[id2] = state_vector[id2]*10.0;
		particles[id1].r_x = state_vector[id2];



		// ------------------ Y ------------------------
        // unpack type
        sa_type = msg->data[7] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);

        state_vector[id3] = (double)sa_payload  * pow(10,-3);
		if((double)sa_type == 1)
			state_vector[id3] *= -1.0;
		state_vector[id3] = state_vector[id3]*10.0;
		particles[id1].r_y = state_vector[id3];

    }

}


/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
	sv_id = 0;

	init(N, M, 0.5, 0.1);
	//printf("Theta antes: %f\n", particles[kilo_uid].theta);


	/* Initialise LED and motors */
	set_color(RGB(0,0,0));

	state_vector = malloc (3*N * sizeof (double)); 
	for(int i=0; i<3*N; ++i)
		state_vector[i] = 0.0;

	dt = 0.5; //0.1;

	new_e = 0.0;
	last_rcvd_e = 0.0;

	t = 0.0;

	set_motion(STOP);

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


	// First 500 ticks to receive robot states
	if(kilo_ticks > 500){

		// Integrate the pcod model
		step_forward();


		if(KK > 10.0)
			KK -= 1.0;

		// Compute error between state and dynamics ------------
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

		// Book: G. F. Franklin, M. L. Workman, and D. Powell, Digital Control of Dynamic Systems, 3rd ed. Boston, MA, USA: Addison-Wesley Longman Publishing Co., Inc., 1997.
		// Page 66, section 3.3 PID control, equation 3.17.
		uk = uk_1  + K* (  (1.0 + T/T_I + T_D/T) * e - (1.0 + 2.0*(T_D/T)) * e_1  + (T_D/ T) * e_2 );
		

		double angular_vel = uk / pi_over_4;

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

}


int main()
{
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);

    return 0;
}
