#include<math.h>
#include<stdio.h>
#include<stdlib.h>

#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <math.h>

#include <string.h>

#include <numeric>
#include <complex>
#include <cmath>
#include <iomanip>



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

using namespace std;



class particle_{

public:
	double r_x; // coordinates
	double r_y;
	double theta; // Heading angle and phase
	double w; // natural frequency
	int id;

};







class PCOD{

	
public:

	// PCOD class constructor
	PCOD(int N_, double M_, double omega0_, double h_);
	PCOD();

	// Initializer
	void init(int N_, double M_, double omega0_, double h_);

	// Random number generator
	float ran1(long *idum_);
	// Random number generator Seed
	long idum{-98667};

	// Integrate model for time dt
	void step_forward();

	// Finished the model and deallocate all structures
	void destroy();

	// Each tick corresponds to a period of time dt
	int get_ticks();

	// Current time (seconds)
	double get_t();

	// Integration time dt
	double get_h();


	// Public access attributes --- Ok, It is not a good idea, but let's do it for now =)
	particle_ *particles;
	double *d_theta;
	complex<double> *cc;

	double v=0.00997009; //  0.1; //  0.009985; //0.00302;


protected:

	// ---------- RK4 methods from Numerical Recipes ------------
	void nrerror(char error_text[]);
	void derivs(double y[],double df[]);
	double *dvector(long nl,long nh);
	void free_dvector(double *v, long nl, long nh);
	// --------------------------------------------------

	// ----------- RK4 attributes ------------------------------
	double h=0.1;
	double tf = 5000.0;
	double t;
	int it;
	double *x, *a, *b, *c,*df,*y;
	//----------------------------------------------------------
	



	// ---------- PCOD model methods ------------
	void projectionMatrix();
	complex<double> scalar_product(double *v1, complex<double> *v2);
	double *matrix_line(double *matrix, int l);
	void rotation_center(double *state);
	double uk_circular_symmetric_paley_all_to_all(double state[], double theta, int id);
	// ------------------------------------------

	// ----------- PCOD model attributes -----------
	double *P;
	
	// --------------------------------------------



	// ----- PCOD model parameters ------------
	int N = 12;
	double M = 3;
	int NE = 36;
	double K_m = 0.18;
	double K_M = -0.02;
	double K = 0.1;
	double K0 = 0.1;
	double omega0 = 0.05;
};



