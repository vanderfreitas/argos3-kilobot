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
//#define NE 3


using namespace std;









class PCOD{

	
public:
	struct particle_{
		double r_x;
		double r_y;
		double c_x;
		double c_y;
		double theta;
		double w;
		int id;
	};


	float ran1(long *idum_);

	void init(int N_, double M_, double omega0_, double h_);
	void step_forward();
	void destroy();

	int get_ticks();


	particle_ *particles;
	std::vector<double> d_theta;

	double K = 0.1;

	

private:
	


	// Numerical integration
	void nrerror(char error_text[]);
	void derivs(double *y,double *df, particle_ *particles);
	double *dvector(long nl,long nh);
	void free_dvector(double *v, long nl, long nh);


	// Model
	void projectionMatrix();
	complex<double> scalar_product(double *v1, complex<double> *v2);
	double *matrix_line(double *matrix, int l);
	void rotation_center(double *state, particle_ *particles);
	double distance_particles(particle_ *particles, int id1, int id2);

	double uk_circular_symmetric_paley_all_to_all(particle_ *particles, double *state, double theta, int id);
	//double uk_Kuramoto(std::vector<particle_> particles, double theta, int id);


	

	// Random number generation Seed
	long idum{-986967};


	// --- Runge-Kutta 4th order --------
	double h=0.01;
	double tf = 2000.0;

	double t;
	int it;

	
	double *x, *a, *b, *c,*df,*y;
	double Pi_2 = 2.0*M_PI; 
	//-----------------------------------


	// ----- Model parameters ------------
	int N = 12;
	double M = 3;

	int NE = 36;

	double K_m = 0.18;
	double K_M = -0.02;

	
	double K0 = 0.1;
	double omega0 = 0.05; //0.05;

	double *P;
	complex<double> *cc;

	// -----------------------------------

};



