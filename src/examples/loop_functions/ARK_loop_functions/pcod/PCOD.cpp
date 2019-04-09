#include "PCOD.h"




// ------------- Random number generator -----------------------------------

// Uniform distribution
/*Minimal random number generator of Park and Miller with Bays-Durham shuffle and added safeguards. Returns a uniform random deviate between 0.0 and 1.0 (exclusive of the endpoint values). Call with idum a negative integer to initialize; thereafter, do not alter idum
between successive deviates in a sequence. RNMX should approximate the largest floating value that is less than 1.*/
float PCOD::ran1(long *idum_){


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

// Create the all-to-all projection matrix P = L / N, for L is the graph Laplacian.
void PCOD::projectionMatrix(){

	P = new double[N*N];

	for(int l=0; l<N; ++l){
		//std::cout << std::endl;
		for(int c=0; c<N; ++c){
			//std::cout << l << "  " << c << std::endl;
			if(l == c){
				*(P + l*N + c) = (double(N)-1.0) / double(N);
			}else{
				*(P + l*N + c) = -1.0 / double(N);
			}

			//std::cout << *(P + l*N + c) << "  ";
		}		
	}
}

complex<double> PCOD::scalar_product(double *v1, complex<double> *v2){
	complex<double> dot = 0.0;

	for(int i=0; i<N; ++i){
		dot += *(v1+i) * *(v2+i);
	}
	
	return dot;
}

// Get the line l of the Projection matrix
double *PCOD::matrix_line(double *matrix, int l){
	double *Pk = new double[N];


	for(int i=0; i<N; ++i)
		*(Pk+i) = *(matrix + l*N + i);

	return Pk;
}

// Compute the rotation center c_k of particle k
void PCOD::rotation_center(double *state){

	double r_x, r_y, theta;
	
	int index_c = 0;
	for(int i=0; i<NE; i+=3){

		theta = state[i+1];
		r_x = state[i+2];
		r_y = state[i+3];
		
		complex<double> rk(r_x,r_y);
		complex<double> vel(cos(theta),sin(theta));
		complex<double> ck( std::real(rk)- (std::imag(vel)/ particles[index_c].w), std::imag(rk) + ( std::real(vel) / particles[index_c].w));

		cc[index_c] = ck;

		++index_c;
	}
}

double PCOD::uk_circular_symmetric_paley_all_to_all(double state[], double theta, int id){

	double *Pk = matrix_line(P, id);

	complex<double> eitk(cos(theta), sin(theta));
	complex<double> Pk_c = scalar_product(Pk, cc);

	double potential_derivative = 0.0;
	double Km = 0.0;
	double theta_j=0.0;
	
	int m,j;	

	for(m=1; m<M+1; ++m){
		if(m == M)
			Km = K_M;
		else
			Km = K_m;
		
		for(j=1; j<=NE; j+=3){
			theta_j = state[j];
			potential_derivative += (Km/double(m)) * sin(double(m) * ( theta_j - theta ));	
		}		
	}


	potential_derivative = potential_derivative / double(N);

	double ukk = particles[id].w * (1.0 + K * std::real(std::conj(eitk) * Pk_c)) - potential_derivative;

	delete Pk;

	return ukk;
}
// ------------------------ END PCOD MODEL --------------------------------------









// --------- Numerical Integration -------------------------------
void PCOD::nrerror(char error_text[]){
	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);
	fprintf(stderr,"...now exiting to system...\n");
	exit(1);
}

void PCOD::derivs(double y[],double df[]){

	int index=0, i;

	rotation_center(y);

	for(i=1; i<=NE; i+=3){
		df[i]=uk_circular_symmetric_paley_all_to_all(y, y[i], index);
		df[i+1]=cos(y[i]);
		df[i+2]=sin(y[i]);

		++index;
	}	
}

double *PCOD::dvector(long nl,long nh){
	double *v;

	v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
	return v-nl+NR_END;
}

void PCOD::free_dvector(double *v, long nl, long nh){
	free((FREE_ARG) (v+nl-NR_END));
}
// --------------------------------------------------------------------



PCOD::PCOD(){

};


void PCOD::init(int N_, double M_, double omega0_, double h_){
	int i;	

	N = N_;
	M = M_;
	omega0 = omega0_;
	h = h_;

	NE = N_ * 3;
	
	K_m = 0.18;
	K_M = -0.02;

	K = 0.3;

	particles = new particle_[N];

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

	double box_side = 15.0; 

	cc = new complex<double>[N];
	projectionMatrix();
	d_theta = new double[N];

	
	// Initial conditions
	for(i=0; i<N; ++i){
		particles[i].id = i;
		particles[i].theta = ran1(&idum) * 2.0 * M_PI;
		particles[i].w = omega0;

		if(ran1(&idum) > 0.5)
			particles[i].r_x = ran1(&idum) * box_side;
		else
			particles[i].r_x = -ran1(&idum) * box_side;



		if(ran1(&idum) > 0.5)
			particles[i].r_y = ran1(&idum) * box_side;
		else
			particles[i].r_y = -ran1(&idum) * box_side;


		// Rotation center ck
		complex<double> rk(particles[i].r_x,particles[i].r_y);
		complex<double> vel(cos(particles[i].theta),sin(particles[i].theta));
		complex<double> ck( std::real(rk)- (std::imag(vel)/ particles[i].w), std::imag(rk) + ( std::real(vel) / particles[i].w));

		// Allocating memory
		cc[i] = ck;

		d_theta[i] = omega0;		
	}
}



PCOD::PCOD(int N_, double M_, double omega0_, double h_){
	init(N_, M_, omega0_, h_);
}


void PCOD::step_forward(){
	
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

		if(particles[index].theta > Pi_2)
			particles[index].theta -= Pi_2;
		else if(particles[index].theta < 0.0)
			particles[index].theta += Pi_2;

		++index;
	}		
	
	t=t+h;
	it++;
}



// Getters
double PCOD::get_h(){
	return h;
}


int PCOD::get_ticks(){
	return it;
}

double PCOD::get_t(){
	return t;
}



void PCOD::destroy(){

	// Free memmory
	delete P;
	delete cc;
	delete d_theta;
	free_dvector(y,1,NE+2);
	free_dvector(df,1,NE+2); 

	free_dvector(x,1,NE+1);
	free_dvector(a,1,NE+2);
	free_dvector(b,1,NE+2);
	free_dvector(c,1,NE+2);
}





