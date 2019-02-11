#include "PCOD.h"



double v = 0.01;


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



// ------------------- MODEL ---------------------------
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

double *PCOD::matrix_line(double *matrix, int l){
	double *Pk = new double[N];


	for(int i=0; i<N; ++i)
		*(Pk+i) = *(matrix + l*N + i);

	return Pk;
}



void PCOD::rotation_center(double *state, particle_ *particles){

	double r_x, r_y, theta;
	
	int index_c = 0;
	for(int i=0; i<NE; i+=3){

		theta = state[i+1];
		r_x = state[i+2];
		r_y = state[i+3];
		
		complex<double> rk(r_x,r_y);
		//std::cout << "rk: " << rk << std::endl;
		complex<double> vel(cos(theta),sin(theta));
		//std::cout << "vel: " << vel << std::endl;
		complex<double> ck( std::real(rk)- (std::imag(vel)/ particles[index_c].w), std::imag(rk) + ( std::real(vel) / particles[index_c].w));
		//std::cout << "ck: " << ck << std::endl;

		cc[index_c] = ck;

		++index_c;
	}
}


double PCOD::distance_particles(particle_ *particles, int id1, int id2){
	double d = 0.0;
	
	d = sqrt( (particles[id1].r_x - particles[id2].r_x) * (particles[id1].r_x - particles[id2].r_x) + 
	    (particles[id1].r_y - particles[id2].r_y) * (particles[id1].r_y - particles[id2].r_y) );

	return d;
}



/*double PCOD::uk_Kuramoto(std::vector<particle_> particles, double theta, int id){
	
	double potential_derivative = 0.0;
	double theta_j=0.0;
	
	for(int j=0; j<N; ++j){
		theta_j = particles[j].theta;
		potential_derivative += sin( theta_j - theta );	
	}		

	potential_derivative = potential_derivative / double(N);

	double ukk = particles[id].w - potential_derivative;

	return ukk;
}*/



double PCOD::uk_circular_symmetric_paley_all_to_all(particle_ *particles, double *state, double theta, int id){

	double *Pk = matrix_line(P, id);

	complex<double> eitk(cos(theta), sin(theta));
	complex<double> Pk_c = scalar_product(Pk, cc);

	double potential_derivative = 0.0;
	double Km = 0.0;
	double theta_j=0.0;
	
	for(int m=1; m<M+1; ++m){
		if(m == M)
			Km = K_M;
		else
			Km = K_m;
		
		for(int j=1; j<=NE; j+=3){
			theta_j = state[j];
			potential_derivative += (Km/double(m)) * sin(double(m) * ( theta_j - theta ));	
		}		
	}


	potential_derivative = potential_derivative / double(N);

	double ukk = particles[id].w * (1.0 + K * std::real(std::conj(eitk) * Pk_c)) - potential_derivative;

	delete Pk;

	return ukk;
}




// ------------------------ END MODEL --------------------------------------







// --------- Numerical Integration -------------------------------
void PCOD::nrerror(char error_text[]){
	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);
	fprintf(stderr,"...now exiting to system...\n");
	exit(1);
}



void PCOD::derivs(double *y, double *df, particle_ *particles){
	// Rotation centers
	rotation_center(y, particles);

	int index=0;
	for(int i=1; i<=NE; i+=3){
		df[i]=uk_circular_symmetric_paley_all_to_all(particles, y, y[i], index);
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


int PCOD::get_ticks(){
	return it;
}



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

	particles = new particle_[N]; // (particle_ *)malloc((size_t) ((N_)*sizeof(particle_)));

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

	double box_side = 15.0; //4.0*sqrt(double(N)*M_PI); //1.0/(3.0*omega0);

	cc = new complex<double>[N];
	projectionMatrix();
	
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

		// It is not correct, but it does not matter at all, since it will be checked only in the first dt.
		particles[i].c_x = particles[i].r_x;
		particles[i].c_y = particles[i].r_y;


		d_theta.push_back(omega0);		

		//std::cout << "id: " << i << "  x: " << particle.r_x << "  y: " << particle.r_y << std::endl;
	}
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
	   
	derivs(y,df, particles);

	for(j=1;j<=NE;j++){
		a[j]=h*df[j];
		y[j]=x[j]+a[j]/2.0;
	}

	derivs(y,df, particles);

	for(j=1;j<=NE;j++){
		b[j]=h*df[j];
		y[j]=x[j]+b[j]/2.0;
	}

	derivs(y,df, particles);

	for(j=1;j<=NE;j++){
		c[j]=h*df[j];
		y[j]=x[j]+c[j];
	}

	derivs(y,df, particles);



	for(j=1;j<=NE;j++)
		x[j]=x[j]+(a[j]+h*df[j])/6.0+(b[j]+c[j])/3.0;


	index = 0;
	for(i=1; i<=NE; i+=3){
		particles[index].theta = x[i];
		particles[index].r_x = x[i+1];
		particles[index].r_y = x[i+2];

		// Updating the output for the ALF
		d_theta[index] = (  (a[i]+h*df[i])/6.0+(b[i]+c[i])/3.0  ) / h;


		if(particles[index].theta > Pi_2)
			particles[index].theta -= Pi_2;
		else if(particles[index].theta < 0.0)
			particles[index].theta += Pi_2;

		++index;
	}

		//std::cout << "\nid: " << i << "  x: " << particles[i].r_x << "  y: " << particles[i].r_y << "  theta: " << (particles[i].theta/M_PI) * 180.0 << std::endl;			
	
	t=t+h;
	it++;
}



void PCOD::destroy(){

	// Free memmory
	delete P;
	delete cc;
	free_dvector(y,1,NE+2);
	free_dvector(df,1,NE+2); 

	free_dvector(x,1,NE+1);
	free_dvector(a,1,NE+2);
	free_dvector(b,1,NE+2);
	free_dvector(c,1,NE+2);
}





