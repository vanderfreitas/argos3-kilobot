#include <iostream>
#include<math.h>
#include<stdio.h>
#include<stdlib.h>


void change(double *x){

	x[0] = 1;
}




int main(){
	double *x;


	x=(double *)malloc((size_t) ((3)*sizeof(double)));

	for(int i=0; i<3; ++i)
		x[i] = 0.5;


	std::cout << "Before x[0] = " << x[0] << std::endl;
	
	change(x);

	std::cout << "After x[0] = " << x[0] << std::endl;

	return 0;
}	
