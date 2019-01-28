#include <stdlib.h>
#include "PID.h"

#define WINDOW 20


using namespace std;


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	window      = (int*) calloc( WINDOW , sizeof(window[0]) ) ;
	i           = WINDOW - 1 ;
	sum         = 0 ;
}

void PID::UpdateError(double cte, double dt) {
	d_error = (cte - p_error) / dt;
	p_error = cte;
	i_error = sum_i(cte * dt);
}

double PID::TotalError(double cur_vel) {
	  //PID parameters corrected based on the vehicle velocity.
	return (Kp - 0.0032 * cur_vel) * p_error + Ki * i_error + (Kd + 0.0002 * cur_vel) * d_error;
}

// For i_error calculation we sum up only last 20 measurements
// // inspired by code from https://stackoverflow.com/questions/25024012/running-sum-of-the-last-n-integers-in-an-array
double PID::sum_i(double err){
	i          = (i+1) % WINDOW ;
	sum        = sum - window[i] + err ;
  return sum;
}
