#include <stdlib.h>
#include "PID.h"

#define WINDOW 20


using namespace std;


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd) {
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	
}

void PID::UpdateError(double cte) {
	/*	d_error = (cte - p_error) / dt;
	p_error = cte;
	i_error = sum_i(cte * dt);*/
	// d_error is difference from old cte (p_error) to the new cte
	d_error = (cte - p_error);
	// p_error gets set to the new cte
	p_error = cte;
	//i_error is the sum of ctes to this point
	i_error += cte;
}

double PID::TotalError() {
	  //PID parameters corrected based on the vehicle velocity.
	return -Kp * p_error - Kd * d_error - Ki * i_error;
}

