#include <cmath>
#include "alpha_main/pid.h"

PID::PID(float k_p, float k_i, float k_d):
	k_p(k_p),k_i(k_i),k_d(k_d)
{
	reset();
}

float PID::compute(float err, float dt){
	e_p = err;
	e_i += err * dt;
	e_d = (err - e_d);

	float res = k_p*e_p + k_i*e_i + k_d*e_d;
	e_d = err; // remember err for later
	return res;
}

void PID::reset(){
	e_p = e_i = e_d = 0.0;
}
