#ifndef __PID_H__
#define __PID_H__

class PID{
	private:
		float k_p, k_i, k_d;
		float e_p, e_i, e_d;
	public:
		PID(float k_p, float k_i, float k_d);
		//~PID() //unnecessary
		float compute(float err, float dt);
		void reset();

};

#endif
