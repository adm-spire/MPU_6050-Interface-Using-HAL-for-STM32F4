#include "main.h"
#include "mpu.h"
#include "complementry.h"

#include <math.h>
#include "kalman_filter.h"

Kalman_t roll_kalman = {0};
Kalman_t pitch_kalman = {0};

void kalman_init(Kalman_t* kf) {
    kf->Q_angle = 0.001f;
    kf->Q_bias  = 0.003f;
    kf->R_measure = 0.03f;

    kf->angle = 0.0f;
    kf->bias  = 0.0f;

    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

float kalman_get_angle(Kalman_t* kf, float new_angle, float new_rate, float dt) {
	// Prediction step
	    float rate = new_rate - kf->bias;
	    kf->angle += dt * rate;
    // Update estimation error covariance - Project the error covariance ahead
	    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
	    kf->P[0][1] -= dt * kf->P[1][1];
	    kf->P[1][0] -= dt * kf->P[1][1];
	    kf->P[1][1] += kf->Q_bias * dt;

	    // Correction step
	        float y = new_angle - kf->angle;
	        float S = kf->P[0][0] + kf->R_measure;
	        float K[2];
	        K[0] = kf->P[0][0] / S;
	        K[1] = kf->P[1][0] / S;

	        kf->angle += K[0] * y;
	        kf->bias  += K[1] * y;

	        // Update error covariance matrix
	        float P00_temp = kf->P[0][0];
	        float P01_temp = kf->P[0][1];

	        kf->P[0][0] -= K[0] * P00_temp;
	        kf->P[0][1] -= K[0] * P01_temp;
	        kf->P[1][0] -= K[1] * P00_temp;
	        kf->P[1][1] -= K[1] * P01_temp;

	        return kf->angle;


}
