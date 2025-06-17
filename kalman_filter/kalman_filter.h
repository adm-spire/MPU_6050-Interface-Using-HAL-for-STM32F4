
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct {
    float angle;
    float bias;
    float rate;

    float P[2][2];

    float Q_angle;
    float Q_bias;
    float R_measure;
} Kalman_t;

extern Kalman_t roll_kalman;
extern Kalman_t pitch_kalman;

void kalman_init(Kalman_t* kf);
float kalman_get_angle(Kalman_t* kf, float new_angle, float new_rate, float dt);

#endif
