#include "filter.h"
#include <math.h>
#include <string.h>

// === Kalman Filter ===
void kalman_init(KalmanFilter *kf, float Q, float R, float initial_value) {
    kf->Q = Q;
    kf->R = R;
    kf->P = 1.0f;
    kf->x = initial_value;
}

float kalman_update(KalmanFilter *kf, float measurement) {
    // Prediction update
    kf->P += kf->Q;

    // Measurement update
    float K = kf->P / (kf->P + kf->R);
    kf->x += K * (measurement - kf->x);
    kf->P *= (1 - K);

    return kf->x;
}

// === Median Filter ===
static int compare(const void *a, const void *b) {
    float diff = (*(float *)a - *(float *)b);
    return (diff < 0) ? -1 : (diff > 0);
}

void median_init(MedianFilter *mf) {
    memset(mf->buffer, 0, sizeof(mf->buffer));
    mf->index = 0;
    mf->is_filled = false;
}

float median_update(MedianFilter *mf, float new_value) {
    mf->buffer[mf->index++] = new_value;
    if (mf->index >= MEDIAN_WINDOW_SIZE) {
        mf->index = 0;
        mf->is_filled = true;
    }

    float temp[MEDIAN_WINDOW_SIZE];
    memcpy(temp, mf->buffer, sizeof(temp));
    qsort(temp, MEDIAN_WINDOW_SIZE, sizeof(float), compare);
    
    return temp[MEDIAN_WINDOW_SIZE / 2];
}

// === Outlier Detection ===
bool is_outlier(float value, float mean, float threshold) {
    return fabsf(value - mean) > threshold;
}
