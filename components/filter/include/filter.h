#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== KALMAN FILTER =====
typedef struct {
    float x;  // estimated value
    float P;  // estimation error covariance
    float Q;  // process noise
    float R;  // measurement noise
} KalmanFilter;

void kalman_init(KalmanFilter *kf, float Q, float R, float initial_value);
float kalman_update(KalmanFilter *kf, float measurement);

// ===== MEDIAN FILTER =====
#define MEDIAN_WINDOW_SIZE 5

typedef struct {
    float buffer[MEDIAN_WINDOW_SIZE];
    int index;
    bool is_filled;
} MedianFilter;

void median_init(MedianFilter *mf);
float median_update(MedianFilter *mf, float new_value);

// ===== OUTLIER DETECTION =====
bool is_outlier(float value, float mean, float threshold);

#ifdef __cplusplus
}
#endif
