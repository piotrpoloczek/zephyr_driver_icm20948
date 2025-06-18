#pragma once

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} vec3f;

// Set all fields to 0
static inline void vec3f_zero(vec3f *v) {
    v->x = 0;
    v->y = 0;
    v->z = 0;
}

// Set from 3 floats
static inline void vec3f_set(vec3f *v, float x, float y, float z) {
    v->x = x;
    v->y = y;
    v->z = z;
}

// Scale a vector in-place
static inline void vec3f_scale(vec3f *v, float factor) {
    v->x *= factor;
    v->y *= factor;
    v->z *= factor;
}

// Copy one vector to another
static inline void vec3f_copy(vec3f *dest, const vec3f *src) {
    dest->x = src->x;
    dest->y = src->y;
    dest->z = src->z;
}

// Compute magnitude
static inline float vec3f_magnitude(const vec3f *v) {
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

// Normalize vector (returns magnitude before normalization)
static inline float vec3f_normalize(vec3f *v) {
    float mag = vec3f_magnitude(v);
    if (mag > 0.00001f) {
        float inv_mag = 1.0f / mag;
        vec3f_scale(v, inv_mag);
    }
    return mag;
}

#ifdef __cplusplus
}
#endif
