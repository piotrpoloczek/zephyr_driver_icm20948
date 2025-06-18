#pragma once

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct icm20948_vec3f {
    float x;
    float y;
    float z;
};

// Constructors
static inline struct icm20948_vec3f vec3f_make(float x, float y, float z) {
    return (struct icm20948_vec3f){x, y, z};
}

static inline struct icm20948_vec3f vec3f_zero(void) {
    return vec3f_make(0.0f, 0.0f, 0.0f);
}

// Unary operators
static inline struct icm20948_vec3f vec3f_neg(struct icm20948_vec3f v) {
    return vec3f_make(-v.x, -v.y, -v.z);
}

// Binary operations
static inline struct icm20948_vec3f vec3f_add(struct icm20948_vec3f a, struct icm20948_vec3f b) {
    return vec3f_make(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline struct icm20948_vec3f vec3f_sub(struct icm20948_vec3f a, struct icm20948_vec3f b) {
    return vec3f_make(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline struct icm20948_vec3f vec3f_scale(struct icm20948_vec3f v, float s) {
    return vec3f_make(v.x * s, v.y * s, v.z * s);
}

static inline struct icm20948_vec3f vec3f_div(struct icm20948_vec3f v, float s) {
    return vec3f_make(v.x / s, v.y / s, v.z / s);
}

// In-place operations
static inline void vec3f_add_inplace(struct icm20948_vec3f *a, struct icm20948_vec3f b) {
    a->x += b.x;
    a->y += b.y;
    a->z += b.z;
}

static inline void vec3f_sub_inplace(struct icm20948_vec3f *a, struct icm20948_vec3f b) {
    a->x -= b.x;
    a->y -= b.y;
    a->z -= b.z;
}

static inline void vec3f_scale_inplace(struct icm20948_vec3f *v, float s) {
    v->x *= s;
    v->y *= s;
    v->z *= s;
}

static inline void vec3f_div_inplace(struct icm20948_vec3f *v, float s) {
    v->x /= s;
    v->y /= s;
    v->z /= s;
}

#ifdef __cplusplus
}
#endif
