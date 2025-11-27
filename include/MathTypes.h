#pragma once

#include <cmath>

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quat {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

inline Quat Multiply(const Quat& a, const Quat& b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    Quat qConj{-q.x, -q.y, -q.z, q.w};
    Quat vQuat{v.x, v.y, v.z, 0.0f};
    Quat tmp = Multiply(q, vQuat);
    Quat res = Multiply(tmp, qConj);
    return {res.x, res.y, res.z};
}
