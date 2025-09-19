#pragma once
#include <cassert>
#ifndef VEC3_H
#define VEC3_H


class vec3 {
public:
    float e[3];

    vec3() : e{ 0,0,0 } {}
    vec3(float e0, float e1, float e2) : e{ e0, e1, e2 } {}

    float x() const { return e[0]; }
    float y() const { return e[1]; }
    float z() const { return e[2]; }

    vec3 operator-() const { return vec3(-e[0], -e[1], -e[2]); }
    //float operator[](int i) const { return e[i]; }
    float operator[](int i) const {
        assert(i >= 0 && i < 3);
        return e[i];
    }
    float& operator[](int i) { return e[i]; }

    vec3& operator+=(const vec3& v) {
        e[0] += v.e[0];
        e[1] += v.e[1];
        e[2] += v.e[2];
        return *this;
    }

    vec3& operator*=(float t) {
        e[0] *= t;
        e[1] *= t;
        e[2] *= t;
        return *this;
    }

    vec3& operator/=(float t) {
        return *this *= 1 / t;
    }
    float length() const {
        return std::sqrt(length_squared());
    }

    float length_squared() const {
        return e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    }
    bool near_zero() const {
        // Return true if the vector is close to zero in all dimensions.
        auto s = 1e-8;
        return (std::fabs(e[0]) < s) && (std::fabs(e[1]) < s) && (std::fabs(e[2]) < s);
    }
    static vec3 random() {
        return vec3(random_double(), random_double(), random_double());
    }

    static vec3 random(float min, float max) {
        return vec3(random_double(min,max), random_double(min,max), random_double(min,max));
    }
};

// point3 is just an alias for vec3, but useful for geometric clarity in the code.
using point3 = vec3;


// Vector Utility Functions

inline std::ostream& operator<<(std::ostream& out, const vec3& v) {
    return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

inline vec3 operator+(const vec3& u, const vec3& v) {
    return vec3(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

inline vec3 operator-(const vec3& u, const vec3& v) {
    return vec3(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

inline vec3 operator*(const vec3& u, const vec3& v) {
    return vec3(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
}

inline vec3 operator*(float t, const vec3& v) {
    return vec3(t * v.e[0], t * v.e[1], t * v.e[2]);
}

inline vec3 operator*(const vec3& v, float t) {
    return t * v;
}

inline vec3 operator/(const vec3& v, float t) {
    return (1 / t) * v;
}
inline vec3 Min(const vec3& p1, const vec3& p2) {
    return vec3(std::min(p1[0], p2[0]), std::min(p1[1], p2[1]),
        std::min(p1[2], p2[2]));
}

inline vec3 Max(const vec3& p1, const vec3& p2) {
    return vec3(std::max(p1[0], p2[0]), std::max(p1[1], p2[1]),
        std::max(p1[2], p2[2]));
}

inline float dot(const vec3& u, const vec3& v) {
    return u.e[0] * v.e[0]
        + u.e[1] * v.e[1]
        + u.e[2] * v.e[2];
}

inline vec3 cross(const vec3& u, const vec3& v) {
    return vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
        u.e[2] * v.e[0] - u.e[0] * v.e[2],
        u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

inline vec3 unit_vector(const vec3& v) {
    return v / v.length();
}
inline vec3 random_unit_vector() {
    while (true) {
        auto p = vec3::random(-1, 1);
        auto lensq = p.length_squared();
        //to esure the point is inside sphere
        if (lensq <= 1)
            //return normalized vector
            return p / sqrt(lensq);
    }
}
inline vec3 random_on_hemisphere(const vec3& normal) {
    //diffuse matriral
    vec3 on_unit_sphere = random_unit_vector();
    if (dot(on_unit_sphere, normal) > 0.0) // In the same hemisphere as the normal
        return on_unit_sphere;
    else
        return -on_unit_sphere;
}
inline vec3 reflect(const vec3& v,const vec3&n ) {
    return v - 2 * dot(n, v) * n;
}
inline vec3 refract(const vec3& uv, const vec3& n, float etai_over_etat) {
    float cos_theta = std::min(dot(-uv, n), 1.f);
    vec3 r_out_perpindicular = etai_over_etat * (uv + cos_theta * n);
    vec3 r_out_parallel = -std::sqrt(std::fabs(1 - r_out_perpindicular.length_squared())) * n;
    return r_out_parallel + r_out_perpindicular;
}
#endif