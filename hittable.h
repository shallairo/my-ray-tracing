#pragma once
#ifndef HITTABLE_H
#define HITTABLE_H

struct Bounds3; // 前向声明，不包含 fasterStructrue.h

class material;
class hit_record {
public:
    point3 p;
    vec3 normal;
    std::shared_ptr<material> mat;
    float t;
    bool front_face;
    void set_face_normal(const ray& r, const vec3& outward_normal) {
        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};
class hittable {
public:
    virtual ~hittable() = default;
    virtual Bounds3 bounding_box() const = 0;
    virtual inline point3 getMaxCornerPoint() const = 0;
    virtual inline point3 getMinCornerPoint() const = 0;
    virtual const point3 getCenter() const = 0;
    virtual const float getRadius() const = 0;
    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;
};

#endif