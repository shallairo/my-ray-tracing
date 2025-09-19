#pragma once
#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"
#include "vec3.h"

class sphere : public hittable {
public:
    sphere(const point3& center, float radius, shared_ptr<material> mat)
        : center(center), radius(std::fmax(0, radius)), mat(mat) {
    }
    //sphere(const point3& center, double radius) : center(center), radius(std::fmax(0, radius)) {}
    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        vec3 oc = center - r.origin();
        auto a = r.direction().length_squared();
        auto h = dot(r.direction(), oc);
        auto c = oc.length_squared() - radius * radius;
        //精简版求根公式
        auto discriminant = h * h - a * c;
        if (discriminant < 0)
            return false;

        auto sqrtd = std::sqrt(discriminant);

        // Find the nearest root that lies in the acceptable range.
        auto root = (h - sqrtd) / a;
        //必须与在物体边界范围内
        if (!ray_t.surrounds(root)) {
            root = (h + sqrtd) / a;
            if (!ray_t.surrounds(root))
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        rec.mat = mat;
        vec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);

        return true;
    }
    Bounds3 bounding_box()const override {
        point3 pmin = Min(getMinCornerPoint(), getMaxCornerPoint());
        point3 pmax = Max(getMinCornerPoint(), getMaxCornerPoint());
        return Bounds3(pmin, pmax);
    }
    inline  point3 getMaxCornerPoint() const override {
      return point3(center[0] + radius, center[1] +  radius, center[2] + radius);
    }
    inline point3 getMinCornerPoint()const override {
        return point3(center[0] - radius, center[1] - radius, center[2] - radius);
    }
   const point3 getCenter()const override {
        return center;
    }
   const float getRadius()const override {
        return radius;
    }
private:
    point3 center;
    float radius;
    shared_ptr<material>mat;
};

#endif
