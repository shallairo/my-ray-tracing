#ifndef HITTABLE_LIST_H
#define HITTABLE_LIST_H

#include "hittable.h"

#include <vector>

class hittable_list : public hittable {
public:
    std::vector<shared_ptr<hittable>> objects;

    hittable_list() {}
    hittable_list(shared_ptr<hittable> object) { add(object); }
    inline  point3 getMaxCornerPoint()const override {
        return point3(0,0,0);
    }
    inline point3 getMinCornerPoint()const override {
        return point3(0, 0, 0);
    }
     Bounds3 bounding_box()const override {
       
        return Bounds3( );
    }
    const point3 getCenter()const override {
        return point3(0, 0, 0);
    }
    const float getRadius()const override {
        return 0;
    }
    void clear() { objects.clear(); }

    void add(shared_ptr<hittable> object) {
        objects.push_back(object);
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        hit_record temp_rec;
        bool hit_anything = false;
        auto closest_so_far = ray_t.max;
        //返回最近的交点
        for (const auto& object : objects) {
            if (object->hit(r, interval(ray_t.min, closest_so_far), temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }

        return hit_anything;
    }
    
};

#endif
