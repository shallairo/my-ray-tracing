#ifndef CAMERA_H
#define CAMERA_H

#include "hittable.h"
#include <fstream>
#include <vector>
#include "material.h"
class camera {
public:
    float aspect_ratio = 1.0;  // Ratio of image width over height
    int    image_width = 100;  // Rendered image width in pixel count
    int    samples_per_pixel = 10;   // Count of random samples for each pixel
    float vfov = 90;  // Vertical view angle (field of view)
    float RR_rate;//ray dis rate
    point3 lookfrom = point3(0, 0, 0);   // Point camera is looking from
    point3 lookat = point3(0, 0, -1);  // Point camera is looking at
    vec3   up = vec3(0, 1, 0);     // Camera-relative "up" direction
    BVHNode* node;
    int objectNum;
    void render(const hittable& world) {
        initialize();
        std::ofstream file("output.ppm");
        file << "P3\n" << image_width << " " << image_height << "\n255\n";  // 头写入文件

        for (int y = 0; y < image_height; ++y) {
            std::cout << "\rScanlines remaining: " << (image_height - y) << ' ' << std::flush;
            for (int x = 0; x < image_width; ++x) {
                color pixel_color(0, 0, 0);
                //多重采样抗锯齿
                for (int s = 0; s < samples_per_pixel; ++s) {
                    ray r = get_ray(x, y);  // 确保此函数正确实现
                    pixel_color += ray_color(r, world);
                }
                write_color(file, pixel_color* pixel_samples_scale);  // 传入采样数
            }
        }
        std::cout << "\rDone.\n";
    }


private:
    int    image_height;   // Rendered image height
    float pixel_samples_scale;  // Color scale factor for a sum of pixel samples
    point3 center;         // Camera center
    point3 pixel00_loc;    // Location of pixel 0, 0
    vec3   pixel_delta_u;  // Offset to pixel to the right
    vec3   pixel_delta_v;  // Offset to pixel below
    vec3   u, v, w;              // Camera frame basis vectors
    void initialize() {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        pixel_samples_scale = 1.0 / samples_per_pixel;

        center = lookfrom;

        // Determine viewport dimensions.
        auto focal_length = (lookfrom - lookat).length();
        auto theta = degrees_to_radians(vfov);
        auto h = std::tan(theta / 2);
        auto viewport_height = 2 * h * focal_length;
        auto viewport_width = viewport_height * (float(image_width) / image_height);

        // Calculate the u,v,w unit basis vectors for the camera coordinate frame.
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(up, w));
        v = cross(w, u);
        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        auto viewport_u = u * viewport_width;
        auto viewport_v = -v * viewport_height;

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Calculate the location of the upper left pixel.
        auto viewport_upper_left =
            center - focal_length*w - viewport_u / 2 - viewport_v / 2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);
    }
    ray get_ray(int i, int j) const {
        // Construct a camera ray originating from the origin and directed at randomly sampled
        // point around the pixel location i, j.

        auto offset = sample_square();
        auto pixel_sample = pixel00_loc
            + ((i + offset.x()) * pixel_delta_u)
            + ((j + offset.y()) * pixel_delta_v);

        auto ray_origin = center;
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }

    vec3 sample_square() const {
        // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
    }
    color ray_color(const ray& r, const hittable& world) {



        hit_record rec;
        BVHNode* head = node;

        // 首先检测光线与场景的交点
        if (intersection(head, r, rec, world)) {
            ray scattered;
            color attenuation;

            //  如果光线击中物体，并发生散射（例如反射或折射）
            if (rec.mat->scatter(r, rec, attenuation, scattered)) {

                //处理浮点精度问题：偏移散射光线的原点以避免自相交
                vec3 offset_origin = rec.p + rec.normal * 0.001f; // 沿法线方向微小偏移
                ray offset_scattered(offset_origin, scattered.direction());

                // 应用RR决定是否继续追踪
                double continue_probability = std::max(attenuation.x(), std::max(attenuation.y(), attenuation.z()));
                continue_probability = std::min(1.0, continue_probability); // 确保概率不超过1

                if (random_double() < continue_probability) {
                   
                    color recursive_color = ray_color(offset_scattered, world);
                    return attenuation * recursive_color / continue_probability;
                }
                else {
                    // 终止：返回黑色
                    return color(0, 0, 0);
                }
            }
            else {
          
                return color(0, 0, 0);
            }
        }
        vec3 unit_direction = unit_vector(r.direction());
        auto a = 0.5 * (unit_direction.y() + 1.0);
        return (1.0 - a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);
    }
    bool intersection(BVHNode* head, const ray& ray, hit_record& rec, const hittable& world) {
        
        if (objectNum>50) {
          return  BVHIntersect(head, ray, rec);
        }
        else {
            
             return world.hit(ray,interval(0.001, infinity),rec);
            
        }
    }
};

#endif