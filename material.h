#ifndef MATERIAL_H
#define MATERIAL_H

#include "hittable.h"


class material {
public:
    virtual ~material() = default;

    virtual bool scatter(
        const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
    ) const {
        return false;
    }
};
//���뷴����� ��ÿ�����䷽����ʶ���ͬ
class lambertian : public material {
public:
    lambertian(const color& albedo) : albedo( ) {}

    bool scatter(const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered)
        const override {
     
        auto scatter_direction = rec.normal + random_unit_vector();
        //���ⷨ�ߺ������������Ϊ0
        // Catch degenerate scatter direction
        if (scatter_direction.near_zero())
            scatter_direction = rec.normal;
        scattered = ray(rec.p, scatter_direction);
        //������ģ�
        attenuation = albedo;
        return true;
    }

private:
    //������
    color albedo;
};
//�������� ���շ��䶨��
class metal : public material {
public:
    metal(const color& albedo,float fuzz) : albedo(albedo),fuzz(fuzz) {}

    bool scatter(const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered)
        const override {
        vec3 reflected = reflect(r_in.direction(), rec.normal);
        reflected = unit_vector(reflected) + fuzz*random_unit_vector();
        scattered = ray(rec.p, reflected);
        attenuation = albedo;
        //
        return dot(rec.normal,reflected)>0;
    }

private:
    color albedo;
    //ģ��ϵ��
    float fuzz;
};
//��������
class dielectric : public material {
public:
    //refraction_index n2/n1
    dielectric(double refraction_index) : refraction_index(refraction_index) {}

    bool scatter(const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered)
        const override {
        attenuation = color(1.0, 1.0, 1.0);
        //�����ʣ��ж��ǹ�����ʽ����ܽ���  ���ǹ��ܽ��ʹ������
        double ri = rec.front_face ? (1.0 / refraction_index) : refraction_index;

        vec3 unit_direction = unit_vector(r_in.direction());
        double cos_theta = std::fmin(dot(-unit_direction, rec.normal), 1.0);
        double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
        //����ȫ����
        bool cannot_refract = ri * sin_theta > 1.0;
        vec3 direction;

        //cannot_refractΪһ���������  
        //reflectance������Ǻܴ�ʱͨ�����Ƶõ���R����ģ����Է���ĸ���
        if (cannot_refract || reflectance(cos_theta, ri) > random_double())
            direction = reflect(unit_direction, rec.normal);
        else
            direction = refract(unit_direction, rec.normal, ri);

        scattered = ray(rec.p, direction);
        return true;
    }

private:

    double refraction_index;
    static double reflectance(double cosine, double refraction_index) {
        // Use Schlick's approximation for reflectance.
        auto r0 = (1 - refraction_index) / (1 + refraction_index);
        r0 = r0 * r0;
        return r0 + (1 - r0) * std::pow((1 - cosine), 5);
    }
};
#endif
