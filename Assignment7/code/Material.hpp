//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType {DIFFUSE, MICROFACET, MIRROR};

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        /* a为单位球坐标系下的光线向量（比例值） */
        Vector3f B, C;  /*B,C为将N作为z建立的坐标系的任意选取的x,y轴*/
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N; /*将采样的单位光线向量比例值转为全局坐标系下*/
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f brdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            /*这里更加科学的做法是局部坐标与全局坐标相互转换，但效率会偏低*/
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z); /*仅是设置一个单位球坐标系下的局部光线向量（视为x,y,z分量比例）*/
            return toWorld(localRay, N);
            
            break;
        }
        case MICROFACET:
        {
            // uniform sample on the hemisphere
            /*这里更加科学的做法是局部坐标与全局坐标相互转换，但效率会偏低*/
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z); /*仅是设置一个单位球坐标系下的局部光线向量（视为x,y,z分量比例）*/
            return toWorld(localRay, N);
            
            break;
        }
        case MIRROR:
        {
            return reflect(-wi, N);
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case MICROFACET:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case MIRROR:
        {
            if (dotProduct(wo, N) > 0.0f)
                return 1.0f;
            else
                return 0.0f;
        }
    }
}

Vector3f Material::brdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFACET:
        {
            float cosalpha = dotProduct(N, wo);
            if(cosalpha<=0.0f)
                return Vector3f(0.0f);

            float roughness = 0.4;

            float Fr;   
            fresnel(wi, N, ior, Fr);

            auto G_func = [&](const Vector3f& wi, const Vector3f& wo, const Vector3f& n, const float& roughness=1.0)
            {
                double theta_i = acos(dotProduct(wi, n));
                double theta_o = acos(dotProduct(wo, n));
                double A_i = (-1 + sqrt(1+roughness*roughness*pow(tan(theta_i), 2))) / 2;
                double A_o = (-1 + sqrt(1+roughness*roughness*pow(tan(theta_o), 2))) / 2;
                float divisor = 1.0 + A_i + A_o;
                float res = 1.0f / divisor;
                if(divisor<EPSILON)
                    return 1.0f;

                return res;
            };
            float G = G_func(wi, wo, N, roughness);

            auto D_func = [&](const Vector3f& h, const Vector3f& n, const float& roughness=1.0)
            {
                double theta = acos(dotProduct(h, n));
                float divisor = M_PI * pow(roughness*roughness*pow(cos(theta), 2)+pow(sin(theta), 2), 2);
                float res = roughness*roughness / divisor;

                if(divisor<EPSILON)
                    return 1.0f;
                return res;
            };
            Vector3f h = (wi + wo).normalized();
            float D = D_func(h, N, roughness);
            Vector3f specular;
            float divisor = (4*dotProduct(N, wi)*dotProduct(N, wo));
            if(divisor<EPSILON)
                specular = Vector3f(1.0f);
            else
                specular = Fr*G*D / divisor;

            Vector3f diffuse = (1-Fr)*Kd / M_PI;

            return diffuse + specular;

            break;
        }
        case MIRROR:
        {
            float cosalpha = dotProduct(N, wo);
            float kr;
            if (cosalpha > 0.0f) {
                fresnel(wi, N, ior, kr);
                Vector3f mirror = 1 / cosalpha;
                return kr * mirror;        
            }
            else
                return Vector3f(0.0f);
            break;
        }
    }
}



#endif //RAYTRACING_MATERIAL_H
