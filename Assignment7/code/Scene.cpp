//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection ray_inter = this->intersect(ray); 
    if(!ray_inter.happened)
        return Vector3f(0.0);

    /*待渲染点p的属性*/
    Vector3f p = ray_inter.coords;  /*相交点p*/
    Vector3f N = ray_inter.normal.normalized();
    Material* m = ray_inter.m;
    Vector3f wo = (ray.origin - p).normalized();    /*p->camera*/

    /*0.自发光*/
    Vector3f L_selfdir = 0.0;  
    if(ray_inter.obj->hasEmit() && depth==0)
        L_selfdir = ray_inter.m->getEmission();

    /*1.直接光照*/
    /*从所有面光源（包括自发光物体）采样点光源（一根光线）*/
    Vector3f L_dir = 0.0;  
    Intersection inter_light;
    float pdf_light;
    sampleLight(inter_light, pdf_light);
    Vector3f ws = (inter_light.coords - p).normalized();
    float distance = (inter_light.coords-p).norm();
    p = (dotProduct(ws, N)<0) ? p-N*EPSILON : p+N*EPSILON;

    // Intersection light2scene = this->intersect(Ray(inter_light.coords, -ws));   /*从光源发出射线判断相交*/
    // if(light2scene.happened && light2scene.distance>distance-0.01) /*判断p点与采样点光源之间没有物体遮挡*/
    // {
    //     Vector3f NN = inter_light.normal.normalized();
        
    //     Vector3f eval = m->eval(wo, ws, N);
    //     L_dir = inter_light.emit * eval * dotProduct(ws, N) * dotProduct(-ws, NN) / pow(distance, 2) / pdf_light;
    // }
    Intersection p2scene = this->intersect(Ray(p, ws));   /*从p向光源发出射线判断相交*/
    if(p2scene.happened && p2scene.m->hasEmission()) /*判断p点与采样点光源之间没有物体遮挡*/
    {
        Vector3f NN = inter_light.normal.normalized();
        
        Vector3f eval = m->eval(wo, ws, N);
        L_dir = inter_light.emit * eval * dotProduct(ws, N) * dotProduct(-ws, NN) / pow(distance, 2) / pdf_light;
    }

    /*2.间接光照*/
    Vector3f L_indir = 0.0;
    float p_rr = get_random_float();
    if(p_rr < RussianRoulette)
    {
        Vector3f wi = m->sample(wo, N).normalized(); /*采样一个渲染点p的反射方向*/
        Intersection q_inter = this->intersect(Ray(p, wi)); /*p->q*/
        if(q_inter.happened && !q_inter.m->hasEmission())
        {
            float pdf = m->pdf(wo, wi, N);
            if(pdf<EPSILON)
                L_indir = 0.0;
            else
                L_indir = castRay(Ray(p, wi), depth+1) * m->eval(wo, wi, N) * dotProduct(wi, N) / pdf / RussianRoulette;
        }
    }

    return L_selfdir + L_dir + L_indir;
}
