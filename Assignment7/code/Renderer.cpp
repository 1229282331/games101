//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <thread>
#include <mutex>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 1e-4;
int prog = 0;
std::mutex locker;


void sub_render(const Scene& scene, std::vector<Vector3f> &framebuf, int spp, int msaa_n, float imageAspectRatio, float scale, const Vector3f& eye_pos, int start_height, int end_height)
{
    for (uint32_t j = start_height; j < end_height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            float msaa_rate = 1.0 / msaa_n;
            Vector3f pixel_sum(0.0f);
            for(uint32_t k_j=0; k_j<msaa_n; k_j++)
            {
                for(uint32_t k_i=0; k_i<msaa_n; k_i++)
                {
                    // generate primary ray direction
                    float x = (2 * (i + (k_i+0.5)*msaa_rate) / (float)scene.width - 1) *
                            imageAspectRatio * scale;
                    float y = (1 - 2 * (j + (k_j+0.5)*msaa_rate) / (float)scene.height) * scale;

                    float alpha = 0;
                    Vector3f rotateX_row1(1.0, 0, 0);
                    Vector3f rotateX_row2(0.0, cos(alpha/180.0*M_PI), sin(alpha/180.0*M_PI));
                    Vector3f rotateX_row3(0.0, -sin(alpha/180.0*M_PI), cos(alpha/180.0*M_PI));
                    Vector3f ray_dir(-x, y, 1);
                    Vector3f dir = normalize(Vector3f(dotProduct(rotateX_row1,ray_dir), dotProduct(rotateX_row2,ray_dir), dotProduct(rotateX_row3,ray_dir)));
                    Vector3f pixel_sub(0.0f);
                    for (int k = 0; k < spp; k++){
                        pixel_sub += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
                    }
                    pixel_sum += pixel_sub;
                }
            }
            framebuf[j*scene.width+i] = pixel_sum / float(msaa_n*msaa_n);
        }
        locker.lock();
        prog++;
        UpdateProgress(prog / (float)scene.height);
        locker.unlock();
    }
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    int m = 0;

    // change the spp value to change sample ammount
    int spp = 16;
    int msaa_n = 1;
    const int thread_num = 16;
    std::vector<std::thread> th;
    int thread_height = scene.height / thread_num;
    std::cout << "SPP: " << spp << "\n";
    
    for(int i=0; i<thread_num-1; i++)
        th.push_back(std::thread(sub_render, std::cref(scene), std::ref(framebuffer),
                         spp, msaa_n, imageAspectRatio, scale, std::cref(eye_pos), i*thread_height, (i+1)*thread_height));
    th.push_back(std::thread(sub_render, std::cref(scene), std::ref(framebuffer),
                         spp, msaa_n, imageAspectRatio, scale, std::cref(eye_pos), (thread_num-1)*thread_height, scene.height));

    for(int i=0; i<thread_num; i++)
        th[i].join();

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
