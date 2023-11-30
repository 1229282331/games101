#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

int main()
{
    Eigen::Vector3f x1 = {1, 0, 1}, x2 = {0, 1, 0};
    std::cout << pow((x1-x2).norm(), 2) << std::endl;




}