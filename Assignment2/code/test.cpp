#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;

inline float cross2(const Vector2f v1, const Vector2f v2)
{
    return v1[0]*v2[1] - v2[0]*v1[1];
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    /*v0,v1,v2各点遵循逆时针*/
    Vector2f p = {x, y};
    Vector2f v0v1 = (_v[1]-_v[0]).head<2>();
    Vector2f v1v2 = (_v[2]-_v[1]).head<2>();
    Vector2f v2v0 = (_v[0]-_v[2]).head<2>();
    Vector2f v0p = p-_v[0].head<2>();
    Vector2f v1p = p-_v[1].head<2>();
    Vector2f v2p = p-_v[2].head<2>();

    /*如果点在三角形内，则点均在三边围成的闭合向量的左侧，也即点到顶点向量与对应边向量二维叉乘的绝对值小于0*/
    float res1 = cross2(v0p, v0v1), res2 = cross2(v1p, v1v2), res3 = cross2(v2p, v2v0);
    cout << "res: " << res1 << ' '<< res2 << ' ' << res3 << endl;
    return res1<=0 && res2<=0 && res3<=0;
}

int main()
{
    cout << int(true) << end;
}