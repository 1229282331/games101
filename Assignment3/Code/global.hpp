//
// Created by LEI XU on 4/9/19.
//

#ifndef RASTERIZER_GLOBAL_H
#define RASTERIZER_GLOBAL_H

typedef unsigned char u08;
#define MY_PI 3.1415926
#define TWO_PI (2.0* MY_PI)

template<typename T, typename iter>
T sum(iter begin, iter end)
{
    T s;
    while(begin!=end)
        s += *(begin++);
    return s;
}



#endif //RASTERIZER_GLOBAL_H
