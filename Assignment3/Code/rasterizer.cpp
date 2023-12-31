//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

inline float cross2(const Vector2f v1, const Vector2f v2)   /*二维向量叉乘*/
{
    return v1[0]*v2[1] - v2[0]*v1[1];
}

static bool insideTriangle(float x, float y, const Vector4f* _v)
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
    // cout << "res: " << res1 << ' '<< res2 << ' ' << res3 << endl;
    return res1<=0 && res2<=0 && res3<=0;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) { return v.template head<3>(); });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight=1.0)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight=1.0)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

float msaa(float x, float y, const Eigen::Vector4f *_v, int rate=4, float p=1.0)
{
    float res=0;
    if(rate==1)
        return float(insideTriangle(x, y, _v)) / float(rate);
    else
    {
        p /= 2;
        rate /= 4; 
        res += msaa(x-p/2, y-p/2, _v, rate, p);
        res += msaa(x+p/2, y-p/2, _v, rate, p);
        res += msaa(x-p/2, y+p/2, _v, rate, p);
        res += msaa(x+p/2, y+p/2, _v, rate, p);
        return res / 4;
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos, float threshold) 
{
    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer
    float xs[3] = {t.v[0][0], t.v[1][0], t.v[2][0]};
    float ys[3] = {t.v[0][1], t.v[1][1], t.v[2][1]};
    float left = *std::min_element(xs, xs+3);
    float right = *std::max_element(xs, xs+3);
    float bottom = *std::min_element(ys, ys+3);
    float top = *std::max_element(ys, ys+3);
    
    for(int i=int(left); i<=int(right); i++)
        for(int j=int(bottom); j<=int(top); j++)
        {
            if(insideTriangle(i+0.5, j+0.5, t.v))
            {
                /*msaa超采样*/
                float k = msaa(i+0.5, j+0.5, t.v, 1);
                auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v); /*获得重心坐标*/
                float Z = 1.0 / (alpha / t.v[0].w() + beta / t.v[1].w() + gamma / t.v[2].w());  /*获得深度插值*/
                /*0.插值深度值*/
                float zp = alpha * t.v[0].z() / t.v[0].w() + beta * t.v[1].z() / t.v[1].w() + gamma * t.v[2].z() / t.v[2].w();
                zp *= Z;
                /*1.矫正重心坐标*/
                alpha *= (Z/t.v[0].w());
                beta *= (Z/t.v[1].w());
                gamma *= (Z/t.v[2].w());
                /*2.插值颜色值*/
                auto interpolated_color = interpolate(alpha, beta, gamma, t.color[0], t.color[1], t.color[2], 1);
                /*3.插值法向量*/
                auto interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], 1);
                /*4.插值纹理位置*/
                auto interpolated_texcoords = interpolate(alpha, beta, gamma, t.tex_coords[0], t.tex_coords[1], t.tex_coords[2], 1);
                /*5.插值底纹颜色*/
                auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2], 1);
                fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                payload.view_pos = interpolated_shadingcoords;
                auto pixel_color = fragment_shader(payload);
                
                Eigen::Vector2i point = {i, j};
                save_history(point, pixel_color, zp, k);

                if(zp < depth_buf[get_index(i, j)])
                {
                    depth_buf[get_index(i, j)] = zp;
                    if(k <= threshold)
                        is_edge[get_index(i, j)] = true;
                    else
                    {
                        is_edge[get_index(i, j)] = false;
                        set_pixel(point, pixel_color*k);
                    }
                }
            }   
        } 
    for(int i=int(left); i<=int(right); i++)
        for(int j=int(bottom); j<=int(top); j++)
        {
            if(is_edge[get_index(i, j)])
            {
                Eigen::Vector2i point = {i, j};
                set_edge(point);
            }

        }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    history_frame_buf.resize(w * h);
    is_edge.resize(w * h);
    history_sample_buf.resize(w * h);
    history_depth_buf.resize(w * h);
    std::fill(is_edge.begin(), is_edge.end(), false);
    texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() <= 0 || point.x() >= width ||
        point.y() <= 0 || point.y() >= height) return;
    int ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_edge(const Vector2i &point)
{
    int ind = (height-point.y())*width + point.x();
    auto color_vec = history_frame_buf[ind];
    auto sample_vec = history_sample_buf[ind];
    auto depth_vec = history_depth_buf[ind];
    int len = sample_vec.size();
    Eigen::Vector3f res_color = {0, 0, 0};

    float mean_depth = sum<float>(depth_vec.begin(), depth_vec.end())/depth_vec.size();
    for(int i=0; i<len;i++)
    {
        if(color_vec[i].norm()==0 && depth_buf[i]>=mean_depth)
            sample_vec[i] = 0;
    }
    float sample_sum = sum<float>(sample_vec.begin(), sample_vec.end());
    for(int i=0; i<len; i++)
    {
        sample_vec[i] /= sample_sum; 
        res_color += (color_vec[i] * sample_vec[i]);
        // printf("c%d[%.3f,%.3f,%.3f, %.3f, %.3f] ", i, color_vec[i].x(), color_vec[i].y(), color_vec[i].z(), depth_vec[i], sample_vec[i]);
    }
    // printf("result color:[%.3f,%.3f,%.3f]\n", res_color.x(), res_color.y(), res_color.z());


    set_pixel(point, res_color);
}

void rst::rasterizer::save_history(const Vector2i &point, const Eigen::Vector3f &color, float depth, float k)
{
    int ind = (height-point.y())*width + point.x();
    k = (k==0.f)?0.1:k;
    history_frame_buf[ind].push_back(color);
    history_depth_buf[ind].push_back(depth);
    history_sample_buf[ind].push_back(k);
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}

