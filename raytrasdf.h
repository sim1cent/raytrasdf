#define arr_size(x) (sizeof(x)/sizeof(*(x)))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

typedef uint32_t uint;

typedef struct{
    float x,y,z;
}V3;

#define v_cmp(a,b) (((a).x==(b).x)&&((a).y==(b).y)&&((a).z==(b).z))

typedef struct{
    uint8_t r,g,b;
}Color;

typedef struct Obj{
    V3 pos;
    V3 color;
    int specular;
    float reflection;
    enum{SPHERE,PLANE,BOX,MESH_TRIANGLE,MESH}type;
    union{
        float sphere_radius;
        struct Plane{
            V3 normal;
            float tile_size;
            V3 color2;
        }plane;
        V3 aabb_v1;
        struct Triangle{
            struct Vertex{
                V3 pos,normal;
            }vertices[3];
            V3 normal;
            float area;
        }triangle;
        struct Mesh{
            char *file_name;
            float yaw,pitch,roll,scale;
            float smooth;
            uint octree_depth;
            struct Obj *triangles_buf;
            void *root;
        }mesh;
    }obj;
}Obj;

typedef struct Octree_node{
    Obj bounding;
    Obj *triangles;
    uint triangle_n;
    struct Octree_node *c_nodes[8];
}Octree_node;

typedef struct{
    V3 pos;
    V3 intensity;
    enum{POINT,SURFACE}type;
    V3 u,v;
    uint u_size,v_size;
}Light;

static inline float v_dot(V3 v0, V3 v1){
    return v0.x*v1.x+v0.y*v1.y+v0.z*v1.z;
}

static inline V3 v_cross(V3 v0, V3 v1){
    return (V3){v0.y*v1.z-v0.z*v1.y,v0.z*v1.x-v0.x*v1.z,v0.x*v1.y-v0.y*v1.x};
}

static inline float v_len(V3 v){
    return sqrtf(v_dot(v,v));
}

static inline V3 v_mul(V3 v, float s){
    return (V3){v.x*s,v.y*s,v.z*s};
}

static inline V3 v_mul2(V3 v0, V3 v1){
    return (V3){v0.x*v1.x,v0.y*v1.y,v0.z*v1.z};
}

static inline V3 v_add(V3 v0, V3 v1){
    return (V3){v0.x+v1.x,v0.y+v1.y,v0.z+v1.z};
}

static inline V3 v_sub(V3 v0, V3 v1){
    return (V3){v0.x-v1.x,v0.y-v1.y,v0.z-v1.z};
}

static inline V3 v_norm(V3 v){
    float len=v_len(v);
    if(len!=0)
        return v_mul(v,1/len);
    return (V3){0,0,0};
}

static inline V3 v_transform(V3 v, V3 *matrix){
    V3 new_v;
    new_v.x=v.x*matrix[0].x+v.y*matrix[1].x+v.z*matrix[2].x;
    new_v.y=v.x*matrix[0].y+v.y*matrix[1].y+v.z*matrix[2].y;
    new_v.z=v.x*matrix[0].z+v.y*matrix[1].z+v.z*matrix[2].z;
    return new_v;
}

static inline V3 *matrix_mul(V3 *m0, V3 *m1, V3 *m2){
    V3 new_matrix[3];
    for(uint i=0;i<3;++i)
        new_matrix[i]=v_transform(m0[i],m1);
    memcpy(m2,new_matrix,3*sizeof(V3));
    return m2;
}

static inline void gen_matrix(float yaw, float pitch, float roll, float scale, V3 *matrix){
    yaw*=2*M_PI;
    pitch*=2*M_PI;
    roll*=2*M_PI;
    V3 yaw_matrix[]={{cosf(yaw),0,-sinf(yaw)},{0,1,0},{sinf(yaw),0,cosf(yaw)}};
    V3 pitch_matrix[]={{1,0,0},{0,cosf(pitch),-sinf(pitch)},{0,sinf(pitch),cosf(pitch)}};
    V3 roll_matrix[]={{cosf(roll),sinf(roll),0},{-sinf(roll),cosf(roll),0},{0,0,1}};
    V3 scale_matrix[]={{scale,0,0},{0,scale,0},{0,0,scale}};
    matrix_mul(scale_matrix,matrix_mul(roll_matrix,matrix_mul(pitch_matrix,yaw_matrix,matrix),matrix),matrix);
}

static inline bool within_box(Obj aabb, V3 p){
    union{
        V3 v;
        float v_arr[3];
    }p_v,min_v,max_v;
    p_v.v=p;
    min_v.v=aabb.pos;
    max_v.v=aabb.obj.aabb_v1;
    for(uint i=0;i<3;++i){
        if(p_v.v_arr[i]<min_v.v_arr[i]||p_v.v_arr[i]>max_v.v_arr[i])
            return false;
    }
    return true;
}

static inline Color saturate(V3 c){
    union{
        V3 c;
        float v_arr[3];
    }c0;
    c0.c=c;
    union{
        Color c;
        uint8_t c_arr[3];
    }c1;
    for(uint i=0;i<3;++i){
        if(c0.v_arr[i]>255)
            c1.c_arr[i]=255;
        else if(c0.v_arr[i]<0)
            c1.c_arr[i]=0;
        else
            c1.c_arr[i]=c0.v_arr[i];
    }
    return c1.c;
}

typedef struct{
    Obj *closest_obj;
    float t;
    V3 p,n;
}Intersection;

enum TYPE{FIRST,SHADOW,REFLECTION};

struct draw_args{
    uint w,h;
    Color *pic;
};

void load_stl(Obj*);
void destroy_mesh(Obj*);
void start_work(void(*)(uint,uint,V3*),uint,V3*);
void *draw_stuff(struct draw_args*);
