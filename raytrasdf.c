#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>
#include "raytrasdf.h"

#define WIDTH (1000)
#define HEIGHT (700)
#define RECURSION_DEPTH (3)
#define SPP (2)
#define MESHCULLING
//#define OUTPUT360

#define SDLPREVIEW

static Light light_sources[]={
    {{0.5,0,0},{2,2,.5},SURFACE,{0,0.03,0},{0.03,0,0},3,3},
    {{-0.5,0,7},{.5,.2,1},POINT},
};

static float ambient_light = 0;

static Obj scene[]={
    {{0,-1,0},{400,400,400},-1,0.2,PLANE,{.plane={{0,1,0},0.5,{50,50,50}}}},
    {{0,3,0},{400,400,400},-1,0,PLANE,{.plane={{0,-1,0},1,{50,50,50}}}},
    {{-2,0,0},{300,0,0},-1,0,PLANE,{.plane={{1,0,0},1,{50,0,0}}}},
    {{0,0,10},{0,300,0},-1,0,PLANE,{.plane={{0,0,-1},1,{0,50,0}}}},
    {{2,0,0},{0,0,300},-1,0,PLANE,{.plane={{-1,0,0},1,{0,0,50}}}},
    {{-1,-1,2.5},{0,100,100},-1,0,BOX,{.aabb_v1={1,-0.9,5}}},
    {{-1,0,-2},{255,0,255},500,0.2,SPHERE,{.sphere_radius=0.7}},
    {{0,-0.9,3},{300,300,300},1000,0.5,MESH,{.mesh={"teapot.stl",0,.25,.5,.1,1,3}}},
};

static V3 camera_pos = {-0.5,0,0};
static float yaw = 0.04;
static float pitch = 0;
static float roll = 0;

static Color pic[WIDTH*HEIGHT];
static bool ray_sphere(V3,V3,Obj,float*,V3*);
static bool ray_plane(V3,V3,Obj,float*,V3*);
static bool ray_triangle(V3,V3,Obj,float*,V3*);
static bool ray_aabb(V3,V3,Obj,float*,V3*);
static void ray_octree(V3,V3,enum TYPE,Intersection*,Octree_node*);
static inline V3 plane_color(Obj,V3);
static Intersection ray_scene(V3,V3,enum TYPE,Obj*,uint);
static void shade(float*,float*,Intersection,Light,V3);
static V3 trace_ray(V3,V3,uint);
static inline void out_pic(char*);
static void render_worker(uint,uint,V3*);

static bool ray_sphere(V3 O, V3 D, Obj sphere, float *t, V3 *n){
    V3 OC=v_sub(O,sphere.pos);
    float a=v_dot(D,D);
    float b=2*v_dot(D,OC);
    float c=v_dot(OC,OC)-sphere.obj.sphere_radius*sphere.obj.sphere_radius;
    float d=b*b-4*a*c;
    if(d>=0){
        float t1=(-b+sqrtf(d))/(2*a);
        float t2=(-b-sqrtf(d))/(2*a);
        *t=((t1<t2||t2<0)&&t1>=0?t1:t2);
        if(*t>=0){
            if(n!=NULL) *n=v_sub(v_add(v_mul(D,*t),O),sphere.pos);
            return true;
        }
    }
    return false;
}

static bool ray_plane(V3 O, V3 D, Obj plane, float *t, V3 *n){
    float a=v_dot(O,plane.obj.plane.normal);
    float b=v_dot(plane.pos,plane.obj.plane.normal);
    float c=v_dot(D,plane.obj.plane.normal);
    *t=-(a-b)/c;
    if(*t>=0){
        if(n!=NULL) *n=plane.obj.plane.normal;
        return true;
    }
    return false;
}

static bool ray_triangle(V3 O, V3 D, Obj triangle, float *t, V3 *n){
    Obj new_plane;
    new_plane.pos=triangle.obj.triangle.vertices[0].pos;
    new_plane.obj.plane.normal=triangle.obj.triangle.normal;
    if(ray_plane(O,D,new_plane,t,NULL)){
        V3 p,ab,ac;
        p=v_add(O,v_mul(D,*t));
        ab=v_sub(triangle.obj.triangle.vertices[1].pos,triangle.obj.triangle.vertices[0].pos);
        ac=v_sub(triangle.obj.triangle.vertices[2].pos,triangle.obj.triangle.vertices[0].pos);
        float u=v_len(v_cross(ab,v_sub(p,new_plane.pos)))/triangle.obj.triangle.area;
        float v=v_len(v_cross(ac,v_sub(p,new_plane.pos)))/triangle.obj.triangle.area;
        float w=v_len(v_cross(v_sub(ac,ab),v_sub(v_sub(p,new_plane.pos),ab)))/triangle.obj.triangle.area;
        if(u+v+w<=1.0001){
            if(n!=NULL){
                *n=(V3){0,0,0};
                *n=v_add(*n,v_mul(triangle.obj.triangle.vertices[0].normal,w));
                *n=v_add(*n,v_mul(triangle.obj.triangle.vertices[1].normal,v));
                *n=v_add(*n,v_mul(triangle.obj.triangle.vertices[2].normal,u));
            }
            return true;
        }
    }
    return false;
}

static bool ray_aabb(V3 O, V3 D, Obj aabb, float *t, V3 *n){
    union{
        V3 v;
        float v_arr[3];
    }t_v,o_v,d_v,n_v,min_v,max_v;
    d_v.v=D;
    o_v.v=O;
    min_v.v=aabb.pos;
    max_v.v=aabb.obj.aabb_v1;
    uint max_i=0;
    bool inside_box=within_box(aabb,O);
    for(uint i=0;i<3;++i){
        if(d_v.v_arr[i]!=0){
            if(inside_box){
                t_v.v_arr[i]=((d_v.v_arr[i]<0?min_v.v_arr[i]:max_v.v_arr[i])-o_v.v_arr[i])/d_v.v_arr[i];
                max_i=(t_v.v_arr[i]<t_v.v_arr[max_i]||(t_v.v_arr[max_i]<0&&t_v.v_arr[i]>=0)?i:max_i);
            }
            else{
                t_v.v_arr[i]=((d_v.v_arr[i]>0?min_v.v_arr[i]:max_v.v_arr[i])-o_v.v_arr[i])/d_v.v_arr[i];
                max_i=(t_v.v_arr[i]>t_v.v_arr[max_i]?i:max_i);
            }
        }
        else if(max_i==i) ++max_i;
    }
    if(max_i<3&&t_v.v_arr[max_i]>=0){
        *t=t_v.v_arr[max_i];
        if(inside_box||within_box(aabb,v_add(v_mul(D,*t+0.0001),O))){
            if(n!=NULL){
                n_v.v=(V3){0,0,0};
                n_v.v_arr[max_i]=1;
                *n=n_v.v;
            }
            return true;
        }
    }
    return false;
}

static void ray_octree(V3 O, V3 D, enum TYPE type, Intersection *intersection, Octree_node *node){
    Intersection new_intersection;
    if(node!=NULL&&ray_aabb(O,D,node->bounding,&new_intersection.t,NULL)&&
    (intersection->closest_obj==NULL||new_intersection.t<intersection->t||within_box(node->bounding,O))){
        new_intersection=ray_scene(O,D,type,node->triangles,node->triangle_n);
        if(new_intersection.closest_obj!=NULL&&(intersection->closest_obj==NULL||new_intersection.t<intersection->t))
            *intersection=new_intersection;
        for(uint i=0;i<8;++i){
            if(intersection->closest_obj!=NULL&&type==SHADOW) return;
            ray_octree(O,D,type,intersection,node->c_nodes[i]);
        }
    }
}

static inline V3 plane_color(Obj plane, V3 p){
    V3 ort1=v_norm(v_cross(plane.obj.plane.normal,(V3){1,0,0}));
    if(ort1.x==0&&ort1.y==0&&ort1.z==0)
        ort1=v_norm(v_cross(plane.obj.plane.normal,(V3){0,0,1}));
    V3 ort2=v_norm(v_cross(plane.obj.plane.normal,ort1));
    float u=v_dot(ort1,v_sub(p,plane.pos));
    float v=v_dot(ort2,v_sub(p,plane.pos));
    if((((int)(v/plane.obj.plane.tile_size)+(int)(u/plane.obj.plane.tile_size))&1)!=(v*u<0))
        return plane.color;
    else
        return plane.obj.plane.color2;
}

static Intersection ray_scene(V3 O, V3 D, enum TYPE type, Obj *buf, uint s){
    Intersection intersection,new_intersection;
    intersection.closest_obj=NULL;
    bool success;
    for(uint i=0;i<s;++i){
        switch(buf[i].type){
            case SPHERE:
                success=ray_sphere(O,D,buf[i],&new_intersection.t,(type!=SHADOW?&new_intersection.n:NULL));
                break;
            case PLANE:
                success=ray_plane(O,D,buf[i],&new_intersection.t,(type!=SHADOW?&new_intersection.n:NULL));
                break;
            case BOX:
                success=ray_aabb(O,D,buf[i],&new_intersection.t,(type!=SHADOW?&new_intersection.n:NULL));
                break;
            case MESH_TRIANGLE:
#ifdef MESHCULLING
                if(v_dot(buf[i].obj.triangle.normal,D)>=0)
                    success=false;
                else
#endif
                    success=ray_triangle(O,D,buf[i],&new_intersection.t,(type!=SHADOW?&new_intersection.n:NULL));
                break;
            case MESH:
                new_intersection=intersection;
                ray_octree(O,D,type,&new_intersection,buf[i].obj.mesh.root);
                if(new_intersection.closest_obj!=NULL)
                    success=true;
                else
                    success=false;
                break;
            default:
                success=false;
        }
        if(intersection.closest_obj==NULL||new_intersection.t<intersection.t){
            if(success&&((type==FIRST&&new_intersection.t>=1)||(type==SHADOW&&new_intersection.t<=1)||type==REFLECTION)){
                intersection=new_intersection;
                intersection.p=v_add(O,v_mul(D,intersection.t-0.0001));
                intersection.closest_obj=buf+i;
                if(type==SHADOW) return intersection;
            }
        }
    }
    if(intersection.closest_obj!=NULL)
        intersection.n=v_norm(v_mul(intersection.n,-v_dot(intersection.n,D)));
    return intersection;
}

static void shade(float *diffuse_intensity, float *specular_intensity, Intersection intersection, Light light, V3 d){
    if(light.type==POINT){
        V3 l=v_sub(light.pos,intersection.p);
        if(v_dot(l,intersection.n)>0&&ray_scene(intersection.p,l,SHADOW,scene,arr_size(scene)).closest_obj==NULL){
            *diffuse_intensity+=v_dot(intersection.n,l)/(v_len(l)*v_dot(l,l));
            V3 s=v_sub(v_mul(intersection.n,2*v_dot(intersection.n,l)),l);
            if(intersection.closest_obj->specular>0&&v_dot(d,s)>0)
                *specular_intensity+=pow(v_dot(d,s)/(v_len(d)*v_len(s)),intersection.closest_obj->specular);
        }
    }
    else{
        for(uint i=0;i<light.v_size;++i){
            for(uint j=0;j<light.u_size;++j)
                shade(diffuse_intensity,specular_intensity,intersection,(Light){v_add(light.pos,
                    v_add(v_mul(light.v,(float)i/light.v_size),v_mul(light.u,(float)j/light.u_size))),light.intensity,POINT},d);
        }
        *diffuse_intensity/=(light.u_size*light.v_size);
        *specular_intensity/=(light.u_size*light.v_size);
    }
}

static V3 trace_ray(V3 O, V3 D, uint r){
    Intersection intersection=ray_scene(O,D,(r==RECURSION_DEPTH?FIRST:REFLECTION),scene,arr_size(scene));
    if(intersection.closest_obj==NULL)
        return (V3){0,0,0};
    V3 d=v_sub((V3){0,0,0},D);
    V3 c;
    if(intersection.closest_obj->type==PLANE)
        c=plane_color(*intersection.closest_obj,intersection.p);
    else
        c=intersection.closest_obj->color;
    V3 diffuse_color=v_mul(c,ambient_light),specular_color={0,0,0},reflection_color={0,0,0};
    for(uint i=0;i<arr_size(light_sources);++i){
        float diffuse_intensity=0,specular_intensity=0;
        shade(&diffuse_intensity,&specular_intensity,intersection,light_sources[i],d);
        diffuse_color=v_add(diffuse_color,v_mul2(v_mul(c,diffuse_intensity*(1-intersection.closest_obj->reflection)),light_sources[i].intensity));
        specular_color=v_add(specular_color,v_mul2(v_mul((V3){255,255,255},specular_intensity),light_sources[i].intensity));
    }
    if(intersection.closest_obj->reflection>0&&r-->0)
        reflection_color=v_mul(trace_ray(intersection.p,v_sub(v_mul(intersection.n,2*v_dot(intersection.n,d)),d),r),
                                intersection.closest_obj->reflection);
    return v_add(v_add(diffuse_color,reflection_color),specular_color);
}

static inline void out_pic(char *fn){
    FILE *fp=fopen(fn,"wb");
    fprintf(fp,"P6\n%u %u\n255\n",WIDTH,HEIGHT);
    fwrite(pic,sizeof(Color),arr_size(pic),fp);
    fclose(fp);
}

#ifdef OUTPUT360
static void render_worker(uint begin, uint end, V3 *matrix){
    V3 m[3];
    for(;begin<end;++begin){
        V3 c={0,0,0};
        int a=begin%WIDTH;
        int b=begin/WIDTH+HEIGHT/2;
        for(uint i=0;i<SPP*SPP;++i){
            gen_matrix(((float)SPP*a+i%SPP)/(2*HEIGHT*SPP),(SPP*b+(float)i/SPP)/(2*HEIGHT*SPP),0,1,m);
            c=v_add(c,trace_ray(camera_pos,v_transform(v_transform((V3){0,0,1},m),matrix),RECURSION_DEPTH));
        }
        pic[begin]=saturate(v_mul(c,1.0f/(SPP*SPP)));
    }
}
#else
static void render_worker(uint begin, uint end, V3 *matrix){
    for(;begin<end;++begin){
        V3 c={0,0,0};
        int x=(begin%WIDTH)-WIDTH/2;
        int y=(begin/WIDTH)-HEIGHT/2;
        for(uint i=0;i<SPP*SPP;++i)
            c=v_add(c,trace_ray(camera_pos,v_transform(
                (V3){((float)SPP*x+i%SPP)/(HEIGHT*SPP),-(SPP*y+(float)i/SPP)/(HEIGHT*SPP),1},matrix),RECURSION_DEPTH));
        pic[begin]=saturate(v_mul(c,1.0f/(SPP*SPP)));
    }
}
#endif

int main(int argc, char **argv){
    for(uint i=0;i<arr_size(scene);++i)
        if(scene[i].type==MESH) load_stl(scene+i);
    V3 matrix[3];
    gen_matrix(yaw,pitch,roll,1,matrix);
#ifdef SDLPREVIEW
    pthread_t draw_thread;
    struct draw_args args={WIDTH,HEIGHT,pic};
    pthread_create(&draw_thread,NULL,(void*(*)(void*))draw_stuff,&args);
#endif
    start_work(render_worker,arr_size(pic),matrix);
    out_pic(argc>1?argv[1]:"pic.ppm");
    for(uint i=0;i<arr_size(scene);++i)
        if(scene[i].type==MESH) destroy_mesh(scene+i);
#ifdef SDLPREVIEW
    pthread_join(draw_thread,NULL);
#endif
    return 0;
}
