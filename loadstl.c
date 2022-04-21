#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "raytrasdf.h"

#define vert(n) (triangles_buf[(n)/3].obj.triangle.vertices[(n)%3])
static inline void vertex_normals(Obj *triangles_buf, uint triangle_n, float smooth){
    uint n=triangle_n*3;
    uint *s=malloc(sizeof(uint)*n);
    assert(s!=NULL);
    for(uint i=0;i<n;++i){
        vert(i).normal=triangles_buf[i/3].obj.triangle.normal;
        s[i]=0;
    }
    if(smooth>0){
        V3 **buf0=malloc(sizeof(V3*)*n);
        V3 *buf1=malloc(sizeof(V3)*n);
        assert(buf0!=NULL&&buf1!=NULL);
        for(uint i=0;i<n;++i){
            if(s[i]==0){
                for(uint j=i;j<n;++j)
                    if(v_cmp(vert(i).pos,vert(j).pos))
                        ++s[i];
                buf0[i]=buf1;
                buf1+=s[i];
                uint c=0;
                for(uint j=i;c<s[i];++j){
                    if(v_cmp(vert(i).pos,vert(j).pos)){
                        buf0[i][c++]=vert(j).normal;
                        buf0[j]=buf0[i];
                        s[j]=s[i];
                    }
                }
            }
        }
        for(uint i=0;i<n;++i){
            V3 v={0,0,0};
            for(uint j=0;j<s[i];++j)
                if(smooth>=1.0f||((v_dot(buf0[i][j],vert(i).normal)+1)/2)>(1-smooth))
                    v=v_add(v,buf0[i][j]);
            vert(i).normal=v_norm(v);
        }
        free(buf0);
        free(buf1-n);
    }
    free(s);
}

static Octree_node *gen_octree_node(Obj bounding, Obj **triangles_buf, uint *triangle_n, uint od){
    Octree_node *new_node=NULL;
    new_node=malloc(sizeof(Octree_node));
    assert(new_node!=NULL);
    new_node->triangle_n=0;
    new_node->bounding=bounding;
    new_node->bounding.type=BOX;
    bool empty=true;
    if(od--){
        union{
            V3 v;
            float v_arr[3];
        }v0,t;
        Obj new_aabb;
        t.v=v_mul(v_sub(bounding.obj.aabb_v1,bounding.pos),.5f);
        for(uint i=0;i<8;++i){
            v0.v=bounding.pos;
            for(uint j=0;j<3;++j)
                if((i>>j)&1)
                    v0.v_arr[j]+=t.v_arr[j];
            new_aabb.pos=v0.v;
            new_aabb.obj.aabb_v1=v_add(v0.v,t.v);
            new_node->c_nodes[i]=gen_octree_node(new_aabb,triangles_buf,triangle_n,od);
            if(new_node->c_nodes[i]!=NULL) empty=false;
        }
    }
    else{
        for(uint i=0;i<8;++i) new_node->c_nodes[i]=NULL;
    }
    new_node->triangles=*triangles_buf;
    for(uint i=0;i<*triangle_n;++i){
        for(uint j=0;j<=3;++j){
            if(j==3){
                Obj temp=new_node->triangles[i];
                new_node->triangles[i]=**triangles_buf;
                *((*triangles_buf)++)=temp;
                ++new_node->triangle_n;
            }
            else if(!within_box(bounding,new_node->triangles[i].obj.triangle.vertices[j].pos))
                break;
        }
    }
    *triangle_n-=new_node->triangle_n;
    if(empty&&new_node->triangle_n==0){
        free(new_node);
        new_node=NULL;
    }
    return new_node;
}

static void destroy_octree_node(Octree_node *node){
    for(uint i=0;i<8;++i)
        if(node->c_nodes[i]!=NULL)
            destroy_octree_node(node->c_nodes[i]);
    free(node);
}

void destroy_mesh(Obj *obj){
    if(obj->obj.mesh.triangles_buf!=NULL){
        free(obj->obj.mesh.triangles_buf);
        destroy_octree_node(obj->obj.mesh.root);
    }
}

void load_stl(Obj *obj){
    FILE *fp=fopen(obj->obj.mesh.file_name,"rb");
    assert(fp!=NULL);
    fseek(fp,80,SEEK_SET);
    uint32_t triangle_n;
    assert(fread(&triangle_n,4,1,fp));
    if(triangle_n>0){
        Obj *triangles_buf=malloc(triangle_n*sizeof(Obj));
        assert(triangles_buf!=NULL);
        obj->obj.mesh.triangles_buf=triangles_buf;
        V3 v_pos;
        V3 matrix[3];
        gen_matrix(obj->obj.mesh.yaw,obj->obj.mesh.pitch,obj->obj.mesh.roll,obj->obj.mesh.scale,matrix);
        union{
            V3 v;
            float v_arr[3];
        }v0,v1,temp;
        for(uint i=0;i<triangle_n;++i){
            triangles_buf[i].type=MESH_TRIANGLE;
            assert(fread(&triangles_buf[i].obj.triangle.normal,sizeof(V3),1,fp));
            triangles_buf[i].obj.triangle.normal=v_norm(v_transform(triangles_buf[i].obj.triangle.normal,matrix));
            for(uint j=0;j<3;++j){
                assert(fread(&v_pos,sizeof(V3),1,fp));
                triangles_buf[i].obj.triangle.vertices[j].pos=v_add(v_transform(v_pos,matrix),obj->pos);
                if(j==0&&i==0){
                    v0.v=v_sub(triangles_buf[0].obj.triangle.vertices[0].pos,(V3){.05,.05,.05});
                    v1.v=v_add(triangles_buf[0].obj.triangle.vertices[0].pos,(V3){.05,.05,.05});
                }
                temp.v=triangles_buf[i].obj.triangle.vertices[j].pos;
                for(uint k=0;k<3;++k){
                    v0.v_arr[k]=min(v0.v_arr[k],temp.v_arr[k]);
                    v1.v_arr[k]=max(v1.v_arr[k],temp.v_arr[k]);
                }
            }
            triangles_buf[i].obj.triangle.area=v_len(v_cross(v_sub(triangles_buf[i].obj.triangle.vertices[0].pos,
                                                                     triangles_buf[i].obj.triangle.vertices[1].pos),
                                                                     v_sub(triangles_buf[i].obj.triangle.vertices[0].pos,
                                                                     triangles_buf[i].obj.triangle.vertices[2].pos)));
            fseek(fp,2,SEEK_CUR);
        }
        fclose(fp);
        vertex_normals(triangles_buf,triangle_n,obj->obj.mesh.smooth);
        Obj new_aabb;
        new_aabb.pos=v0.v;
        new_aabb.obj.aabb_v1=v1.v;
        obj->obj.mesh.root=gen_octree_node(new_aabb,&triangles_buf,&triangle_n,obj->obj.mesh.octree_depth);
    }
    else{
        fclose(fp);
        obj->obj.mesh.triangles_buf=NULL;
        obj->obj.mesh.root=NULL;
    }
}
