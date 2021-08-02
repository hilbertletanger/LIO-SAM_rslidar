#include "TRK4D_Geometry.h"

// static gp_s32 merged_cluster_id[TRK4D_CLUSTERLIST_NUM];


bool TRK4D_fn_isBBoxOverlap(TRK4D_POINT3 bbox_vertex_a[TRK4D_BBOX_VERTEX_NUM], TRK4D_POINT3 bbox_vertex_b[TRK4D_BBOX_VERTEX_NUM])
{
    TRK4D_POINT3 *bbox_t;
    TRK4D_POINT3 *bbox_another;
    TRK4D_POINT2 proj_axis;
    TRK4D_POINT2 edge_a_t;
    TRK4D_POINT2 edge_b_t;

    gp_f32 proj_t_max, proj_t_min, proj_another_max, proj_another_min;
    gp_f32 proj_t, proj_another;

    gp_u32 bbox_i;
    gp_u32 vertex_j;
    gp_u32 vertex_k;

    for (bbox_i = 0; bbox_i < 2; bbox_i++)
    {
        proj_another_max = GP_F32_MIN;
        proj_another_min = GP_F32_MAX;
        bbox_t = bbox_i == 0 ? bbox_vertex_a:bbox_vertex_b;
        bbox_another =  bbox_i == 0 ? bbox_vertex_b : bbox_vertex_a;
        
        for (vertex_j = 0; vertex_j < 2; vertex_j++)
        {
            proj_t_max = 0.f;
            proj_t_min = 0.f;
            proj_another_max = GP_F32_MIN;
            proj_another_min = GP_F32_MAX;

            proj_axis.x = -(bbox_t[vertex_j + 1].y - bbox_t[vertex_j].y);             
            proj_axis.y = bbox_t[vertex_j + 1].x - bbox_t[vertex_j].x;
            if (MATH_Absf(proj_axis.x) + MATH_Absf(proj_axis.y) == 0)
            {
                proj_axis.x = 1;
                proj_axis.y = 0;
            }

            proj_t = (bbox_t[vertex_j + 2].x - bbox_t[vertex_j].x) * proj_axis.x + (bbox_t[vertex_j + 2].y - bbox_t[vertex_j].y)* proj_axis.y;
            proj_t_max = MATH_Max(proj_t_max, proj_t);
            proj_t_min = MATH_Min(proj_t_min, proj_t);
            for (vertex_k = 0; vertex_k < TRK4D_BBOX_VERTEX_NUM; vertex_k++)
            {
                proj_another = proj_axis.x * (bbox_another[vertex_k].x - bbox_t[vertex_j].x) + proj_axis.y * (bbox_another[vertex_k].y - bbox_t[vertex_j].y);
                proj_another_min = MATH_Min(proj_another_min, proj_another);
                proj_another_max = MATH_Max(proj_another_max, proj_another);
            }
            
            if (proj_t_max < proj_another_min || proj_t_min > proj_another_max)
            {
                return false;
            }   
        }
    }
    return true;     
}

bool TRK4D_fn_isHullOverlap(TRK4D_POINT3 *convex_hull_a, gp_u32 vertex_a_num, TRK4D_POINT3 *convex_hull_b, gp_u32 vertex_b_num)
{
    TRK4D_POINT3 *hull_t;
    TRK4D_POINT3 *hull_another;
    TRK4D_POINT2 proj_axis;
    TRK4D_POINT2 edge_a_t;
    TRK4D_POINT2 edge_b_t;
    gp_u32 vertex_num_t;
    gp_u32 vertex_num_another;

    gp_f32 proj_t_max, proj_t_min, proj_another_max, proj_another_min;
    gp_f32 proj_t, proj_another;

    gp_u32 hull_i;
    gp_u32 vertex_j;
    gp_u32 vertex_k;

    for (hull_i = 0; hull_i < 2; hull_i++)
    {
        proj_another_max = GP_F32_MIN;
        proj_another_min = GP_F32_MAX;
        hull_t = hull_i == 0 ? convex_hull_a: convex_hull_b;
        vertex_num_t = hull_i == 0 ? vertex_a_num : vertex_b_num;
        hull_another =  hull_i == 0 ? convex_hull_b : convex_hull_a;
        vertex_num_another = hull_i == 0 ? vertex_b_num : vertex_a_num;
        
        for (vertex_j = 0; vertex_j < vertex_num_t; vertex_j++)
        {
            proj_t_max = 0.f;
            proj_t_min = 0.f;
            proj_another_max = GP_F32_MIN;
            proj_another_min = GP_F32_MAX;

            proj_axis.x = hull_t[(vertex_j + 1) % vertex_num_t].y - hull_t[vertex_j].y;             
            proj_axis.y = hull_t[(vertex_j + 1) % vertex_num_t].x - hull_t[vertex_j].x;
            for (vertex_k = 0; vertex_k < vertex_num_t; vertex_k++)
            {
                proj_t = (hull_t[vertex_k].x - hull_t[vertex_j].x) * proj_axis.x + (hull_t[vertex_k].y - hull_t[vertex_j].y)* proj_axis.y;
                proj_t_max = MATH_Max(proj_t_max, proj_t);
                proj_t_min = MATH_Min(proj_t_min, proj_t);
            }

            for (vertex_k = 0; vertex_k < TRK4D_BBOX_VERTEX_NUM; vertex_k++)
            {
                proj_another = proj_axis.x * (hull_another[vertex_k].x - hull_t[vertex_j].x) + proj_axis.y * (hull_another[vertex_k].y - hull_t[vertex_j].y);
                proj_another_min = MATH_Min(proj_another_min, proj_another);
                proj_another_max = MATH_Max(proj_another_max, proj_another);
            }
            
            if (proj_t_max < proj_another_min || proj_t_min > proj_another_max)
            {
                return false;
            }   
        }
    }
    return true; 
}

//bool TRK4D_fn_mergeClusterByOverlap(TRK4D_RADARPOINT_FUSION_LIST *radarpoint_list_a, TRK4D_CLUSTER_LIST * cluster_dict_a)
//{
//   TRK4D_CLUSTER *ptr_cluster_i;
//   TRK4D_CLUSTER *ptr_cluster_j;
//   for (gp_u32 i = 0; i < cluster_dict_a.valid_num; i++)
//   {
//        ptr_cluster_i = &cluster_dict_a->cluster_list[i];
//        for (gp_u32 j = i + 1; j < cluster_dict_a.valid_num; j++)
//        { 
//            ptr_cluster_j = &cluster_dict_a->cluster_list[j];
//            if (TRK4D_fn_isHullOverlap(ptr_cluster_i->hull.hull_vertex, ptr_cluster_i->hull.hull_vertex_num, ptr_cluster_j->hull.hull_vertex, ptr_cluste_j->hull.hull_vertex_num))
//            {
//
//            }
//        }
//   }
//}
