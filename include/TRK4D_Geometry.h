#ifndef TRK4D_GEOMETRY_H
#define TRK4D_GEOMETRY_H
#include "TRK4D_Typedefs.h"
#include <stdbool.h>
#include "TRK4D_Math.h"

struct TRK4D_POINT3
{
    float x;
    float y;
    float z;
};

struct TRK4D_POINT2
{
    float x;
    float y;
};

#define TRK4D_BBOX_VERTEX_NUM 4

bool TRK4D_fn_isBBoxOverlap(TRK4D_POINT3 bbox_vertex_a[TRK4D_BBOX_VERTEX_NUM], TRK4D_POINT3 bbox_vertex_b[TRK4D_BBOX_VERTEX_NUM]);
#endif
