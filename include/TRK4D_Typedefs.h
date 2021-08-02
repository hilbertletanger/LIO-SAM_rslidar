#ifndef TRK4D_TYPEDEFS_H
#define TRK4D_TYPEDEFS_H
typedef signed char gp_s8;
typedef unsigned char gp_u8;
typedef signed short gp_s16;
typedef signed int gp_s32;
typedef signed long gp_s64;
typedef unsigned short gp_u16;
typedef unsigned int gp_u32;
typedef unsigned long gp_u64;
typedef float gp_f32;
typedef double gp_d64;
#ifndef GP_NULL
#define GP_NULL (void*)0;
#endif
#ifndef GP_F32_EPSILON
#define GP_F32_EPSILON   1.175494351e-10f
#endif 
#ifndef GP_F32_MAX
#define GP_F32_MAX 3.402823466e+38f
#endif
#ifndef GP_F32_MIN
#define GP_F32_MIN -3.402823466e+38F
#endif
#ifndef GP_U32_MAX
#define GP_U32_MAX 0xFFFFFFFF
#endif
#endif 
