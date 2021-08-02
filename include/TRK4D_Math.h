#ifndef TRK4D_MATH_H
#define TRK4D_MATH_H
#include <math.h>
#ifndef MATH_Absi
#define MATH_Absi(a)                         ((((a) > 0) ? (a) : (-(a))))
#endif /* MATH_Absi */

#ifndef MATH_Absf
#define MATH_Absf                            fabsf
#endif /* MATH_Absf */

#ifndef MATH_Cos
#define MATH_Cos                             cosf
#endif /* MATH_Cos */

#ifndef MATH_Sin
#define MATH_Sin                             sinf
#endif /* MATH_Sin */

#ifndef MATH_Sqrt
#define MATH_Sqrt                            sqrtf
#endif /* MATH_Sqrt */

#ifndef MATH_Atan2f
#define MATH_Atan2f                          atan2f
#endif /* MATH_Atan2f */

#ifndef MATH_Atanf
#define MATH_Atanf                           atanf
#endif /* MATH_Atanf */

#ifndef MATH_Asin
#define MATH_Asin                            asinf
#endif /* MATH_Asin */

#ifndef MATH_Log10
#define MATH_Log10                           log10f
#endif /* MATH_Log10 */

#ifndef MATH_Log2
#define MATH_Log2                            log2f
#endif /* MATH_Log2 */

#ifndef MATH_Modf
#define MATH_Modf                            modff
#endif /* MATH_Modf */

#ifndef MATH_Powf
#define MATH_Powf                            powf
#endif /* MATH_Powf */
#endif
 
#ifndef MATH_Max
#define MATH_Max(a,b)                         ((a) > (b) ? (a) : (b))
#endif /* MATH_Max */

#ifndef MATH_Min
#define MATH_Min(a,b)                         ((a) < (b) ? (a) : (b))
#endif /* MATH_Max */

#define DEG2RAD 0.0174532925199433f
#define RAD2DEG 57.2957795130823f