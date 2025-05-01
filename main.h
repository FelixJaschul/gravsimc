#pragma once

typedef double          f64;
typedef float           f32;
typedef int             i32;
typedef Uint32          u32;

#define WIDTH           800
#define HEIGHT          600

#define G               6.67440e-11f
#define OFFSET          100
#define TIME_STEP       0.05

#define SPACING         15
#define MAX_VECTOR_LENGTH \
50
#define MAX_INFLUENCE   300

#define VCOLOR          0xD3D3D3FF
#define CCOLOR          0xD3D3D3FF
#define BCOLOR          0xFFFFFFFF

#define NUM_PLANETS     2

#define centerX         ({ WIDTH  / 2; })
#define centerY         ({ HEIGHT / 2; })
#define log_radius(a)   ({ (i32)(log(a * 2)); })
#define draw_radius(p)  ({ p->radius; })
#define mult(a, c)      ({ a * c; })
#define delta_x(p, b)   ({ p->x - b->x; })
#define delta_y(p, b)   ({ p->y - b->y; })
#define dist(a, c)      ({ sqrt(a * a + c * c); })
#define force(p, b, c)  ({ G * p->mass * b->mass / (c * c); })
#define angle(a, b)     ({ atan2(a, b); })
#define axc(p, a, c)    ({  a * cos(c) / p->mass; })
#define axs(p, a, c)    ({  a * sin(c) / p->mass; })
#define ayc(p, a, c)    ({ -a * cos(c) / p->mass; })
#define ays(p, a, c)    ({ -a * sin(c) / p->mass; })

#define r(p)            ({ (p->color >> 24) & 0xFF; })
#define g(p)            ({ (p->color >> 16) & 0xFF; })
#define b(p)            ({ (p->color >> 8)  & 0xFF; })
#define a(p)            ({ (p->color & 0xFF); })
