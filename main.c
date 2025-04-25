#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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

typedef struct {
    f64 x, y,
        mass,
        velocityX,
        velocityY;
    i32 radius;
    u32 color;
} Planet;

struct {
    SDL_Window* window;
    SDL_Renderer* renderer;
    Planet planets[NUM_PLANETS];
} state;

void draw_planet(Planet* p) {
    i32 drawRadius = draw_radius(p);
    if (drawRadius < 3) drawRadius = 3;

    SDL_SetRenderDrawColor(
        state.renderer,
        r(p), g(p), b(p), a(p)
    );

    for (i32 w = -drawRadius; w <= drawRadius; w++)
        for (i32 h = -drawRadius; h <= drawRadius; h++)
            if (w * w + h * h <= drawRadius * drawRadius)
                SDL_RenderDrawPoint(state.renderer,
                    (i32)p->x + w,
                    (i32)p->y + h
                );
}

void update_position(Planet* p, f64 ax, f64 ay, f64 dt) {
    p->velocityX += mult(ax, dt);
    p->velocityY += mult(ay, dt);
    p->x += mult(p->velocityX, dt);
    p->y += mult(p->velocityY, dt);
}

void draw_grav_field() {
    SDL_SetRenderDrawColor(
        state.renderer,
        211, 211, 211, 255
    );
    for (int x = 0; x < WIDTH; x += SPACING) {
        for (int y = 0; y < HEIGHT; y += SPACING) {
            f64 totalFx = 0,
                totalFy = 0;

            for (int i = 0; i < NUM_PLANETS; i++) {
                f64 dx = state.planets[i].x - x,
                    dy = state.planets[i].y - y,
                    dist = sqrt(dx * dx + dy * dy);
                if (dist > MAX_INFLUENCE || dist < 1) continue;

                f64 force = G * state.planets[i].mass / (dist * dist),
                    angle = atan2(dy, dx);
                totalFx += force * cos(angle);
                totalFy += force * sin(angle);
            }

            double mag = sqrt(totalFx * totalFx + totalFy * totalFy);
            if (mag > 0) {
                if (mag > MAX_VECTOR_LENGTH) mag = MAX_VECTOR_LENGTH;
                f64 nx = totalFx / sqrt(totalFx * totalFx + totalFy * totalFy),
                    ny = totalFy / sqrt(totalFx * totalFx + totalFy * totalFy);
                int endX = (int)(x + nx * mag);
                int endY = (int)(y + ny * mag);
                SDL_RenderDrawLine(state.renderer
                    , x, y, endX, endY
                );
            }
        }
    }
}

void calculate_gravity(Planet* p1, Planet* p2, \
    double* ax1, double* ay1, \
    double* ax2, double* ay2) {
    double dx = delta_x(p1, p2);
    double dy = delta_y(p1, p2);
    double dist = dist(dx, dy);
    if (dist < 1) dist = 1;

    double force = force(p1, p2, dist);
    double angle = angle(dy, dx);
    *ax1 = axc(p1, force, angle);
    *ay1 = axs(p1, force, angle);
    *ax2 = ayc(p1, force, angle);
    *ay2 = ays(p1, force, angle);
}

void reset_planets() {
    state.planets[0].x = centerX - OFFSET;
    state.planets[0].y = centerY;
    state.planets[0].velocityX = 0;
    state.planets[0].velocityY = 10;

    state.planets[1].x = centerX + OFFSET;
    state.planets[1].y = centerY;
    state.planets[1].velocityX = 0;
    state.planets[1].velocityY = -10;
}

void simulate_step() {
    double ax1, ay1, ax2, ay2;
    calculate_gravity(&state.planets[0], &state.planets[1], &ax1, &ay1, &ax2, &ay2);

    update_position(&state.planets[0], ax1, ay1, TIME_STEP);
    update_position(&state.planets[1], ax2, ay2, TIME_STEP);
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);

    state.window = SDL_CreateWindow(
        "WINDOW",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT,
        SDL_WINDOW_RESIZABLE);
    state.renderer = SDL_CreateRenderer(
        state.window,
        -1,
        SDL_RENDERER_ACCELERATED);

    // Initialize planets
    state.planets[0] = (Planet){
        centerX - OFFSET, centerY,
        1e15, 0, 10,
        log_radius(1e15), CCOLOR};
    state.planets[1] = (Planet){
        centerX + OFFSET, centerY,
        1e15, 0, -10,
        log_radius(1e15), CCOLOR};

    Uint32 lastTime = SDL_GetTicks();

    while (1) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) goto done;
            if (e.type == SDL_WINDOWEVENT && \
                e.window.event == SDL_WINDOWEVENT_RESIZED)
                reset_planets();
        }

        Uint32 current = SDL_GetTicks();
        if (current - lastTime >= 4) {
            simulate_step();
            lastTime = current;

            SDL_SetRenderDrawColor(state.renderer,
                255, 255, 255, 255
            );
            SDL_RenderClear(state.renderer);

            draw_grav_field();

            for (int i = 0; i < NUM_PLANETS; i++)
                draw_planet(&state.planets[i]);

            SDL_RenderPresent(state.renderer);
        }
    }

    done:
    SDL_DestroyRenderer(state.renderer);
    SDL_DestroyWindow(state.window);
    SDL_Quit();
    return 0;
}