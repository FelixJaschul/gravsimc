#include <SDL2/SDL.h>
#include <math.h>
#include "main.h"

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

void draw_planet(const Planet* p) {
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

void update_position(Planet* p, const f64 ax, const f64 ay, const f64 dt) {
    p->velocityX += mult(ax, dt);
    p->velocityY += mult(ay, dt);
    p->x += mult(p->velocityX, dt);
    p->y += mult(p->velocityY, dt);
}

void draw_grav_field() {
    SDL_SetRenderDrawColor(state.renderer,
        211, 211, 211, 255
    );
    for (int x = 0; x < WIDTH; x += SPACING) {
        for (int y = 0; y < HEIGHT; y += SPACING) {
            f64 totalFx = 0;
            f64 totalFy = 0;

            for (int i = 0; i < NUM_PLANETS; i++) {
                const f64 dx = state.planets[i].x - x;
                const f64 dy = state.planets[i].y - y;
                const f64 dist = sqrt(dx * dx + dy * dy);
                if (dist > MAX_INFLUENCE || dist < 1) continue;

                const f64 force = G * state.planets[i].mass / (dist * dist);
                const f64 angle = atan2(dy, dx);
                totalFx += force * cos(angle);
                totalFy += force * sin(angle);
            }

            double mag = sqrt(
                totalFx * totalFx + \
                totalFy * totalFy);
            if (mag > 0) {
                if (mag > MAX_VECTOR_LENGTH) mag = MAX_VECTOR_LENGTH;
                const f64 nx = totalFx / sqrt(totalFx * totalFx + totalFy * totalFy);
                const f64 ny = totalFy / sqrt(totalFx * totalFx + totalFy * totalFy);
                const int endX = (int)(x + nx * mag);
                const int endY = (int)(y + ny * mag);
                SDL_RenderDrawLine(state.renderer
                    , x, y, endX, endY
                );
            }
        }
    }
}

void calculate_gravity(Planet* p1, Planet* p2, \
    f64* ax1, f64* ay1, \
    f64* ax2, f64* ay2) {
    const f64 dx = delta_x(p2, p1);
    const f64 dy = delta_y(p2, p1);
    double dist = dist(dx, dy);
    if (dist < 1) dist = 1;

    const double force = force(p1, p2, dist);
    const double angle = angle(dy, dx);
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
    f64 ax1, ay1, ax2, ay2;
    calculate_gravity(
        &state.planets[0],
        &state.planets[1],
        &ax1, &ay1, &ax2, &ay2);

    update_position(
        &state.planets[0],
        ax1, ay1,
        TIME_STEP);
    update_position(
        &state.planets[1],
        ax2, ay2,
        TIME_STEP);
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);

    state.window =
        SDL_CreateWindow(
        "WINDOW",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT,
        SDL_WINDOW_RESIZABLE);
    state.renderer =
        SDL_CreateRenderer(
        state.window, -1,
        SDL_RENDERER_ACCELERATED);

    // Initialize planets
    state.planets[0] =
        (Planet) {
        centerX - OFFSET,
        centerY,
        1e15,
        0, +10,
        log_radius(1e15),
        CCOLOR };

    state.planets[1] =
        (Planet) {
        centerX + OFFSET,
        centerY,
        1e15,
        0, -10,
        log_radius(1e15),
        CCOLOR };

    Uint32 t0 = SDL_GetTicks();

    while (1) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) goto done;
            if (e.type == SDL_WINDOWEVENT \
            && e.window.event \
            == SDL_WINDOWEVENT_RESIZED)
                reset_planets();
        }

        const Uint32 t1 = SDL_GetTicks();
        if (t1 - t0 >= 4) {
            simulate_step();
            t0 = t1;

            SDL_SetRenderDrawColor(state.renderer,
                255, 255, 255, 255);
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