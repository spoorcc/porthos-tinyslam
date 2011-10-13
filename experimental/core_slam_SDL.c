/* CoreSLAM Test
 * (c) Mines ParisTech 2010-2011 */

// Paths to add to VC++ :
// d:\home\steux\corebots\third_parties\SDL\win32\include;d:\home\steux\corebots\third_parties\SDL_gfx\win32\include

#include "core_slam.h"
#include "core_slam_client.h"
#include "core_control.h"
#ifdef WIN32
#include "SDL.h"
#include "SDL_thread.h"
#include "SDL_rotozoom.h"
#else
#include "SDL/SDL.h"
#include "SDL/SDL_thread.h"
#include "SDL/SDL_rotozoom.h"
#endif

unsigned char *cs_SDL_pixmap;
SDL_Surface *cs_SDL_screen, *cs_SDL_pixmap_surface, *cs_SDL_arrow_up, *cs_SDL_logo, *cs_SDL_target;
SDL_cond *cs_SDL_draw_it;
SDL_mutex *cs_SDL_drawing_mutex;
SDL_Thread *cs_SDL_thread;
volatile int cs_SDL_draw_mode;
volatile int cs_SDL_done_flag;
int cs_SDL_halted;
int cs_SDL_step_by_step;
int cs_SDL_working_with_client;
int cs_SDL_map_size, cs_SDL_resxy, cs_SDL_laser_offset;
double cs_SDL_map_origin_x, cs_SDL_map_origin_y;
void *cs_SDL_state;
cc_state_t *cs_SDL_cc_state;
unsigned char *cs_SDL_map, *cs_SDL_dmap, *cs_SDL_debug_map;
int cs_SDL_xclick, cs_SDL_yclick, cs_SDL_there_was_a_click_on_the_map;
int cs_SDL_there_is_a_target = 0;
volatile int cs_SDL_there_is_a_new_target = 0;
volatile double cs_SDL_xtarget, cs_SDL_ytarget;
volatile int cs_SDL_direction = 0;
volatile int cs_SDL_changing_direction = 0;

const int DISPLAY_SIZE = 768;

void
cs_draw_scan_RGB(cs_scan_t *scan, cs_position_t *pos, unsigned char *pixmap, int map_size, int resxy, int reversey)
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2;
    uint32_t *ptr;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * 1000 / resxy + 0.5);
    y1 = (int)floor(pos->y * 1000 / resxy + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i < scan->nb_points; i++) {
        if (scan->points[i].value) {
            x2p = c * scan->points[i].x - s * scan->points[i].y;
            y2p = s * scan->points[i].x + c * scan->points[i].y;
            x2 = (int)floor((pos->x * 1000 + x2p) / resxy + 0.5);
            y2 = (int)floor((pos->y * 1000 + y2p) / resxy + 0.5);
            if (x2 >= 0 && y2 >= 0 && x2 < map_size && y2 < map_size) {
                if (reversey)
                    ptr = (uint32_t*)&pixmap[((map_size - 1 - y2) * map_size + x2) * 4];
                else
                    ptr = (uint32_t*)&pixmap[(y2 * map_size  + x2) * 4];
                *ptr = 255 << 16;
            }
        }
    }
}

int
cs_SDL_drawing_thread(void *c)
{
    cs_map_pixel_t *ptr;
    int w, x, y;
    double xr, xrp, yr, yrp, theta;
    SDL_Rect rect;
    SDL_Surface *rotate;
    cs_position_t position;

    do {

        SDL_LockMutex(cs_SDL_drawing_mutex);
        SDL_CondWait(cs_SDL_draw_it, cs_SDL_drawing_mutex);

#define SCALE 1
#define ZOOM 1

        if (cs_SDL_draw_mode == 0) {
            if (cs_SDL_working_with_client) {
                for (y = 0; y < cs_SDL_map_size / SCALE; y++) {
                    uint32_t *ptrpmap = (uint32_t*)&cs_SDL_pixmap[y * cs_SDL_map_size / SCALE * 4];
                    ptr = &(cs_SDL_dmap[(cs_SDL_map_size - 1 - y * SCALE) * cs_SDL_map_size]);
                    for (x = 0; x < cs_SDL_map_size / SCALE; x++, ptr += SCALE /*ptr++*/) {
                        if (*ptr == 255)
                            *(ptrpmap++) = (128 << 16) + (128 << 8) + 128;
                        else if (*ptr == 1)
                            *(ptrpmap++) = 255 << 8;
                        else if (*ptr == 0)
                            *(ptrpmap++) = 255;
                        else {
                            unsigned char pixval = (*ptr) << 1;
                            uint32_t value = (((pixval << 8) + pixval) << 8) + pixval;
                            *(ptrpmap++) = value;
                        }
                    }
                }
            } else {
                for (y = 0; y < cs_SDL_map_size / SCALE; y++) {
                    char *ptrx = &(cs_SDL_map[(cs_SDL_map_size - 1 - y * SCALE) * cs_SDL_map_size]);
                    uint32_t *ptrpmap = (uint32_t*)&cs_SDL_pixmap[y * cs_SDL_map_size / SCALE * 4];
                    for (x = 0; x < cs_SDL_map_size / SCALE; x++, ptrx += SCALE) {
                        char pixval = *ptrx;
                        uint32_t value = (((pixval << 8) + pixval) << 8) + pixval;
                        *(ptrpmap++) = value;
                    }
                }
            }
        } else {
            if (cs_SDL_debug_map) {
                for (y = 0; y < cs_SDL_map_size / SCALE; y++) {
/*
                    char *ptrx = &(cs_SDL_debug_map[(cs_SDL_map_size - 1 - y * SCALE) * cs_SDL_map_size]);
                    uint32_t *ptrpmap = (uint32_t*)&cs_SDL_pixmap[y * cs_SDL_map_size / SCALE * 4];
                    for (x = 0; x < cs_SDL_map_size / SCALE; x++, ptrx += SCALE) {
                        char pixval = *ptrx;
                        uint32_t value = (((pixval << 8) + pixval) << 8) + pixval;
                        *(ptrpmap++) = value;
                    }
*/
                    uint32_t *ptrpmap = (uint32_t*)&cs_SDL_pixmap[y * cs_SDL_map_size / SCALE * 4];
                    ptr = &(cs_SDL_debug_map[(cs_SDL_map_size - 1 - y * SCALE) * cs_SDL_map_size]);
                    for (x = 0; x < cs_SDL_map_size / SCALE; x++, ptr += SCALE /*ptr++*/) {
                        if (*ptr == 255)
                            *(ptrpmap++) = (128 << 16) + (128 << 8) + 128;
                        else if (*ptr == 1)
                            *(ptrpmap++) = 255 << 8;
                        else if (*ptr == 0)
                            *(ptrpmap++) = 255;
                        else {
                            unsigned char pixval = (*ptr) << 1;
                            uint32_t value = (((pixval << 8) + pixval) << 8) + pixval;
                            *(ptrpmap++) = value;
                        }
                    }
                }
            }
        }

        // xr, yr : position of the robot with respect to the center of the map (in mm)
        if (cs_SDL_working_with_client) 
            position = cs_client_get_robot_position((cs_client_state_t*)cs_SDL_state);
        else
            position = cs_get_robot_position((cs_state_t*)cs_SDL_state);

        // Position is an absolute position. Move it to a local map position
        position.x -= cs_SDL_map_origin_x; position.y -= cs_SDL_map_origin_y;
        xr = position.x * 1000 - 0.5 * cs_SDL_map_size * cs_SDL_resxy;
        yr = position.y * 1000 - 0.5 * cs_SDL_map_size * cs_SDL_resxy;
        theta = (-position.theta + 90) * M_PI / 180;
        // xrp, yrp : rotated vector of robot in the map (in pixels)
        xrp = xr * cos(theta) - yr * sin(theta);
        yrp = xr * sin(theta) + yr * cos(theta);
        xrp *= (double)(ZOOM / SCALE) / cs_SDL_resxy; yrp *= (double)(ZOOM / SCALE) / cs_SDL_resxy; // Scale to rotate surface size

        if (!cs_SDL_working_with_client) {
            // Draw current scan
            cs_scan_t *scan;
            if (scan = cs_get_scan((cs_state_t*)cs_SDL_state)) {
                cs_position_t pos2 = position;
                pos2.x += cs_SDL_laser_offset * 0.001 * cos(position.theta * M_PI / 180);
                pos2.y += cs_SDL_laser_offset * 0.001 * sin(position.theta * M_PI / 180);
                cs_draw_scan_RGB(scan, &pos2, cs_SDL_pixmap, cs_SDL_map_size / SCALE, cs_SDL_resxy * SCALE, 1);
            }
        }
        if (!cs_SDL_working_with_client) {
            // Draw covariance ellipse
            cs_state_t *state = (cs_state_t*)cs_SDL_state;
            int xc = (int)floor(position.x * 1000 / (cs_SDL_resxy * SCALE) + 0.5);
            int yc = (int)floor(position.y * 1000 / (cs_SDL_resxy * SCALE) + 0.5);
            cs_draw_covariance_matrix_RGBA(cs_SDL_pixmap, cs_SDL_map_size / SCALE, cs_SDL_map_size / SCALE, cs_get_covariance(state), xc, yc, 1000, (255 << 8));
        }

        // SDL code :
        rotate = rotozoomSurface(cs_SDL_pixmap_surface, -position.theta + 90, ZOOM, 0);
        w = -(rotate->w / 2 - DISPLAY_SIZE / 2);

        rect.x = (int)floor(w - xrp + 0.5);
        rect.y = (int)floor(w + yrp + 0.5);
        SDL_FillRect(cs_SDL_screen, NULL, 0);
        SDL_BlitSurface(rotate, NULL, cs_SDL_screen, &rect);

        // Mouse click management
        if (cs_SDL_there_was_a_click_on_the_map) {
            int xc = cs_SDL_xclick, yc = cs_SDL_yclick;
            double xmap, ymap;
            // Move to the rotated map coordinates (in pixels, upper left origin)
            fprintf(stderr, "Click position on screen (in pixels, upper left origin): %d, %d\n", xc, yc);
            xc -= (int)floor(w - xrp + 0.5); yc -= (int)floor(w + yrp + 0.5);
            fprintf(stderr, "Click position in the rotated map (in pixels, upper left origin): %d, %d\n", xc, yc);
            // Move to the rotated map coordinates (in pixels, center of map origin)
            xc -= rotate->w / 2; yc = -(yc - rotate->w / 2);
            fprintf(stderr, "Click position in the rotated map (in pixels, centered): %d, %d\n", xc, yc);
            // Unrotate to the original map
            xmap = xc * cos(-theta) - yc * sin(-theta);
            ymap = xc * sin(-theta) + yc * cos(-theta);
            fprintf(stderr, "Click position in the map : %lg, %lg\n", xmap, ymap);
            xmap /= ZOOM / (double)(SCALE * cs_SDL_resxy); ymap /= ZOOM / (double)(SCALE * cs_SDL_resxy); // Scale to map
            xmap += cs_SDL_map_origin_x * 1000 + 0.5 * cs_SDL_map_size * cs_SDL_resxy;
            ymap += cs_SDL_map_origin_y * 1000 + 0.5 * cs_SDL_map_size * cs_SDL_resxy;
            fprintf(stderr, "Click position in the map : %lg, %lg\n", xmap, ymap);
            cs_SDL_xtarget = (int)xmap * 0.001; cs_SDL_ytarget = (int)ymap * 0.001;
            cs_SDL_there_is_a_target = 1;
            cs_SDL_there_is_a_new_target = 1;
            cs_SDL_there_was_a_click_on_the_map = 0;
        }
        if (cs_SDL_there_is_a_target) {
            double xmap = cs_SDL_xtarget * 1000, ymap = cs_SDL_ytarget * 1000, xr, yr;
            xmap -= cs_SDL_map_origin_x * 1000 + 0.5 * cs_SDL_map_size * cs_SDL_resxy;
            ymap -= cs_SDL_map_origin_y * 1000 + 0.5 * cs_SDL_map_size * cs_SDL_resxy;
            xmap *= ZOOM / (double)(cs_SDL_resxy * SCALE); ymap *= ZOOM / (double)(cs_SDL_resxy * SCALE); // Scale to map
            xr = xmap * cos(theta) - ymap * sin(theta);
            yr = xmap * sin(theta) + ymap * cos(theta);
            // xr, yr : Rotated to map. Centered
            xr += rotate->w / 2; yr = -yr + rotate->w / 2;
            xr += (int)floor(w - xrp + 0.5); yr += (int)floor(w + yrp + 0.5);
            rect.x = (int)floor(xr - cs_SDL_target->w / 2 + 0.5);
            rect.y = (int)floor(yr - cs_SDL_target->h / 2 + 0.5);
            SDL_BlitSurface(cs_SDL_target, NULL, cs_SDL_screen, &rect);
        }
        SDL_FreeSurface(rotate);

        rect.x = DISPLAY_SIZE / 2 - cs_SDL_arrow_up->w / 2;
        rect.y = DISPLAY_SIZE / 2 - cs_SDL_arrow_up->h / 2;
        SDL_BlitSurface(cs_SDL_arrow_up, NULL, cs_SDL_screen, &rect);

        rect.x = 0;
        rect.y = 0;
        SDL_BlitSurface(cs_SDL_logo, NULL, cs_SDL_screen, &rect);

        SDL_Flip(cs_SDL_screen);
        SDL_UnlockMutex(cs_SDL_drawing_mutex);
    } while (!cs_SDL_done_flag);
    return 0;
}

int
cs_SDL_draw()
{
    SDL_CondSignal(cs_SDL_draw_it);
    return 1;
}

int
cs_SDL_process_events()
{
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    fprintf(stderr, "Map switch !\n");
                    cs_SDL_draw_mode++;
                    if (cs_SDL_draw_mode == 2) 
                        cs_SDL_draw_mode = 0;
                    SDL_CondSignal(cs_SDL_draw_it);
                }
                if (event.key.keysym.sym == SDLK_UP) {
                    fprintf(stderr, "Halt !\n");
                    cs_SDL_halted = 1 - cs_SDL_halted;
                }
                if (event.key.keysym.sym == SDLK_DOWN) {
                    fprintf(stderr, "Step !\n");
                    cs_SDL_step_by_step = 1;
                }
                if (event.key.keysym.sym == SDLK_SPACE) {
                    cs_SDL_direction = 1 - cs_SDL_direction;
                    cs_SDL_changing_direction = 1;
                    fprintf(stderr, "Changing direction!\n");
                }
                if (event.key.keysym.sym == SDLK_PAGEUP) {
                    if (cs_SDL_cc_state) {
                        int bonus = cc_get_param(cs_SDL_cc_state, CC_PARAM_DTO_BONUS);
                        cc_set_param(cs_SDL_cc_state, CC_PARAM_DTO_BONUS, bonus + 1);
                        fprintf(stderr, "Setting dto bonus to %d\n", bonus + 1);
                    }
                }
                if (event.key.keysym.sym == SDLK_PAGEDOWN) {
                    if (cs_SDL_cc_state) {
                        int bonus = cc_get_param(cs_SDL_cc_state, CC_PARAM_DTO_BONUS);
                        cc_set_param(cs_SDL_cc_state, CC_PARAM_DTO_BONUS, bonus - 1);
                        fprintf(stderr, "Setting dto bonus to %d\n", bonus - 1);
                    }
                }
                if (event.key.keysym.sym == SDLK_PAGEUP) {
                    if (cs_SDL_cc_state) {
                        int bonus = cc_get_param(cs_SDL_cc_state, CC_PARAM_DTO_BONUS);
                        cc_set_param(cs_SDL_cc_state, CC_PARAM_DTO_BONUS, bonus + 1);
                        fprintf(stderr, "Setting dto bonus to %d\n", bonus + 1);
                    }
                }
                if (event.key.keysym.sym == SDLK_PAGEDOWN) {
                    if (cs_SDL_cc_state) {
                        int bonus = cc_get_param(cs_SDL_cc_state, CC_PARAM_DTO_BONUS);
                        cc_set_param(cs_SDL_cc_state, CC_PARAM_DTO_BONUS, bonus - 1);
                        fprintf(stderr, "Setting dto bonus to %d\n", bonus - 1);
                    }
                }
                if (event.key.keysym.sym == SDLK_a) {
                    if (cs_SDL_cc_state) {
                        int penalty = cc_get_param(cs_SDL_cc_state, CC_PARAM_DTO_MAX_PENALTY);
                        cc_set_param(cs_SDL_cc_state, CC_PARAM_DTO_MAX_PENALTY, penalty + 10);
                        fprintf(stderr, "Setting dto penalty to %d\n", penalty + 10);
                    }
                }
                if (event.key.keysym.sym == SDLK_q) {
                    if (cs_SDL_cc_state) {
                        int penalty = cc_get_param(cs_SDL_cc_state, CC_PARAM_DTO_MAX_PENALTY);
                        cc_set_param(cs_SDL_cc_state, CC_PARAM_DTO_MAX_PENALTY, penalty - 10);
                        fprintf(stderr, "Setting dto penalty to %d\n", penalty - 10);
                    }
                }
                return 0;
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_LEFT) {
                    cs_SDL_xclick = event.button.x; cs_SDL_yclick = event.button.y;
                    cs_SDL_there_was_a_click_on_the_map = 1;
                    SDL_CondSignal(cs_SDL_draw_it);
                }
                break;
            case SDL_QUIT:
                fprintf(stderr, "Quit!\n");
                return -1;
        }
    }
    return 0;
}

int
cs_SDL_init(void *s)
{
    SDL_Surface *temp;
    int i;
   
    if (cc_is_state(s)) {
        cs_SDL_cc_state = (cc_state_t*)s;
        cs_SDL_state = cc_get_attached_client_state(cs_SDL_cc_state);
    } else {
        cs_SDL_cc_state = NULL;
        cs_SDL_state = s; 
    }
    cs_SDL_working_with_client = cs_is_client_state(cs_SDL_state);
    if (cs_SDL_working_with_client) {
        cs_client_state_t *state = (cs_client_state_t*)cs_SDL_state;
        cs_SDL_map_size = cs_client_get_param(state, CS_PARAM_MAP_SIZE);
        cs_SDL_resxy = cs_client_get_param(state, CS_PARAM_MAP_RESOLUTION);
        cs_SDL_map_origin_x = cs_client_get_param(state, CS_PARAM_MAP_ORIGIN_X) * 0.001; // in meters
        cs_SDL_map_origin_y = cs_client_get_param(state, CS_PARAM_MAP_ORIGIN_Y) * 0.001;
        cs_SDL_dmap = cs_client_get_map(state, CS_DISTANCE_MAP);
        cs_SDL_debug_map = cs_client_get_map(state, CS_DEBUG_MAP);
    } else {
        cs_state_t *state = (cs_state_t*)cs_SDL_state;
        cs_SDL_map_size = cs_get_param(state, CS_PARAM_MAP_SIZE);
        cs_SDL_resxy = cs_get_param(state, CS_PARAM_MAP_RESOLUTION);
        cs_SDL_map_origin_x = cs_get_param(state, CS_PARAM_MAP_ORIGIN_X) * 0.001; // in meters
        cs_SDL_map_origin_y = cs_get_param(state, CS_PARAM_MAP_ORIGIN_Y) * 0.001;
        cs_SDL_laser_offset = 0;
        cs_SDL_map = cs_get_map(state, CS_GRID_MAP);
        cs_SDL_debug_map = cs_get_map(state, 0xdead);
    }

    cs_SDL_draw_mode = 0;
    cs_SDL_done_flag = 0;

    // Allocate map
    cs_SDL_pixmap = (unsigned char*)malloc(cs_SDL_map_size / SCALE * cs_SDL_map_size / SCALE * 4);
    if (cs_SDL_pixmap == NULL) {
        fprintf(stderr, "Unable to allocate cs_SDL_pixmap\n");
        return 0;
    }

    // SDL initialization code
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "Unable to initialize SDL: %s\n", SDL_GetError());
        return 0;
    }
    atexit(SDL_Quit);

    SDL_WM_SetCaption("CoreSLAM", "CoreSLAM");
    {
        Uint32          colorkey;
        SDL_Surface     *image;

        image = SDL_LoadBMP("logo_corebots.bmp");
        colorkey = SDL_MapRGB(image->format, 0, 0, 0);
        SDL_SetColorKey(image, SDL_SRCCOLORKEY, colorkey);
        SDL_WM_SetIcon(image,NULL);
    }

    cs_SDL_drawing_mutex = SDL_CreateMutex();
    cs_SDL_draw_it = SDL_CreateCond();
    cs_SDL_thread = SDL_CreateThread(cs_SDL_drawing_thread, NULL);

    cs_SDL_screen = SDL_SetVideoMode(DISPLAY_SIZE, DISPLAY_SIZE, 32, SDL_DOUBLEBUF);
    if (cs_SDL_screen == NULL) {
        fprintf(stderr, "Unable to set video mode: %s\n", SDL_GetError());
        return 0;
    }
    cs_SDL_pixmap_surface = SDL_CreateRGBSurfaceFrom(cs_SDL_pixmap, cs_SDL_map_size / SCALE, cs_SDL_map_size / SCALE, 32, cs_SDL_map_size / SCALE * 4, 0, 0, 0, 0);
    for (i = 0; i != cs_SDL_map_size / SCALE * cs_SDL_map_size / SCALE * 4; i++) cs_SDL_pixmap[i] = 0;

    temp = SDL_LoadBMP("arrow_up.bmp");
    if (temp == NULL) {
        fprintf(stderr, "Unable to load bitmap: %s\n", SDL_GetError());
        return 0;
    }
    cs_SDL_arrow_up = SDL_DisplayFormat(temp);
    SDL_SetColorKey(cs_SDL_arrow_up, SDL_SRCCOLORKEY, 0);
    SDL_FreeSurface(temp);

    temp = SDL_LoadBMP("target.bmp");
    if (temp == NULL) {
        printf("Unable to load bitmap: %s\n", SDL_GetError());
        return 1;
    }
    cs_SDL_target = SDL_DisplayFormat(temp);
    SDL_SetColorKey(cs_SDL_target, SDL_SRCCOLORKEY, 0);
    SDL_FreeSurface(temp);
    
    temp = SDL_LoadBMP("CoreSLAM.bmp");
    if (temp == NULL) {
        fprintf(stderr, "Unable to load bitmap: %s\n", SDL_GetError());
        return 0;
    }
    cs_SDL_logo = SDL_DisplayFormat(temp);
    SDL_SetColorKey(cs_SDL_logo, SDL_SRCCOLORKEY, 0);
    SDL_FreeSurface(temp);

    return 1;
}

int
cs_SDL_done()
{
    cs_SDL_done_flag = 1;
    SDL_CondSignal(cs_SDL_draw_it);
    SDL_WaitThread(cs_SDL_thread, NULL);
    SDL_FreeSurface(cs_SDL_pixmap_surface);
    SDL_FreeSurface(cs_SDL_arrow_up);
    SDL_FreeSurface(cs_SDL_logo);
    SDL_FreeSurface(cs_SDL_target);
    SDL_FreeSurface(cs_SDL_screen);
    SDL_DestroyMutex(cs_SDL_drawing_mutex);
    SDL_DestroyCond(cs_SDL_draw_it);
    // SDL_Quit(); // call already registered by atexit
    free(cs_SDL_pixmap);
    return 1;
}

