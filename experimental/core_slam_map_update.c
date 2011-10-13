/* CoreSLAM
 * (c) Mines ParisTech 2010-2011 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "core_slam_internals.h"

static int 
cs_map_accumulate(cs_state_t *state, int x1, int y1, int x2, int y2)
{
    int dx, dy, xincr, yincr, x, y, xp, yp, i, j;
    int inc, error, correction, acc, temp, ptrincr, runl, runl2;
    unsigned char *ptrX;
    int value;
    const int pixsize = 1; 
    int map_size = CS_MAP_SIZE;
    cs_map_pixel_t *map = state->grid_map;

    if (x1 < x2) {
        x=x1; y=y1; xp=x2; yp=y2;
    } else {
        x=x2; y=y2; xp=x1; yp=y1;
    }
    if (x<0) {
        if (x==xp) return 1;
        y=y+(y-yp)*(0-x)/(x-xp);
        x=0;
        if (xp<0) return 1; /* The line is completely outside the ROI */
    }
    if (xp>=(0+map_size)) {
        if (x==xp) return 1;
        yp=y+(y-yp)*((0+map_size)-x-1)/(x-xp);
        xp=(0+map_size)-1;
        if (x>=(0+map_size)) return 1; /* The line is completely outside the ROI */
    }
    if (y<yp) {
        x1=x; y1=y; x2=xp; y2=yp;
    } else {
        x1=xp; y1=yp; x2=x; y2=y;
    }
    if (y1<0) {
        if (y1==y2) return 1;
        x1=x1+(x1-x2)*(0-y1)/(y1-y2);
        y1=0;
        if (y2<0) return 1; /* The line is completely outside the ROI */
    }
    if (y2>=(0+map_size)) {
        if (y1==y2) return 1;
        x2=x1+(x1-x2)*((0+map_size)-y1-1)/(y1-y2);
        y2=(0+map_size)-1;
        if (y1>=(0+map_size)) return 1; /* The line is completely outside the ROI */
    }

#define SETPIXEL value = *ptrX + state->positive_dynamics; cs_map_set(state, x, y, value);
#define INITPOINTERS ptrX = map + y * map_size * pixsize + x * pixsize
#define SWAP(x,y) temp = x; x = y; y = temp
    dx = abs(x2 - x1) + 1;
    dy = abs(y2 - y1) + 1;
    if (dx > dy) { /* X or Y axis ? */
	if (x1 > x2) {
	    SWAP(x1, x2);
	    SWAP(y1, y2);
	}
	if (y2 > y1) {
	    yincr = 1;
	    ptrincr = map_size * pixsize;
	} else {
	    yincr = -1;
	    ptrincr = -map_size * pixsize;
	}
	x = x1;
	y = y1;
	runl = dx / dy;
	error = 2 * (dx - runl * dy);
	correction = 2 * dy;
	acc = 0;
	inc = pixsize;

	INITPOINTERS;
	for(i = 0; i < dy; i++, y += yincr) {
	    acc += error;
	    runl2 = runl;
	    if (acc > dy) {
		acc -= correction;
		++runl2;
	    }
	    for (j = 0; j < runl2; j++, x++) {
		SETPIXEL;
		ptrX += inc;
	    }
	    ptrX += ptrincr;
	}
    } else {
	if (y1 > y2) {
	    SWAP(x1, x2);
	    SWAP(y1, y2);
	}
	if (x2 > x1) {
	    xincr = 1;
	    ptrincr = pixsize;
	} else {
	    xincr = -1;
	    ptrincr = -pixsize;
	}
	x = x1;
	y = y1;
	runl = dy / dx;
	error = 2 * (dy - runl * dx);
	correction = 2 * dx;
	acc = 0;
	inc = map_size * pixsize;

	INITPOINTERS;
	for(i = 0; i < dx; i++, x += xincr, ptrX += ptrincr) {
	    acc += error;
	    runl2 = runl;
	    if (acc > dx) {
		acc -= correction;
		++runl2;
	    }
	    for (j = 0; j < runl2; j++, y++) {
		SETPIXEL;
		ptrX += inc;
	    }
	}
    }
    return 1;
}

int 
cs_map_update(cs_state_t *state, cs_scan_t *scan, cs_position_t *pos)
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2;
    unsigned char *ptr;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * 1000 * CS_MAP_SCALE + 0.5);
    y1 = (int)floor(pos->y * 1000 * CS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i < scan->nb_points; i++) {
        if (scan->points[i].value) {
            x2p = c * scan->points[i].x - s * scan->points[i].y;
            y2p = s * scan->points[i].x + c * scan->points[i].y;
            x2 = (int)floor((pos->x * 1000 + x2p) * CS_MAP_SCALE + 0.5);
            y2 = (int)floor((pos->y * 1000 + y2p) * CS_MAP_SCALE + 0.5);
            if (x2 >= 0 && y2 >= 0 && x2 < CS_MAP_SIZE && y2 < CS_MAP_SIZE) {
                 int value;
                 ptr = state->grid_map + (y2 << CS_MAP_SIZE_BITS) + x2;
                 value = *ptr;
                 cs_map_accumulate(state, x1, y1, x2, y2);
                 if (value - state->negative_dynamics <= 0) {
                     cs_map_set(state, x2, y2, 0);
                 } else *ptr = value - state->negative_dynamics;
            } else 
                 cs_map_accumulate(state, x1, y1, x2, y2);
        } else {
            x2p = c * scan->points[i].x - s * scan->points[i].y;
            y2p = s * scan->points[i].x + c * scan->points[i].y;
            x2 = (int)floor((pos->x * 1000 + x2p) * CS_MAP_SCALE + 0.5);
            y2 = (int)floor((pos->y * 1000 + y2p) * CS_MAP_SCALE + 0.5);
            cs_map_accumulate(state, x1, y1, x2, y2);
        }
    }
    return 1;
}

int 
cs_map_update_new_points(cs_state_t *state, cs_scan_t *scan, cs_position_t *pos)
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2;
    unsigned char *ptr;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * 1000 * CS_MAP_SCALE + 0.5);
    y1 = (int)floor(pos->y * 1000 * CS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i < scan->nb_points; i++) {
        if (scan->points[i].value) {
            x2p = c * scan->points[i].x - s * scan->points[i].y;
            y2p = s * scan->points[i].x + c * scan->points[i].y;
            x2 = (int)floor((pos->x * 1000 + x2p) * CS_MAP_SCALE + 0.5);
            y2 = (int)floor((pos->y * 1000 + y2p) * CS_MAP_SCALE + 0.5);
            if (x2 >= 0 && y2 >= 0 && x2 < CS_MAP_SIZE && y2 < CS_MAP_SIZE) {
                 int value;
                 ptr = state->obstacle_map + (y2 << CS_MAP_SIZE_BITS) + x2;
                 value = *ptr;
                 if (value == CDM_UNEXPLORED) {
                     cs_map_accumulate(state, x1, y1, x2, y2);
                     cs_map_set(state, x2, y2, 0);
                }
            }
        }
    }
    return 1;
}
