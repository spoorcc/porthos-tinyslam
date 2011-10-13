/* CoreSLAM
 * (c) Mines ParisTech 2010 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "core_slam_internals.h"

#define X(idx) ((idx) & (CS_MAP_SIZE - 1))
#define Y(idx) ((idx) >> CS_MAP_SIZE_BITS)

static void 
dmap_enforce_law(cs_state_t *state, cs_map_pixel_t *map)
{
    int i, j;
//#define CDM_EXPLORATION_DISTANCE 20
#define CDM_EXPLORATION_DISTANCE 1
	while (state->queue_end != state->queue_start) {
        // Take the next element in queue
        int idx = state->queue[state->queue_start];
        // A value of 0 or 1 shouldn't be modified. These are obstacles (definitive for 0. Erasable for 1)
        if (map[idx] > 0) {
            int x = X(idx), y = Y(idx);
            // We check that we need to modify or not the value
            // We look at all the neighbours values
            static const int x_neighbour[2][6] = {{-1,-1,0,1,0,-1}, {-1,0,1,1,1,0}};
            static const int y_neighbour[6] = {0,-1,-1,0,1,1};
            int odd = y & 1;
            int minimum = CDM_UNEXPLORED - 1;
            for (i = 0; i != 6; i++) {
                // Get the value of neighbour
                int xneighb = x + x_neighbour[odd][i];
                int yneighb = y + y_neighbour[i];
                // Check that we are inside the map
                if (!((xneighb | yneighb) & ~(CS_MAP_SIZE - 1))) {
                    // Compute the minimum value among neighbours
                    int idxn = (yneighb << CS_MAP_SIZE_BITS) + xneighb;
                    if (map[idxn] < minimum) minimum = map[idxn];
                }
            }
            minimum++; // minimum + 1
            if (minimum > state->dmap_max_dto) minimum = state->dmap_max_dto;
            // Compare to the current value
            if (map[idx] != minimum) {
                // Enforce the law for the point itself
                map[idx] = minimum;
                // Make sure to enforce the law for the neighbours
                for (j = 0; j != 6; j++) {
                    // Get the value of neighbour
                    int xneighb = x + x_neighbour[odd][j];
                    int yneighb = y + y_neighbour[j];
                    // Check that we are inside the map
                    if (!((xneighb | yneighb) & ~(CS_MAP_SIZE - 1))) {
                        int idxn = (yneighb << CS_MAP_SIZE_BITS) + xneighb;
                        if (map[idxn] != CDM_UNEXPLORED || map[idx] < CDM_EXPLORATION_DISTANCE) {
                            // Put it into the queue in order to enforce the law
                            state->queue[state->queue_end++] = idxn;
                            state->queue_end &= (state->queue_size - 1);
                        }
                    } 
                }
            }
        }
        state->queue_start++;
        state->queue_start &= (state->queue_size - 1);
    }
}

int
dmap_set_obstacle(cs_state_t *state, cs_map_pixel_t *map, int x, int y)
{
    int j, idx = (y << CS_MAP_SIZE_BITS) + x;
    int odd = y & 1;
    static const int x_neighbour[2][6] = {{-1,-1,0,1,0,-1}, {-1,0,1,1,1,0}};
    static const int y_neighbour[6] = {0,-1,-1,0,1,1};
    if (map[idx] != 0) {
        int res = 0;
        map[idx] = 0;
        // Make sure to enforce the law for the neighbours
        for (j = 0; j != 6; j++) {
            // Get the value of neighbour
            int xneighb = x + x_neighbour[odd][j];
            int yneighb = y + y_neighbour[j];
            // Check that we are inside the map
            if (!((xneighb | yneighb) & ~(CS_MAP_SIZE - 1))) {
                int idxn = (yneighb << CS_MAP_SIZE_BITS) + xneighb;
                // Do enforce the law even for unexplored neighbors (which by the way will then be considered as explored)
                //if (map->map[idxn] != CDM_UNEXPLORED) 
                {
                    // Put it into the queue in order to enforce the law
                    state->queue[state->queue_end++] = idxn;
                    state->queue_end &= (state->queue_size - 1);
                }
            } 
        }
        dmap_enforce_law(state, map);
        return 1;
    }
    return 0;
}

void 
dmap_set_noobstacle(cs_state_t *state, cs_map_pixel_t *map, int x, int y)
{
    int idx = (y << CS_MAP_SIZE_BITS) + x;
    map[idx] = CDM_UNEXPLORED - 1;
    state->queue[state->queue_end++] = idx;
    state->queue_end &= (state->queue_size - 1);
    dmap_enforce_law(state, map);
}

int 
dmap_set_explored(cs_state_t *state, cs_map_pixel_t *map, int x, int y, int *discovered)
{
    int idx = (y << CS_MAP_SIZE_BITS) + x;
    if (map[idx] == CDM_UNEXPLORED) {
        map[idx] = CDM_UNEXPLORED - 1; // Set as non obstacle
        if (discovered) (*discovered)++;
        state->queue[state->queue_end++] = idx;
        state->queue_end &= (state->queue_size - 1);
        dmap_enforce_law(state, map);
        return 1;
    }
    return 0;
}
