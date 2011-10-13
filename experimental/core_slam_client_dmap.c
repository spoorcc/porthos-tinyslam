/* CoreSLAM
 * (c) Mines ParisTech 2010 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "core_slam_client_internals.h"

#define X(idx) ((idx) & (CS_MAP_SIZE - 1))
#define Y(idx) ((idx) >> CS_MAP_SIZE_BITS) 
#define DMAP_MAX_DTO 60

static void 
cs_client_dmap_enforce_law(cs_client_state_t *state)
{
    int i, j;
    while (state->queue_end != state->queue_start) {
        // Take the next element in queue
        int idx = state->queue[state->queue_start];
        // A value of 0 or 1 shouldn't be modified. These are obstacles (definitive for 0. Erasable for 1)
        if (state->dmap[idx] > 1) {
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
                // Check that we are inside the state->dmap
                if (!((xneighb | yneighb) & ~(CS_MAP_SIZE - 1))) {
                    // Compute the minimum value among neighbours
                    int idxn = (yneighb << CS_MAP_SIZE_BITS) + xneighb;
                    if (state->dmap[idxn] < minimum) minimum = state->dmap[idxn];
                }
            }
            minimum++; // minimum + 1
            // New : 0 AND 1 are special values
            if (minimum == 1) minimum = 2;
            if (minimum > DMAP_MAX_DTO) minimum = DMAP_MAX_DTO;
            // Compare to the current value
            if (state->dmap[idx] != minimum) {
                // Enforce the law for the point itself
                state->dmap[idx] = minimum;
                // Make sure to enforce the law for the neighbours
                for (j = 0; j != 6; j++) {
                    // Get the value of neighbour
                    int xneighb = x + x_neighbour[odd][j];
                    int yneighb = y + y_neighbour[j];
                    // Check that we are inside the state->dmap
                    if (!((xneighb | yneighb) & ~(CS_MAP_SIZE - 1))) {
                        int idxn = (yneighb << CS_MAP_SIZE_BITS) + xneighb;
                        if (state->dmap[idxn] != CDM_UNEXPLORED) {
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
cs_client_dmap_set_obstacle(cs_client_state_t *state, int x, int y, int value)
{
    int j, idx = (y << CS_MAP_SIZE_BITS) + x;
    int odd = y & 1;
    static const int x_neighbour[2][6] = {{-1,-1,0,1,0,-1}, {-1,0,1,1,1,0}};
    static const int y_neighbour[6] = {0,-1,-1,0,1,1};
    if (state->dmap[idx] != value) {
        int res = 0;
        if (state->dmap[idx] > 1) res = 1; // That was not an obstacle
        state->dmap[idx] = value;
        // Make sure to enforce the law for the neighbours
        for (j = 0; j != 6; j++) {
            // Get the value of neighbour
            int xneighb = x + x_neighbour[odd][j];
            int yneighb = y + y_neighbour[j];
            // Check that we are inside the state->dmap
            if (!((xneighb | yneighb) & ~(CS_MAP_SIZE - 1))) {
                int idxn = (yneighb << CS_MAP_SIZE_BITS) + xneighb;
                // Do enforce the law even for unexplored neighbors (which by the way will then be considered as explored)
                //if (state->dmap->state->dmap[idxn] != CDM_UNEXPLORED) 
                {
                    // Put it into the queue in order to enforce the law
                    state->queue[state->queue_end++] = idxn;
                    state->queue_end &= (state->queue_size - 1);
                }
            } 
        }
        cs_client_dmap_enforce_law(state);
        return res;
    }
    return 0;
}

int 
cs_client_dmap_set_noobstacle(cs_client_state_t *state, int x, int y, int value)
{
    int idx = (y << CS_MAP_SIZE_BITS) + x;
    if (state->dmap[idx] >= value) {
        state->dmap[idx] = CDM_UNEXPLORED - 1;
        state->queue[state->queue_end++] = idx;
        state->queue_end &= (state->queue_size - 1);
        cs_client_dmap_enforce_law(state);
        return 1;
    }
    return 0;
}

int
cs_client_dmap_set_noobstacle_fast(cs_client_state_t *state, int x, int y, int value)
{
    int idx = (y << CS_MAP_SIZE_BITS) + x;
    if (state->dmap[idx] >= value) {
        state->dmap[idx] = CDM_UNEXPLORED - 1;
        return 1;
    }
    return 0;
}

// Doesn't change the status of obstacle / not obstacle. Just make sure it's not UNEXPLORED anymore.
int 
cs_client_dmap_set_explored(cs_client_state_t *state, int x, int y, int *discovered)
{
    int idx = (y << CS_MAP_SIZE_BITS) + x;
    if (state->dmap[idx] == CDM_UNEXPLORED) {
        if (discovered) (*discovered)++;
        state->queue[state->queue_end++] = idx;
        state->queue_end &= (state->queue_size - 1);
        cs_client_dmap_enforce_law(state);
        return 1;
    }
    return 0;
}
