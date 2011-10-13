#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "core_slam_client_internals.h"

#define MAX_WALL_BLOCKS 10000
#define MAX_GROUND_BLOCKS 10000

cs_client_state_t*
cs_client_init(const unsigned char *config_data, int config_data_size)
{
    cs_client_state_t *state;
    uint32_t *buffer = (uint32_t*)config_data;
    
    // Check config data
    if (config_data_size != 7 * sizeof(uint32_t)) {
        fprintf(stderr, "CoreSLAMClient : Bad initialization data\n");
        return NULL;
    }
    if (buffer == NULL || buffer[0] != 0xdead) {
        fprintf(stderr, "CoreSLAMClient : Bad initialization data\n");
        return NULL;
    }

    state = (cs_client_state_t*)malloc(sizeof(cs_client_state_t));
    if (state) {
        state->magic = CS_CLIENT_MAGIC;
        state->map_size_bits = 0;
        state->dmap = NULL;
        state->queue = NULL;
        state->position.x = 0;
        state->position.y = 0;
        state->position.theta = 0;
        state->pitch = 0;
        state->roll = 0;
        state->altitude = 0;
        state->counter = 0;

        state->map_size_bits = buffer[1];
        state->nb_layers = buffer[2];
        state->resxy = buffer[3];
        state->resz = buffer[4];
        state->map_origin_x = buffer[5];
        state->map_origin_y = buffer[6];

        {
            int map_size_bytes = (1 << state->map_size_bits) * (1 << state->map_size_bits);
            if ((state->dmap = (cs_map_pixel_t*)malloc(map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t))) == NULL)
                return 0;
            memset(state->dmap, CDM_UNEXPLORED, map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t));
            if ((state->debug_map = (cs_map_pixel_t*)malloc(map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t))) == NULL)
                return 0;
            if ((state->obstacle_map = (cs_map_pixel_t*)malloc(map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t))) == NULL)
                return 0;
            memset(state->obstacle_map, CDM_UNEXPLORED, map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t));
            if ((state->shield_map = (cs_map_pixel_t*)malloc(map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t))) == NULL)
                return 0;
            memset(state->shield_map, 0, map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t));
            if ((state->zmap = (int16_t*)malloc(map_size_bytes * sizeof(int16_t))) == NULL)
                return 0;
            memset(state->zmap, 0, map_size_bytes * sizeof(int16_t));
        }
        state->queue_size = CS_MAP_SIZE * CS_MAP_SIZE;
        state->queue = (int*) malloc(sizeof(int) * state->queue_size);
        state->queue_start = 0;
        state->queue_end = 0;

        // Transient obstacles management
        state->transient_obstacles_threshold = CDM_UNEXPLORED - 1;
        state->transient_obstacles_delay = 1000; // 50 seconds at 20Hz
        state->transient_obstacles_queue_size = CS_MAP_SIZE * CS_MAP_SIZE;
        state->transient_obstacles_queue = (cs_transient_obstacle_data_t*) malloc(sizeof(cs_transient_obstacle_data_t) * state->transient_obstacles_queue_size);
        state->transient_obstacles_queue_start = 0;
        state->transient_obstacles_queue_end = 0;

        // Diff transmission to client initialization
        state->diff_size = 0;
        state->nb_clients = 0;

        state->diff_cb = NULL;

        return state;
    }
    return NULL;
}

int
cs_client_done(cs_client_state_t *state)
{
    CS_CHECK_CLIENT_MAGIC;
    if (state->queue) free(state->queue);
    if (state->dmap) free(state->dmap);
    if (state->debug_map) free(state->debug_map);
    if (state->obstacle_map) free(state->obstacle_map);
    if (state->shield_map) free(state->shield_map);
    if (state->zmap) free(state->zmap);
    if (state->transient_obstacles_queue) free(state->transient_obstacles_queue);
    free(state);
    return 1;
}

int
cs_client_set_diff_cb(cs_client_state_t *state, cs_client_diff_cb_t cs_client_diff_cb, void *context)
{
    CS_CHECK_CLIENT_MAGIC;
    state->diff_cb = cs_client_diff_cb;
    state->diff_cb_context = context;
    return 1;
}

cs_map_pixel_t 
*cs_client_get_map(cs_client_state_t *state, int map_id)
{
    CS_CHECK_CLIENT_MAGIC;
    switch (map_id) {
        case CS_DISTANCE_MAP:
            return state->dmap;
        case CS_DEBUG_MAP:
            return state->debug_map;
        case CS_OBSTACLE_MAP:
            return state->obstacle_map;
        case CS_SHIELD_MAP:
            return state->shield_map;
    }
    return NULL;
}

cs_position_t
cs_client_get_robot_position(cs_client_state_t *state)
{
    if (state == NULL || state->magic != CS_CLIENT_MAGIC) {
        cs_position_t pos;
        pos.x = 0; pos.y = 0; pos.theta = 0;
        fprintf(stderr, "CoreSLAMClient : Bad magic. State is corrupted\n");
        return pos;
    }
    return state->position;
}

double 
cs_client_get_attitude(cs_client_state_t *state, int which)
{
    CS_CHECK_CLIENT_MAGIC;
    switch (which) {
        case CS_PITCH:
            return state->pitch;
        case CS_ROLL:
            return state->roll;
        case CS_ALTITUDE:
            return state->altitude;
    }
    return 0.0/0.05;
}

int 
cs_is_client_state(void *state)
{
    cs_client_state_t *s = (cs_client_state_t*)state;
    if (s && s->magic == CS_CLIENT_MAGIC)
        return 1;
    return 0;
}

int
cs_client_set_param(cs_client_state_t *state, int param, int value)
{
    CS_CHECK_CLIENT_MAGIC;
    switch (param) {
        case CS_PARAM_TRANSIENT_OBSTACLE_DELAY:
            if (value > 0) {
                state->transient_obstacles_delay = value;
                return 1;
            }
            break;
    }
    return 0;
}

int
cs_client_get_param(cs_client_state_t *state, int param)
{
    CS_CHECK_CLIENT_MAGIC;
    switch (param) {
        case CS_PARAM_MAP_SIZE:
            return CS_MAP_SIZE;
        case CS_PARAM_MAP_RESOLUTION :
            return state->resxy;
        case CS_PARAM_Z_RESOLUTION :
            return state->resz;
        case CS_PARAM_MAP_ORIGIN_X :
            return state->map_origin_x;
        case CS_PARAM_MAP_ORIGIN_Y :
            return state->map_origin_y;
    }
    return 0;
}

int 
cs_client_set_obstacle_from_server(cs_client_state_t *state, int x, int y)
{
    int result;
    cs_map_pixel_t *pobs = &state->shield_map[(y << CS_MAP_SIZE_BITS) + x];
    if (!*pobs) {
        if (result = cs_client_dmap_set_obstacle(state, x, y, 1)) {
            // Add to diff table
            cs_diff_point_t *point = &state->diff_points[state->diff_size++];
            point->x = x; point->y = y; point->z = 0;
            point->plusminus = 1;
            if (state->diff_size == CS_MAX_DIFF_DATA)
                cs_client_send_diff_data(state);
        }
        return result;
    }
    return 0;
}

int 
cs_client_set_noobstacle_from_server(cs_client_state_t *state, int x, int y)
{
    int result;
    cs_map_pixel_t *pobs = &state->shield_map[(y << CS_MAP_SIZE_BITS) + x];
    if (!*pobs) {
        if (result = cs_client_dmap_set_noobstacle(state, x, y, 1)) {
            // add to diff table
            cs_diff_point_t *point = &state->diff_points[state->diff_size++];
            point->x = x; point->y = y; point->z = 0;
            point->plusminus = 0;
            if (state->diff_size == CS_MAX_DIFF_DATA)
                cs_client_send_diff_data(state);
        }
        return result;
    }
    return 0;
}

int 
cs_client_set_explored_from_server(cs_client_state_t *state, int x, int y)
{
    int result;
    if (result = cs_client_dmap_set_explored(state, x, y, NULL)) {
        // add to diff table
        cs_diff_point_t *point = &state->diff_points[state->diff_size++];
        point->x = x; point->y = y; point->z = -1;
        point->plusminus = 1;
        if (state->diff_size == CS_MAX_DIFF_DATA)
            cs_client_send_diff_data(state);
    }
    return result;
}

// Returns 0 on error
// Returns 1 on first time obstacle
// Returns -1 when already previously an obstacle
// if strength >= 255, not a transient obstacle
int
cs_client_set_obstacle(cs_client_state_t *state, int x, int y, int strength)
{
    int size;
    CS_CHECK_CLIENT_MAGIC;
    size = 1 << state->map_size_bits;
    if (x >= 0 && x < size && y >= 0 && y < size) {
        cs_map_pixel_t *pobs = &state->shield_map[(y << CS_MAP_SIZE_BITS) + x];
        if (!*pobs) {
            cs_map_pixel_t *pobs = &state->obstacle_map[(y << CS_MAP_SIZE_BITS) + x];
            int value = *pobs;
            if (value == 0 || value == CDM_UNEXPLORED) {
                // It was not an obstacle before. Update hole map
                if (cs_client_dmap_set_obstacle(state, x, y, 0)) {
                    // Add to diff table
                    cs_diff_point_t *point = &state->diff_points[state->diff_size++];
                    point->x = x; point->y = y; point->z = 0;
                    point->plusminus = 1;
                    if (state->diff_size == CS_MAX_DIFF_DATA)
                        cs_client_send_diff_data(state);
                }
                if (strength < 255) {
                    // Push to transient obstacles queue
                    cs_transient_obstacle_data_t *data = &state->transient_obstacles_queue[state->transient_obstacles_queue_end];
                    state->transient_obstacles_queue_end++;
                    if (state->transient_obstacles_queue_end == state->transient_obstacles_queue_size)
                        state->transient_obstacles_queue_end = 0;
                    if (state->transient_obstacles_queue_end == state->transient_obstacles_queue_start) {
                        fprintf(stderr, "CoreSLAM Client : transient obstacle queue full. Internal system error.\n");
                        return 0;
                    }
                    data->map_index = (y << CS_MAP_SIZE_BITS) + x;
                    data->timestamp = state->counter;
                } else {
                    *pobs = CDM_UNEXPLORED - 1;
                    return 1;
                }
                value = strength; // Increment obstacle persistence
                if (value > CDM_UNEXPLORED - 2) value = CDM_UNEXPLORED - 2;
                *pobs = value;
                return 1; // First time obstacle
            }
            if (strength < 255) {
                value += strength; // Increment obstacle persistence
                if (value > CDM_UNEXPLORED - 2) value = CDM_UNEXPLORED - 2;
                *pobs = value;
            } else {
                *pobs = CDM_UNEXPLORED - 1;
            }
            return -1;
        }
    }
    return 0;
}

int 
cs_client_no_obstacle(cs_client_state_t *state, int x, int y, int strength)
{
    int size;
    CS_CHECK_CLIENT_MAGIC;
    size = 1 << state->map_size_bits;
    if (x >= 0 && x < size && y >= 0 && y < size) {
        cs_map_pixel_t *pobs = &state->shield_map[(y << CS_MAP_SIZE_BITS) + x];
        if (!*pobs) {
            cs_map_pixel_t *pobs = &state->obstacle_map[(y << CS_MAP_SIZE_BITS) + x];
            int value = *pobs;
            if (value == CDM_UNEXPLORED) value = 0;
            value -= strength; // Filtering
            if (value <= 0) { 
                *pobs = 0;
                if (cs_client_dmap_set_noobstacle(state, x, y, 0)) {
                    // add to diff table
                    cs_diff_point_t *point = &state->diff_points[state->diff_size++];
                    point->x = x; point->y = y; point->z = 0;
                    point->plusminus = 0;
                    if (state->diff_size == CS_MAX_DIFF_DATA)
                        cs_client_send_diff_data(state);
                }
            } else 
                *pobs = value;
            return 1;
        }
    }
    return 0;
}

int 
cs_client_protect(cs_client_state_t *state, int x1, int y1, int x2, int y2)
{
    int x, y;
    cs_map_pixel_t *ptr;
    CS_CHECK_CLIENT_MAGIC;

    ptr = state->shield_map;
    for (y = y1; y <= y2; y++) {
        ptr = state->shield_map + (y << state->map_size_bits) + x1;
        for (x = x1; x <= x2; x++, ptr++)
            // TODO : Put this outside loop
            if (x >= 0 && y >= 0 && x < CS_MAP_SIZE && y < CS_MAP_SIZE) 
                *ptr = 1;
    }
    return 1;
}

int 
cs_client_unprotect(cs_client_state_t *state, int x1, int y1, int x2, int y2)
{
    int x, y;
    cs_map_pixel_t *ptr;
    CS_CHECK_CLIENT_MAGIC;

    ptr = state->shield_map;
    for (y = y1; y <= y2; y++) {
        ptr = state->shield_map + (y << state->map_size_bits) + x1;
        for (x = x1; x <= x2; x++, ptr++)
            // TODO : Put this outside loop
            if (x >= 0 && y >= 0 && x < CS_MAP_SIZE && y < CS_MAP_SIZE) 
                *ptr = 0;
    }
    return 1;
}

int 
cs_client_clear_zone(cs_client_state_t *state, int x1, int y1, int x2, int y2)
{
    int x, y;
    CS_CHECK_CLIENT_MAGIC;
    for (y = y1; y <= y2; y++) 
        for (x = x1; x <= x2; x++) {
            cs_client_dmap_set_noobstacle(state, x, y, 1);
            cs_client_no_obstacle(state, x, y, 255);
        }
    return 1;
}

int 
cs_client_obstruct_zone(cs_client_state_t *state, int x1, int y1, int x2, int y2)
{
    int x, y;
    CS_CHECK_CLIENT_MAGIC;
    for (y = y1; y <= y2; y++)
        for (x = x1; x <= x2; x++) {
            cs_client_dmap_set_obstacle(state, x, y, 1);
            cs_client_set_obstacle(state, x, y, 255);
        }
    return 1;
}

int 
cs_client_decompress_map(cs_client_state_t *state, const unsigned char *buffer, int buffer_size)
{
    cs_dmap_run_t *runs = (cs_dmap_run_t *)buffer;
    int c, i, x, y = 0, l = buffer_size / sizeof(cs_dmap_run_t);
    CS_CHECK_CLIENT_MAGIC;
    for (c = 0; c < l; c++) {
        if (runs[c].length == 0) {
            x = 0;
            y = runs[c].code;
        } else for (i = 0; i < runs[c].length; i++, x++) {
            switch (runs[c].code) {
                case 0:
                    cs_client_set_obstacle_from_server(state, x, y);
                    break;
                case 2:
                    cs_client_set_noobstacle_from_server(state, x, y);
                    break;
            }
        }
    }
    return 1;
}

void
cs_client_send_diff_data(cs_client_state_t *state)
{
    int i;
    unsigned char buffer[sizeof(cs_diff_point_t) * CS_MAX_DIFF_DATA + 2 * sizeof(uint32_t)];
    
    // Fill the buffer
    ((uint32_t*)buffer)[0] = 1;
    ((uint32_t*)buffer)[1] = state->diff_size;
    memcpy(buffer + 2 * sizeof(uint32_t), state->diff_points, state->diff_size * sizeof(cs_diff_point_t));
    
    // Send to clients
    for (i = 0; i < state->nb_clients; i++)
        if (state->clients[i].send_fct)
            state->clients[i].send_fct(buffer, 2 * sizeof(uint32_t) + state->diff_size * sizeof(cs_diff_point_t), state->clients[i].context);

    state->diff_size = 0;
}

void
cs_client_send_position(cs_client_state_t *state)
{
    int i;
    int32_t buffer[4];
    cs_position_t pos = cs_client_get_robot_position(state);
    
    // Fill the buffer
    buffer[0] = 2; // Position command
    buffer[1] = (int)(pos.x * 1000);
    buffer[2] = (int)(pos.y * 1000);
    buffer[3] = (int)(pos.theta * 100);
    
    // Send to clients
    for (i = 0; i < state->nb_clients; i++)
        if (state->clients[i].send_fct)
            state->clients[i].send_fct((const char*)buffer, 4 * sizeof(uint32_t), state->clients[i].context);
}

int 
cs_client_register_client(cs_client_state_t *state, cs_send_to_client_fct send_to_client, void *context, char client_config[256], int *client_config_data_size)
{
    CS_CHECK_CLIENT_MAGIC;
    if (state->nb_clients < CS_MAX_CLIENTS) {
        int client_nb = state->nb_clients++;
        cs_client_t *client = &state->clients[client_nb];
        client->send_fct = send_to_client;
        client->context = context;

        // Send a message to the client (transmit grid_map size)
        {
            uint32_t *data = (uint32_t*)client_config;
            data[0] = 0xdead; // 0xdead = first version of protocol
            data[1] = state->map_size_bits;
            data[2] = state->nb_layers;
            data[3] = state->resxy;
            data[4] = state->resz;
            data[5] = state->map_origin_x;
            data[6] = state->map_origin_y;
            if (client_config_data_size) *client_config_data_size = 7 * sizeof(uint32_t);
        }
        return client_nb + 1; 
    } 
    return 0;
}

int 
cs_client_unregister_client(cs_client_state_t *state, int client)
{
    CS_CHECK_CLIENT_MAGIC;
    if (client > 0 && client <= state->nb_clients) {
        state->clients[client - 1].send_fct = NULL;
        return 1;
    }
    return 0;
}

int 
cs_client_compress_map(cs_client_state_t *state, unsigned char *buffer, int buffer_size)
{
    int nb_runs = 0;
    int x, y;
    cs_dmap_run_t *runs = (cs_dmap_run_t *)buffer;
    if (buffer == NULL || buffer_size < 1000 * sizeof(cs_dmap_run_t))
        return 0;
    CS_CHECK_CLIENT_MAGIC;

    for (y = 0; y < CS_MAP_SIZE; y++) {
        cs_map_pixel_t *ptr = &state->dmap[y * CS_MAP_SIZE];
        int code;
        // Line count run
        runs[nb_runs].code = y;
        runs[nb_runs++].length = 0;
        // Let's get first map code
        if (*ptr == 0 || *ptr == 1) code = 0; // Obstacle
        else if (*ptr == CDM_UNEXPLORED) code = 1; // Not explored
        else code = 2; // No obstacle
        ptr++;
        runs[nb_runs].code = code;
        runs[nb_runs].length = 1;
        for (x = 1; x < CS_MAP_SIZE; x++, ptr++) {
            // Let's get the following grid map code
            if (*ptr == 0 || *ptr == 1) code = 0; // Obstacle
            else if (*ptr == CDM_UNEXPLORED) code = 1; // Not explored
            else code = 2; // No obstacle
            if (code == runs[nb_runs].code) {
                // It's the same... Let's grow the run
                runs[nb_runs].length++;
            } else {
                // Let's write this run
                nb_runs++;
                if ((nb_runs + 1) * sizeof(cs_dmap_run_t) >= (unsigned int)buffer_size)
                    return 0;
                // and start another one
                runs[nb_runs].code = code;
                runs[nb_runs].length = 1;
            }
        }
        // Let's write the last run of the current line
        nb_runs++;
        if ((nb_runs + 1) * sizeof(cs_dmap_run_t) >= (unsigned int)buffer_size)
            return 0;
    }
    return nb_runs * sizeof(cs_dmap_run_t);
}

int
cs_client_robot_position_to_map_coordinates(cs_client_state_t *state, cs_position_t *position, int *x, int *y)
{
    cs_position_t pos;
    CS_CHECK_CLIENT_MAGIC;
    if (position) {
        pos.x = position->x;
        pos.y = position->y;
    } else {
        pos.x = state->position.x;
        pos.y = state->position.y;
    }
    // Translate position according to map origin
    pos.x -= state->map_origin_x * 0.001; pos.y -= state->map_origin_y * 0.001;
    *x = (int)(pos.x * 1000 * CS_MAP_SCALE + 0.5); 
    *y = (int)(pos.y * 1000 * CS_MAP_SCALE + 0.5);
    return 1;
}

int
cs_client_map_coordinates_to_robot_position(cs_client_state_t *state, int x, int y, cs_position_t *position)
{
    if (position == NULL)
        return 0;
    CS_CHECK_CLIENT_MAGIC;
    position->x = (x * state->resxy + state->map_origin_x) * 0.001;
    position->y = (y * state->resxy + state->map_origin_y) * 0.001; 
    return 1;
}

int
cs_update_client_state(cs_client_state_t *state, const unsigned char *data, int data_size)
{    
    uint32_t command = *(uint32_t*)data;
    CS_CHECK_CLIENT_MAGIC;
    switch (command) {
        case 1:
            {
                int i;
                int diff_size = ((uint32_t*)data)[1];
                cs_client_diff_point_t *points = (cs_client_diff_point_t*)(data + 2 * sizeof(uint32_t));
                
                // Diff callback
                if (state->diff_cb) 
                    state->diff_cb(state, points, diff_size, state->diff_cb_context); 

                for(i = 0; i < diff_size; i++) {
                    if (points[i].plusminus == 1) {
                        if (points[i].z == -1)
                            cs_client_set_explored_from_server(state, points[i].x, points[i].y);
                        else
                            cs_client_set_obstacle_from_server(state, points[i].x, points[i].y);
                    } else 
                        cs_client_set_noobstacle_from_server(state, points[i].x, points[i].y);
                }
            }
            return 1;
        case 2:
            {
                int32_t *buffer = (int32_t*)data;
                state->position.x = buffer[1] * 0.001;
                state->position.y = buffer[2] * 0.001;
                state->position.theta = buffer[3] * 0.01;
                state->pitch = buffer[4] * 0.01;
                state->roll = buffer[5] * 0.01;
                state->altitude = buffer[6] * 0.001;

                // Create an explored area around the robot
                if (state->counter == 0) {
                    const int width = 50;
                    int x, y, xc, yc;
                    cs_client_robot_position_to_map_coordinates(state, &state->position, &x, &y);
                    for (xc = x - width / 2; xc <= x + width / 2; xc++) 
                        for (yc = y - width / 2; yc <= y + width / 2; yc++) 
                            cs_client_dmap_set_noobstacle_fast(state, xc, yc, 1);
                }
        
                // Transient obstacles management
                if (state->transient_obstacles_delay) {
                    while (state->transient_obstacles_queue_start != state->transient_obstacles_queue_end) {
                        cs_transient_obstacle_data_t *data = &state->transient_obstacles_queue[state->transient_obstacles_queue_start];
                        // Check if this obstacle is old enough
                        if (data->timestamp < state->counter - state->transient_obstacles_delay) {
                            // OK. That one must be consumed
                            state->transient_obstacles_queue_start++;
                            if (state->transient_obstacles_queue_start == state->transient_obstacles_queue_size)
                                state->transient_obstacles_queue_start = 0;
                            // Check if this obstacle is transient or not
                            if (state->obstacle_map[data->map_index] != 0 && state->obstacle_map[data->map_index] < state->transient_obstacles_threshold) {
                                // Yes. This is a transient obstacle. Let's remove it from the map.
                                int x, y;
                                x = data->map_index & ((1 << CS_MAP_SIZE_BITS) - 1);
                                y = data->map_index >> CS_MAP_SIZE_BITS;
                                cs_client_no_obstacle(state, x, y, 255);
                            }
                        } else break; // Don't check the tail
                    }
                }
                cs_client_send_position(state);

                state->counter++;
                return 1;
            }
        case 3:
            {
                int x, y, i;
                int diff_size = ((uint32_t*)data)[1];
                cs_zmap_diff_zone_t *zone = (cs_zmap_diff_zone_t*)(data + 2 * sizeof(uint32_t));
                for (i = 0; i < diff_size; i++, zone++) {
                    for (y = zone->y1; y <= zone->y2; y++) {
                        int16_t *ptr = &state->zmap[(y << CS_MAP_SIZE_BITS) + zone->x1];
                        for (x = zone->x1; x <= zone->x2; x++, ptr++)
                            *ptr = zone->z;
                    }
                }
                // Send to clients
                for (i = 0; i < state->nb_clients; i++)
                    if (state->clients[i].send_fct)
                        state->clients[i].send_fct(data, data_size, state->clients[i].context);
            }
            return 1;
    }
    return 0;
}

int
cs_client_get_counter(cs_client_state_t *state)
{
    CS_CHECK_CLIENT_MAGIC;
    return state->counter;
}

