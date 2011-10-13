/* CoreSLAM client
 * (c) Mines ParisTech 2010 */

#include "core_slam_internals.h"
#include "core_slam_client.h"

#define CS_CLIENT_MAGIC            0xc03e51a4
#define CS_CHECK_CLIENT_MAGIC      if (state == NULL || state->magic != CS_CLIENT_MAGIC) { \
    fprintf(stderr, "CoreSLAMClient : Bad magic. State is corrupted\n"); \
    return 0; \
}

struct cs_client_state {
    uint32_t magic;                 // Magic number
   
    cs_map_pixel_t *dmap;
    cs_map_pixel_t *debug_map;
    cs_map_pixel_t *obstacle_map;   // This is a map where 0 are non obstacles, 255 are not explored and values in between are obstacles persistence counters
    cs_map_pixel_t *shield_map;     // Shield map, to prevent modifications to the map
    int16_t        *zmap;           // Height / Altitude map
    
    cs_position_t position;
    double pitch, roll, altitude;
    int counter;
    
    // The queue for distance map computation
    int *queue;
    int queue_size;
    int queue_start, queue_end;

    // The queue for transient obstacles removal
    int transient_obstacles_delay;
    int transient_obstacles_threshold;
    cs_transient_obstacle_data_t *transient_obstacles_queue;
    int transient_obstacles_queue_size;
    int transient_obstacles_queue_start, transient_obstacles_queue_end;
    
    // Client/server data
    int diff_size;
    cs_diff_point_t diff_points[CS_MAX_DIFF_DATA];
    cs_client_t clients[CS_MAX_CLIENTS];
    int nb_clients;

    // Parameters (retrieved from server) 
    int map_size_bits;
    int nb_layers; // number of z-layers
    int resxy; // horizontal dimension in mm of a block
    int resz; // vertical dimension in mm of a block
    int map_origin_x;           // map origin (in mm)
    int map_origin_y;           // map origin (in mm)
    
    cs_client_diff_cb_t diff_cb;
    void *diff_cb_context;
};

cs_client_state_t* cs_init_client();

int cs_client_dmap_set_obstacle(cs_client_state_t *state, int x, int y, int value);
int cs_client_dmap_set_noobstacle(cs_client_state_t *state, int x, int y, int value);
int cs_client_dmap_set_noobstacle_fast(cs_client_state_t *state, int x, int y, int value);
int cs_client_dmap_set_explored(cs_client_state_t *state, int x, int y, int *discovered);

void cs_client_send_diff_data(cs_client_state_t *state);
void cs_client_send_position(cs_client_state_t *state);

int cs_client_set_obstacle_from_server(cs_client_state_t *state, int x, int y);
int cs_client_set_noobstacle_from_server(cs_client_state_t *state, int x, int y);
int cs_client_set_explored_from_server(cs_client_state_t *state, int x, int y);

