/* CoreSLAM Client
 * (c) Mines ParisTech 2010 */

#ifndef _CORESLAM_CLIENT_H_
#define _CORESLAM_CLIENT_H_

#include "core_slam.h"

#ifdef __CPLUSPLUS
extern "C" {
#endif

#ifdef WIN32
#ifndef CS_SHARED
#define CS_SHARED __declspec(dllimport)
#endif // CS_SHARED
#else
#define CS_SHARED
#endif // WIN32

#define CS_PARAM_TRANSIENT_OBSTACLE_DELAY 1
    
typedef struct {
    int plusminus;
    int x, y, z;
} cs_client_diff_point_t;

typedef struct cs_client_state cs_client_state_t;
typedef void (*cs_client_diff_cb_t)(cs_client_state_t *state, cs_client_diff_point_t *p, int nb_diff, void *context);

CS_SHARED cs_client_state_t* cs_client_init(const unsigned char* config_data, int config_data_size);
CS_SHARED int cs_client_done(cs_client_state_t *state);
CS_SHARED int cs_client_set_diff_cb(cs_client_state_t *state, cs_client_diff_cb_t cs_client_diff_cb, void *context);
CS_SHARED int cs_update_client_state(cs_client_state_t *state, const unsigned char *data, int data_size);
CS_SHARED cs_map_pixel_t *cs_client_get_map(cs_client_state_t *state, int map_id);
CS_SHARED cs_position_t cs_client_get_robot_position(cs_client_state_t *state);
CS_SHARED double cs_client_get_attitude(cs_client_state_t *state, int which);
CS_SHARED int cs_is_client_state(void *state);
CS_SHARED int cs_client_set_param(cs_client_state_t *state, int param, int value);
CS_SHARED int cs_client_get_param(cs_client_state_t *state, int param);
CS_SHARED int cs_client_decompress_map(cs_client_state_t *state, const unsigned char *buffer, int buffer_size);
CS_SHARED int cs_client_robot_position_to_map_coordinates(cs_client_state_t *state, cs_position_t *position, int *x, int *y);
CS_SHARED int cs_client_map_coordinates_to_robot_position(cs_client_state_t *state, int x, int y, cs_position_t *position);
CS_SHARED int cs_client_set_obstacle(cs_client_state_t *state, int x, int y, int strength);
CS_SHARED int cs_client_no_obstacle(cs_client_state_t *state, int x, int y, int strength);
CS_SHARED int cs_client_get_counter(cs_client_state_t *state);
CS_SHARED int cs_client_register_client(cs_client_state_t *state, cs_send_to_client_fct send_to_client, void *context, char config_buffer[256], int *client_config_data_size);
CS_SHARED int cs_client_compress_map(cs_client_state_t *state, unsigned char *buffer, int buffer_size);
CS_SHARED int cs_client_unregister_client(cs_client_state_t *state, int client);

// Protect / unprotect zones
CS_SHARED int cs_client_protect(cs_client_state_t *state, int x1, int y1, int x2, int y2);
CS_SHARED int cs_client_unprotect(cs_client_state_t *state, int x1, int y1, int x2, int y2);
// Clear zone (in order to authorize control), or obstruct
CS_SHARED int cs_client_clear_zone(cs_client_state_t *state, int x1, int y1, int x2, int y2);
CS_SHARED int cs_client_obstruct_zone(cs_client_state_t *state, int x1, int y1, int x2, int y2);

// Obsolete functions :
#define cs_init_client cs_client_init
#define cs_done_client cs_client_done

#ifdef __CPLUSPLUS
}
#endif

#endif // _CORE_SLAM_CLIENT_H_

