/* CoreSLAM
 * (c) Mines ParisTech 2010-2011 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#ifdef _WIN32
#include <malloc.h>
#define alloca _alloca
#else
#include <alloca.h>
#endif // _WIN32
#include "core_slam_internals.h"

#define CS_DEBUG_MESSAGES

static int
cs_allocate_maps(cs_state_t *state)
{
    int map_size_bytes = (1 << state->map_size_bits) * (1 << state->map_size_bits);
    if ((state->grid_map = (cs_map_pixel_t*)malloc(map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t))) == NULL)
        return 0;
    if ((state->hole_map = (cs_map_pixel_t*)malloc(map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t))) == NULL)
        return 0;
    if ((state->obstacle_map = (cs_map_pixel_t*)malloc(map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t))) == NULL)
        return 0;
    if ((state->shield_map = (cs_map_pixel_t*)malloc(map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t))) == NULL)
        return 0;
    if ((state->zmap = (int16_t*)malloc(map_size_bytes * sizeof(int16_t))) == NULL)
        return 0;
    state->owner = NULL;

    // Clean up maps
    memset(state->grid_map, 128, map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t));
    memset(state->hole_map, CDM_UNEXPLORED, map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t));
    memset(state->obstacle_map, CDM_UNEXPLORED, map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t));
    memset(state->shield_map, 0, map_size_bytes * state->nb_layers * sizeof(cs_map_pixel_t));
    memset(state->zmap, 0, map_size_bytes * sizeof(int16_t));
    
    return 1;
}

static int
cs_realloc_maps(cs_state_t *state)
{
    if (state->owner == NULL || state->owner == state) {
        if (state->grid_map) free(state->grid_map);
        if (state->hole_map) free(state->hole_map);
        if (state->obstacle_map) free(state->obstacle_map);
        if (state->shield_map) free(state->shield_map);
        if (state->zmap) free(state->zmap);
    }
    return cs_allocate_maps(state);
}

static int 
cs_init_base(cs_state_t *state, int laser_type, int laser_offset, int options)
{
    int i;

    state->magic = CS_MAGIC;

    state->grid_map = NULL;
    state->hole_map = NULL;
    state->obstacle_map = NULL;
    state->owner = NULL;
    
    state->update_map = (options & CS_NO_MAP_UPDATE) ? 0 : 1;
    state->mode = CS_MODE_MAPPING;

    state->laser_parameters.detection_margin = 0;
    state->laser_parameters.minimum_distance = 50;
    state->laser_parameters.filter = 0;
    state->laser_parameters.upside_down = (options & CS_SENSOR_UPSIDE_DOWN); 
    state->laser_parameters.angle_start = 0xdead;
    state->laser_parameters.angle_end = 0xdead;

    switch (laser_type) {
        case CS_HOKUYO_UTM30LX :
            state->laser_parameters.scan_size = 1081;
            state->laser_parameters.scan_angle = 270;
            state->laser_parameters.scan_duration = 25;
            state->laser_parameters.distance_no_detection = 7000;
            state->laser_parameters.max_distance = 30000;
            state->laser_parameters.filter = 100;
            state->sigma_xy = 0.2;
            state->sigma_theta = 10;
            state->latency = 200;
            break;
        case CS_HOKUYO_URG04LX :
            state->laser_parameters.scan_size = 682;
            state->laser_parameters.scan_angle = 240;
            state->laser_parameters.scan_duration = 100;
            state->laser_parameters.distance_no_detection = 2000;
            state->laser_parameters.max_distance = 5600;
            state->sigma_xy = 0.2;
            state->sigma_theta = 10;
            state->latency = 10;
            break;
        case CS_SICK_LMS100 :
        case CS_SICK_LMS111 :
            state->laser_parameters.scan_size = 1082;
            state->laser_parameters.scan_angle = 270;
            state->laser_parameters.scan_duration = 40;
            state->laser_parameters.distance_no_detection = 4000;
            state->laser_parameters.max_distance = 30000;
            state->sigma_xy = 0.1;
            state->sigma_theta = 10;
            state->latency = 20;
            break;
        case CS_SICK_LMS200 :
            state->laser_parameters.scan_size = 361; 
            state->laser_parameters.scan_angle = 180;
            state->laser_parameters.scan_duration = 500;
            state->laser_parameters.distance_no_detection = 600;
            state->laser_parameters.max_distance = 3000;
            state->sigma_xy = 0.2;
            state->sigma_theta = 10;
            state->latency = 10;
            break;
		case CS_NEATO_XV11 :
            state->laser_parameters.scan_size = 360; 
            state->laser_parameters.scan_angle = 360;
            state->laser_parameters.scan_duration = 450;
            state->laser_parameters.distance_no_detection = 500;
            state->laser_parameters.max_distance = 5000;
            state->sigma_xy = 0.2;
            state->sigma_theta = 10;
            state->latency = 10;
            break;
        case CS_MICROSOFT_KINECT :
            return 0;
    }  

    state->switch_to_mapping_threshold = 200000;
    state->switch_to_reloc_threshold = 20000000; // Very high
    state->auto_switch_to_mapping = 1000; // 50s at 20Hz
    
    state->position.x = 0; // in meters
    state->position.y = 0;
    state->position.theta = 0;
    state->pitch = 0;
    state->roll = 0;
    state->altitude = 0;
    state->update_altitude_wrt_attitude = 0;

    state->counter = 0;
    state->speed = 0;
    state->score = 0;
    state->model_error = 0;
    state->ok = 0;
    state->delayed_avg_score = 0;
    state->current_avg_score = state->switch_to_reloc_threshold;
    state->previous_failed = 0;
    state->reloc_counter = 0;

    state->map_size_bits = 11;
    state->positive_dynamics = 10;
    state->negative_dynamics = 100;
    state->resxy = 20; // 20mm per pixel
    state->resz = 0;
    state->nb_layers = 1;
    state->neval = 3000;
    state->min_search_res_xy = 0; //0.0125;
    state->min_search_res_theta = 0; //0.625;
    state->get_rid_of_outliers = 0;
    state->transient_obstacles_threshold = 20;
    state->transient_obstacles_delay = 600; // One minute at 10Hz
    state->dmap_max_dto = 20;
    if (options & CS_WHEEL_ROBOT) 
        state->max_model_error = 0.5;
    else 
        state->max_model_error = 1000;
    state->param_ground_rejection = -100;
    state->param_magic_ratio[CS_MODE_MAPPING] = 12;
    state->param_magic_ratio[CS_MODE_RELOC] = 24;

    // Stationnarity management
    state->param_stationnarity_max_counter = 10;
    state->param_stationnarity_posxy = 0.04;
    state->param_stationnarity_theta = 0.4;
    state->stationnarity_counter = 0;

    state->map_origin_x = -(CS_MAP_SIZE * state->resxy) / 2; // in mm
    state->map_origin_y = state->map_origin_x;
    state->map_origin_z = 0;

    for (i = 0; i < CS_MAX_DEBUG_INFO; i++)
        state->debug_info[i] = 0;

    state->queue_size = CS_MAP_SIZE * CS_MAP_SIZE;
    state->queue = (int*) malloc(sizeof(int) * state->queue_size);
    state->queue_start = 0;
    state->queue_end = 0;

    state->transient_obstacles_queue_size = CS_MAP_SIZE * CS_MAP_SIZE;
    state->transient_obstacles_queue = (cs_transient_obstacle_data_t*) malloc(sizeof(cs_transient_obstacle_data_t) * state->transient_obstacles_queue_size);
    state->transient_obstacles_queue_start = 0;
    state->transient_obstacles_queue_end = 0;

    state->scan = (cs_scan_t*)malloc(sizeof(cs_scan_t));
    if (state->scan == NULL) return 0;

    state->covariance = cs_allocate_matrix(3, 3);
    if (state->covariance == NULL) return 0;
    cs_matrix_empty(state->covariance, 3, 3);

    state->laser_position = cs_allocate_matrix(4, 4);
    if (state->laser_position == NULL) return 0;
    cs_matrix_empty(state->laser_position, 4, 4);
    state->laser_position[1][1] = 1;
    state->laser_position[2][2] = 1;
    state->laser_position[3][3] = 1;
    state->laser_position[4][4] = 1;
    state->laser_position[1][4] = (int32_t)((int16_t)(laser_offset & 0xffff));
    state->laser_position[3][4] = (laser_offset >> 16);

    printf("laser_position : %lf %f\n", state->laser_position[1][4], state->laser_position[3][4]);

    state->diff_size = 0;
    state->zmap_diff_size = 0;
    state->nb_clients = 0;

#ifdef MULTICORE_SUPPORT
    cs_create_pool_of_threads(state);    
    fprintf(stderr, "CoreSLAM : Detected %d cores. Launching pool of threads.\n", state->nb_cores);
#endif // MULTICORE_SUPPORT
    return 1;
}

int 
cs_register_client(cs_state_t *state, cs_send_to_client_fct send_to_client, void *context, char client_config[256], int *client_config_data_size)
{
    CS_CHECK_MAGIC;
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
cs_unregister_client(cs_state_t *state, int client)
{
    CS_CHECK_MAGIC;
    if (client > 0 && client <= state->nb_clients) {
        state->clients[client - 1].send_fct = NULL;
        return 1;
    }
    return 0;
}

cs_state_t *
cs_init(int laser_type, int laser_offset, int options)
{
    cs_state_t *state = (cs_state_t*)malloc(sizeof(cs_state_t));
    if (state) {
        if (cs_init_base(state, laser_type, laser_offset, options) && cs_allocate_maps(state))
            return state;
        free(state);
    }
    return NULL;
}

cs_state_t * 
cs_init_3d(int laser_type, int laser_offset, int resxy, int resz, int nb_layers, int options)
{
    cs_state_t *state = (cs_state_t*)malloc(sizeof(cs_state_t));
    if (state) {
        if (cs_init_base(state, laser_type, laser_offset, options)) {
            state->resxy = resxy;
            state->resz = resz;
            state->nb_layers = nb_layers;
            state->map_origin_x = -(CS_MAP_SIZE * state->resxy) / 2; // in mm
            state->map_origin_y = state->map_origin_x;
            if (cs_allocate_maps(state)) 
                return state;
        }
        free(state);
    }
    return NULL;
}

int
cs_done(cs_state_t *state)
{
    CS_CHECK_MAGIC;
#ifdef MULTICORE_SUPPORT
    cs_destroy_pool_of_threads(state);
#endif
    if (state->owner == NULL || state->owner == state) {
        if (state->grid_map) free(state->grid_map);
        if (state->hole_map) free(state->hole_map);
        if (state->obstacle_map) free(state->obstacle_map);
        if (state->shield_map) free(state->shield_map);
        if (state->zmap) free(state->zmap);
    }
    free(state->queue);
    free(state->transient_obstacles_queue);
    if (state->scan) free(state->scan);
    cs_free_matrix(state->covariance, 3, 3);
    cs_free_matrix(state->laser_position, 4, 4);
    free(state);
    return 1;
}

int 
cs_is_state(void *state)
{
    cs_state_t *s = (cs_state_t*)state;
    if (s && s->magic == CS_MAGIC)
        return 1;
    return 0;
}

int 
cs_set_attitude(cs_state_t *state, double pitch, double roll)
{
    double cpitch, spitch, croll, sroll;
    CS_CHECK_MAGIC;
    state->pitch = pitch;
    state->roll = roll;
    return 1; 
}

void
cs_build_scan(cs_state_t *state, cs_sensor_data_t *sd, cs_scan_t *scan)
{
    int i, j, span = 1;
    double angle_rad, angle_deg;
    cs_matrix_t m, attitude;
    cs_vector_t v, w;
    double cpitch, spitch, croll, sroll;
   
    // Allocate matrix m 
    const int NRH = 4, NCH = 4;
    m = ((cs_matrix_t )alloca(NRH * (sizeof(cs_float_t*) + NCH * sizeof(cs_float_t)))) - 1;
    for(i = 1; i <= NRH; i++) {
	m[i] = (cs_vector_t )(((char*)(m + 1)) + NRH * sizeof(cs_float_t*) + (i - 1) * NCH * sizeof(cs_float_t));
	m[i]--;
    }
    // Allocate matrix attitude 
    attitude = ((cs_matrix_t )alloca(NRH * (sizeof(cs_float_t*) + NCH * sizeof(cs_float_t)))) - 1;
    for(i = 1; i <= NRH; i++) {
	attitude[i] = (cs_vector_t )(((char*)(attitude + 1)) + NRH * sizeof(cs_float_t*) + (i - 1) * NCH * sizeof(cs_float_t));
	attitude[i]--;
    }
    v = ((cs_vector_t )alloca(NRH * sizeof(cs_float_t))) - 1;
    w = ((cs_vector_t )alloca(NRH * sizeof(cs_float_t))) - 1;
    
    // Attitude computation
    cpitch = cos(sd->pitch * M_PI / 180);
    spitch = sin(sd->pitch * M_PI / 180);
    croll = cos(sd->roll * M_PI / 180);
    sroll = sin(sd->roll * M_PI / 180);
    
    attitude[1][1] = cpitch;
    attitude[2][1] = 0;
    attitude[3][1] = -spitch;
    attitude[4][1] = 0;
    attitude[1][2] = sroll * spitch;
    attitude[2][2] = croll;
    attitude[3][2] = sroll * cpitch;
    attitude[4][2] = 0;
    attitude[1][3] = croll * spitch;
    attitude[2][3] = -sroll;
    attitude[3][3] = croll * cpitch;
    attitude[4][3] = 0;
    attitude[1][4] = attitude[2][4] = attitude[3][4] = 0;
    attitude[4][4] = 1;
    
    cs_matrix_multiply(attitude, state->laser_position, 4, 4, 4, m);
    
    scan->nb_points = 0;
    scan->nb_good_points = 0;
    
    // Span the laser scans to better cover the space
    for (i = 0; i < state->laser_parameters.scan_size; i++) {
        for (j = 0; j != span; j++) {
            angle_deg = -(state->laser_parameters.scan_angle / 2) + ((double)(i * span + j)) * (state->laser_parameters.scan_angle) / (state->laser_parameters.scan_size * span - 1);
            angle_deg += sd->psidot * state->laser_parameters.scan_duration / (360.0 * 1000 /*ms*/) * ((double)(i * span + j)) * (state->laser_parameters.scan_angle) / (state->laser_parameters.scan_size * span - 1);

			if (state->laser_parameters.upside_down)
                angle_deg = -angle_deg;
            if (state->laser_parameters.angle_start != 0xdead && angle_deg < state->laser_parameters.angle_start)
                continue;
            if (state->laser_parameters.angle_end != 0xdead && angle_deg > state->laser_parameters.angle_end) 
                continue;

            angle_rad = angle_deg * M_PI / 180;
            if (i >= state->laser_parameters.detection_margin && i < state->laser_parameters.scan_size - state->laser_parameters.detection_margin) {
                if (sd->d[i] == 0 || sd->d[i] > state->laser_parameters.max_distance) { // Bad or no distance reading
                    double z;
                    v[1] = state->laser_parameters.distance_no_detection * cos(angle_rad);
                    v[1] -= sd->v * 1000 * ((double)(i * span + j)) * (state->laser_parameters.scan_angle) / (state->laser_parameters.scan_size * span - 1) / 3600.0;
                    v[2] = state->laser_parameters.distance_no_detection * sin(angle_rad);
                    v[3] = 0;
                    v[4] = 1;
                    cs_matrix_vect_multiply(m, v, 4, 4, w);
                    z = 1 / w[4];
                    scan->points[scan->nb_points].x = (float)(w[1] * z);
                    scan->points[scan->nb_points].y = (float)(w[2] * z);
                    scan->points[scan->nb_points].z = (float)(w[3] * z);
                    scan->points[scan->nb_points].value = 0;
                    scan->nb_points++;
                } else if (sd->d[i] > state->laser_parameters.minimum_distance) {
                    int keep_it = 1;
                    if (state->laser_parameters.filter) {
                        // Compare with previous or next measures
                        if (i > 0 && abs(sd->d[i] - sd->d[i - 1]) * 1000 > state->laser_parameters.filter * sd->d[i]
                            && i < state->laser_parameters.scan_size - 1 && abs(sd->d[i] - sd->d[i + 1]) * 1000 > state->laser_parameters.filter * sd->d[i])
                            keep_it = 0;
                    }
                    if (keep_it) {
                        double z;
                        v[1] = sd->d[i] * cos(angle_rad);
                        v[1] -= sd->v * 1000 * ((double)(i * span + j)) * (state->laser_parameters.scan_angle) / (state->laser_parameters.scan_size * span - 1) / 3600.0;
                        v[2] = sd->d[i] * sin(angle_rad);
                        v[3] = 0;
                        v[4] = 1;
                        cs_matrix_vect_multiply(m, v, 4, 4, w);
                        z = 1 / w[4];
                        scan->points[scan->nb_points].x = (float)(w[1] * z);
                        scan->points[scan->nb_points].y = (float)(w[2] * z);
                        scan->points[scan->nb_points].z = (float)(w[3] * z);
                        // Ground rejection
                        if (scan->points[scan->nb_points].z < state->param_ground_rejection) 
                            scan->points[scan->nb_points].value = 0;
                        else {
                            scan->points[scan->nb_points].value = sd->d[i];
                            scan->nb_good_points++;
                        }
                        scan->nb_points++;
                    }
                }
            }
        }
    }
}

cs_sensor_data_t * 
cs_update_robot_position(cs_state_t *state, int *laser_data, int data_size, cs_position_t *robot_position, int options)
{   
    if (data_size != state->laser_parameters.scan_size) {
        fprintf(stderr, "Wrong scan width for laser data (%d against %d expected)\n", data_size, state->laser_parameters.scan_size);
        return NULL;
    }
    if (state->counter) {
        cs_sensor_data_t *retval;

        if (state->counter < state->latency + CS_FILTER_SIZE) {
            // Send again the very first scan (TODO : change this behavior, though this is not very important since it's
            // regarding only the startup=
            retval = &state->latent_sensor_data[0];
        } else {
            // Send the data with latency
            int pos = (state->counter - state->latency) % (state->latency + CS_FILTER_SIZE);
            int filtered_out = 0;
            cs_position_t filtered_position;

            // Filter latent data in order to find coherency points
            {
                int i;
                const int weight[CS_FILTER_SIZE * 2 + 1] = {1, 2, 4, 2, 1};
                const int total_weight = 10;
                filtered_position.x = 0; filtered_position.y = 0; filtered_position.theta = 0;
                for (i = 0; i != CS_FILTER_SIZE * 2 + 1; i++) {
                    int posx = pos - CS_FILTER_SIZE + i;
                    if (posx < 0) posx += state->latency + CS_FILTER_SIZE;
                    else if (posx >= state->latency + CS_FILTER_SIZE) posx -= state->latency + CS_FILTER_SIZE;
                    filtered_position.x += weight[i] * state->latent_sensor_data[posx].position.x;
                    filtered_position.y += weight[i] * state->latent_sensor_data[posx].position.y;
                    filtered_position.theta += weight[i] * state->latent_sensor_data[posx].position.theta;                   
                    // TODO : Fix troubles around 360° 
                }
                filtered_position.x /= total_weight;
                filtered_position.y /= total_weight;
                filtered_position.theta /= total_weight;
            } 

            {
                int i;
                cs_sensor_data_t *sd = &state->latent_sensor_data[pos];
                if (fabs(filtered_position.theta - sd->position.theta) > 2) {
                    filtered_out = 1;
#ifdef CS_DEBUG_MESSAGES
                    fprintf(stderr, "Too much difference between filtered theta and original theta\n");
#endif // CS_DEBUG_MESSAGES
                }

                for (i = 0; i != CS_FILTER_SIZE * 2 + 1; i++) {
                    cs_sensor_data_t *sdx;
                    int posx = pos - CS_FILTER_SIZE + i;
                    if (posx < 0) posx += state->latency + CS_FILTER_SIZE;
                    else if (posx >= state->latency + CS_FILTER_SIZE) posx -= state->latency + CS_FILTER_SIZE;
                    sdx = &state->latent_sensor_data[posx];
                    // Filter out bad scores
                    if (sd->score >= sdx->score * 2) {
                        filtered_out = 1;
#ifdef CS_DEBUG_MESSAGES
                        fprintf(stderr, "Too much difference between scores (%d > %d)\n", sd->score, sdx->score);
#endif // CS_DEBUG_MESSAGES
                    }
                }

#define CS_AVG_SCORE_FILTER 3
                // Filter based on average score
                if (sd->score > state->delayed_avg_score * CS_AVG_SCORE_FILTER) { // || (state->previous_failed && sd->score > state->delayed_avg_score))) {
                    filtered_out = 1;
                    state->previous_failed++;
#ifdef CS_DEBUG_MESSAGES
                    fprintf(stderr, "Filtered out based on average score (%d > %d)\n", sd->score, state->delayed_avg_score);
#endif // CS_DEBUG_MESSAGES
                } else state->previous_failed = 0;
            
                // Stationnarity filter
                {
                    cs_position_t *px;
                    int posx = pos - 1;
                    double dx, dy, dtheta;
                    if (posx < 0) posx += state->latency + CS_FILTER_SIZE;
                    if (state->stationnarity_counter)
                        px = &state->stationnary_position;
                    else 
                        px = &state->latent_sensor_data[posx].position;
                    // Filter out stationnary positions
                    dx = fabs(px->x - sd->position.x);
                    dy = fabs(px->y - sd->position.y);
                    dtheta = px->theta - sd->position.theta;
                    if (dtheta > 180) dtheta -= 360;
                    else if (dtheta < -180) dtheta += 360;
                    dtheta = fabs(dtheta);
                    if ( dx < state->param_stationnarity_posxy &&
                         dy < state->param_stationnarity_posxy &&
                         dtheta < state->param_stationnarity_theta) {
                        if (!state->stationnarity_counter)
                            state->stationnary_position = sd->position; // Record position in order to further compare to this one
                        state->stationnarity_counter++;
                        fprintf(stderr, "Robot is stationnary (counter = %d).", state->stationnarity_counter);
                        if (state->stationnarity_counter > state->param_stationnarity_max_counter) {
                            filtered_out = 1;
                            fprintf(stderr, "Filtered out.");
                        }
                        fprintf(stderr, "\n");
                    } else {
                        if (state->stationnarity_counter) {
                            fprintf(stderr, "dx = %lf, dy = %lf, dtheta = %lf\n", dx, dy, dtheta); 
                        }
                        state->stationnarity_counter = 0;
                    }
                }
            }

            // Filter average score, whatever happens
            state->delayed_avg_score = (state->delayed_avg_score * 9 + state->latent_sensor_data[pos].score) / 10;

            if (filtered_out) {
#ifdef CS_DEBUG_MESSAGES
                fprintf(stderr, "Bad data association. I don't update grid_map...\n");
#endif // CS_DEBUG_MESSAGES
                retval = NULL;
            } else {
                retval = &state->latent_sensor_data[pos];
            }
        }

        {
            int pos = state->counter % (state->latency + CS_FILTER_SIZE);
            cs_sensor_data_t *sd = &state->latent_sensor_data[pos];
            int i;
            cs_position_t position, new_position;
            double thetarad = state->position.theta * M_PI / 180;

            // Store the laser data along with the current robot position
            for (i = 0; i < state->laser_parameters.scan_size; i++)
                sd->d[i] = laser_data[i];
            sd->v = 0;
            sd->psidot = 0;
            sd->timestamp = state->counter;
            
            // Evaluate robot position
            if (robot_position) {
                // Use of odometry or external position source
                position = *robot_position;
            } else {
                // If we have no odometry or external position source, we filter the speed of the robot
                position = state->position;
                /* TODO : Remove. Incompatible with robot model.
                position.x += state->speed * cos(thetarad);
                position.y += state->speed * sin(thetarad);
                */
            }

            // Translate position according to map origin
            position.x -= state->map_origin_x * 0.001; position.y -= state->map_origin_y * 0.001; 

            // Create the (x, y) scan
            cs_build_scan(state, sd, state->scan);
            
            // The scan to map matching function
            if (options & CS_ROBOT_NOT_MOVING && state->ok) {
                new_position = position;
                state->score = 1;
            } else {
                // Multiscale search
                new_position = cs_multiscale_search(state, state->scan, &position);
            }
            if (state->score >= 200000000) { // Very bad score
                new_position = position; // Don't move...
            }

            // Make sure new_position angle is within 0 and 360°
            if (new_position.theta < 0) new_position.theta += 360;
            else if (new_position.theta >= 360) new_position.theta -= 360;

#if 0
            if (!(state->counter % 500)) {
                cs_test_around_position(state, scan, &new_position, state->sigma_xy, state->sigma_theta);
            }
#endif

            // Filter the speed
            {
                double v = sqrt((new_position.x - position.x) * (new_position.x - position.x) +
                    (new_position.y - position.y) * (new_position.y - position.y));
                state->speed = (3 * state->speed + v) / 4;
            }
            
            // Translate to absolute position
            new_position.x += state->map_origin_x * 0.001; new_position.y += state->map_origin_y * 0.001; 
            state->position = new_position;

            sd->score = state->score;
            if (state->mode == CS_MODE_MAPPING) 
                sd->mapping = 1;
            else sd->mapping = 0;
            sd->position = state->position;
            sd->pitch = state->pitch;
            sd->roll = state->roll;
            sd->altitude = state->altitude;

            // Is this OK ?    
            state->ok = 1;
            if (state->counter > CS_FILTER_SIZE) {
                for (i = 0; i != CS_FILTER_SIZE; i++) {
                    cs_sensor_data_t *sdx;
                    int posx = pos - CS_FILTER_SIZE + i;
                    if (posx < 0) posx += state->latency + CS_FILTER_SIZE;
                    else if (posx >= state->latency + CS_FILTER_SIZE) posx -= state->latency + CS_FILTER_SIZE;
                    sdx = &state->latent_sensor_data[posx];
                    // Filter out bad scores
                    if (state->score > 100000 + sdx->score * 6) {
                        state->ok = 0;
#ifdef CS_DEBUG_MESSAGES
                        fprintf(stderr, "Current data : too much difference between scores (%d > %d)\n", sd->score, sdx->score);
#endif // CS_DEBUG_MESSAGES
                    }
                    // Filter based on average score
                    if (state->score > 100000 + state->current_avg_score * 3) { // || (state->previous_failed && sd->score > state->delayed_avg_score))) {
                        state->ok = 0;
#ifdef CS_DEBUG_MESSAGES
                        fprintf(stderr, "Current data : Filtered out based on average score (%d > %d)\n", state->score, state->delayed_avg_score);
#endif // CS_DEBUG_MESSAGES
                    }
                }
            }

            // Filter average score, whatever happens
            state->current_avg_score = (state->current_avg_score * 9 + state->score) / 10;
        }
        state->counter++;
        return retval;
    } else {
        // No latency on the first scan, which is considered perfect
        cs_sensor_data_t *sd = &state->latent_sensor_data[0];
        int i;

        for (i = 0; i < state->laser_parameters.scan_size; i++)
            sd->d[i] = laser_data[i];
        
        if (state->mode == CS_MODE_MAPPING) 
            sd->mapping = 1; // These data can be integrated in map
        else sd->mapping = 0;

        sd->v = 0;
        sd->psidot = 0;
        sd->position = state->position;
        sd->timestamp = 0;
        sd->score = 0;
        sd->pitch = state->pitch;
        sd->roll = state->roll;
        sd->altitude = state->altitude;
        state->ok = 1;
            
        // Create the (x, y) scan
        cs_build_scan(state, sd, state->scan);
            
        state->counter++;
        return sd;
    }
}

int
cs_robot_position_to_map_coordinates(cs_state_t *state, cs_position_t *position, int *x, int *y)
{
    cs_position_t pos;
    CS_CHECK_MAGIC;
    if (position) {
        pos.x = position->x;
        pos.y = position->y;
    } else {
        pos.x = state->position.x;
        pos.y = state->position.y;
    }
    // Translate position according to grid_map origin
    pos.x -= state->map_origin_x * 0.001; pos.y -= state->map_origin_y * 0.001;
    *x = (int)(pos.x * 1000 * CS_MAP_SCALE + 0.5); 
    *y = (int)(pos.y * 1000 * CS_MAP_SCALE + 0.5);
    return 1;
}

int
cs_map_coordinates_to_robot_position(cs_state_t *state, int x, int y, cs_position_t *position)
{
    CS_CHECK_MAGIC;
    if (position == NULL)
        return 0;
    position->x = (x * state->resxy + state->map_origin_x) * 0.001;
    position->y = (y * state->resxy + state->map_origin_y) * 0.001; 
    return 1;
}

/* if in RELOC mode, returns a value different of 0. If in MAPPING mode, returns 0.
 * If the latter case, the system should stop calling cs_step until there is a new scan. */
int 
cs_localize(cs_state_t *state, int options)
{
    const int CS_MS_POPULATION_SIZE = 1000;
    const int CS_MS_NEVAL = CS_MS_POPULATION_SIZE * 2;
    cs_distance_scan_to_map_part_t *population, **pqueue;
    int i, j, retval = 1;
    int scan_order[CS_SCAN_SIZE];
    cs_3dpoint_t *protated[1];
    cs_3dpoint_t rotated[CS_SCAN_SIZE];
    float theta, c, s;
    cs_3dpoint_t *r;
   
    if (state->mode != CS_MODE_RELOC) 
        return 0;

    cs_init_scan_order(state->scan, scan_order);
    
    population = (cs_distance_scan_to_map_part_t*)malloc(sizeof(cs_distance_scan_to_map_part_t) * CS_MS_POPULATION_SIZE);

    protated[0] = rotated;
    theta = (rand() % 3600) * 0.1;
    // Pre-rotation of points
    c = (float)(cos(theta * M_PI / 180) * CS_MAP_SCALE);
    s = (float)(sin(theta * M_PI / 180) * CS_MAP_SCALE);
    r = rotated;
    for (i = 0; i < state->scan->nb_good_points; i++) {
        j = scan_order[i];
        r[i].value = state->scan->points[j].value;
        r[i].x = c * state->scan->points[j].x - s * state->scan->points[j].y + 0.5f;
        r[i].y = s * state->scan->points[j].x + c * state->scan->points[j].y + 0.5f;
    }

    // Start with a random population
    for (i = 0; i != CS_MS_POPULATION_SIZE;) {
        int x = rand() % (CS_MAP_SIZE - 1);
        int y = rand() % (CS_MAP_SIZE - 1);

        int v = state->grid_map[(y << state->map_size_bits) + x];
        if (v != 128) {
            population[i].pos.x = x * state->resxy * 0.001;
            population[i].pos.y = y * state->resxy * 0.001;
            population[i].pos.theta = theta;
            population[i].which_theta = 0;
            i++;
        }
    }

    // Evaluate the population
    pqueue = cs_eval_positions(state, state->scan, scan_order, population, CS_MS_POPULATION_SIZE, CS_MS_NEVAL, protated);
#if 0
    {
        for (i = 0; i != CS_MS_POPULATION_SIZE; i++) {
            printf("%lg %lg %lg (%d - %d)\n", 
                    pqueue[i]->pos.x, pqueue[i]->pos.y, pqueue[i]->pos.theta,
                    pqueue[i]->score, pqueue[i]->already_done);
        }
        printf("------\n");
    }
#endif

    if (pqueue[0]->score < state->score) {
        retval = 2;
        fprintf(stderr, "CoreSLAM localization : found a better hypothesis\n");
        state->reloc_counter = 0;

        state->position = pqueue[0]->pos;
        // Move to absolute position
        state->position.x += state->map_origin_x * 0.001; 
        state->position.y += state->map_origin_y * 0.001; 
        state->score = pqueue[0]->score;
    }
    free(pqueue);
    free(population);

    return retval;
}

int
cs_step(cs_state_t *state, int *laser_data, int data_size, cs_position_t *robot_position, int options)
{
    CS_CHECK_MAGIC;
    
    if (laser_data) {
        cs_sensor_data_t *sd;
        cs_position_t oldpos;

        if (data_size != state->laser_parameters.scan_size) {
			fprintf(stderr, "Wrong scan width for laser data (%d against %d expected)\n", data_size, state->laser_parameters.scan_size);  
            return 0;
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
                        cs_map_set(state, x, y, 255);
                    }
                } else break; // Don't check the tail
            }
        }

        oldpos = state->position;
        sd = cs_update_robot_position(state, laser_data, data_size, robot_position, options);
        
        // Update altitude
        if (state->update_altitude_wrt_attitude) {
            double dx = state->position.x - oldpos.x;
            double dy = state->position.y - oldpos.y;
            double c = cos(state->position.theta * M_PI / 180);
            double s = sin(state->position.theta * M_PI / 180);
            double v = dx * c + dy * s;
            state->altitude -= v * sin(state->pitch * M_PI / 180); 
        } else {
            // Get altitude through zmap
            int x, y;
            cs_robot_position_to_map_coordinates(state, &state->position, &x, &y); 
            if (x >= 0 && y >= 0 && x < CS_MAP_SIZE && y < CS_MAP_SIZE) {
                int16_t *zmap_ptr = &state->zmap[(y << CS_MAP_SIZE_BITS) + x];
                state->altitude = *zmap_ptr * 0.001; // zmap is in mm, altitude is in meters
            }
        }

        // Delayed grid_map update
        if (sd && state->mode == CS_MODE_MAPPING && state->update_map && sd->mapping) {
            // A grid_map update is required
            cs_scan_t scan2map;
            cs_position_t position;
            cs_build_scan(state, sd, &scan2map);
            position = sd->position;
            // Position is an absolute position. Move it to a local grid_map position
            position.x -= state->map_origin_x * 0.001; position.y -= state->map_origin_y * 0.001;
            cs_map_update(state, &scan2map, &position);
        }
        if (state->mode == CS_MODE_RELOC) {
            state->reloc_counter++;
            if (state->reloc_counter == state->auto_switch_to_mapping) {
                fprintf(stderr, "CoreSLAM  : localisation may be OK. Auto switching to mapping mode\n");
                state->mode = CS_MODE_MAPPING;
                state->altitude = 0;
                state->reloc_counter = 0;
            }
        }
        if (state->mode == CS_MODE_RELOC && state->score > 0 && state->current_avg_score < state->switch_to_mapping_threshold) {
            fprintf(stderr, "CoreSLAM  : localization OK. Switching to mapping mode\n");
            state->mode = CS_MODE_MAPPING;
            state->altitude = 0;
            state->reloc_counter = 0;
        }
        if (state->mode != CS_MODE_RELOC && state->current_avg_score > state->switch_to_reloc_threshold) {
            fprintf(stderr, "CoreSLAM : robot is lost. Switching to reloc mode. Score is %d\n", state->score);
            state->mode = CS_MODE_RELOC;
            state->reloc_counter = 0;
        }
        // New points update (no latency)
        if (state->scan && state->mode == CS_MODE_MAPPING && state->update_map && state->ok) {
            cs_position_t position = state->position;
            // Position is an absolute position. Move it to a local grid_map position
            position.x -= state->map_origin_x * 0.001; position.y -= state->map_origin_y * 0.001;
            cs_map_update_new_points(state, state->scan, &position);
        }
        
        state->debug_info[2] = state->current_avg_score;

        // Update altitude according to inertial measurement and movement 
        cs_send_diff_data(state);
        cs_send_zmap_diff_data(state);
        cs_send_position_and_attitude(state);
        return 1;
    } else if (state->counter) {
        return cs_localize(state, options);
    }
    return 0;
}

// val <= 0 : obstacle. Negative value for persistent obstacle
// >= 255 : no obstacle.
// In between, at least discovered (first time set to no obstacle)
int
cs_map_set(cs_state_t *state, int x, int y, int val)
{
    cs_map_pixel_t *pix;
    CS_CHECK_MAGIC;

    if (x >= 0 && y >= 0 && x < CS_MAP_SIZE && y < CS_MAP_SIZE) {
        // Pix first points to the shield map
        pix = &state->shield_map[(y << CS_MAP_SIZE_BITS) + x];
        if (!*pix) { // If it is not protected
            // pix points to the right place in the grid map
            pix = &state->grid_map[(y << CS_MAP_SIZE_BITS) + x];
            if (val <= 0) { // Supposedly an obstacle
                *pix = 0;
                {
                    // Update obstacle map
                    cs_map_pixel_t *pobs = &state->obstacle_map[(y << CS_MAP_SIZE_BITS) + x];
                    int value = *pobs;
                    if (value == 0 || value == CDM_UNEXPLORED) {
                        // It was not an obstacle before. Update hole map
                        value = 0;
                        dmap_set_obstacle(state, state->hole_map, x, y);
                        {
                            // Add to diff table
                            cs_diff_point_t *point = &state->diff_points[state->diff_size++];
                            point->x = x; point->y = y; point->z = 0;
                            point->plusminus = 1;
                            if (state->diff_size == CS_MAX_DIFF_DATA)
                                cs_send_diff_data(state);
                        }{
                            // Push to transient obstacles queue
                            cs_transient_obstacle_data_t *data = &state->transient_obstacles_queue[state->transient_obstacles_queue_end];
                            state->transient_obstacles_queue_end++;
                            if (state->transient_obstacles_queue_end == state->transient_obstacles_queue_size)
                                state->transient_obstacles_queue_end = 0;
                            if (state->transient_obstacles_queue_end == state->transient_obstacles_queue_start) {
                                fprintf(stderr, "CoreSLAM : transient obstacle queue full. Internal system error.\n");
                                return 0;
                            }
                            data->map_index = (y << CS_MAP_SIZE_BITS) + x;
                            data->timestamp = state->counter;
                        }
                    }
                    if (val < 0) 
                        *pobs = CDM_UNEXPLORED - 1; // Special value : persistant obstacle
                    else {
                        value++; // Increment obstacle persistence
                        if (value == CDM_UNEXPLORED) value = CDM_UNEXPLORED - 1;
                        *pobs = value;
                    }
                }
            } else {
                if (val > 255) val = 255; // Saturation
                if (*pix != val) {
                    *pix = val;
                    if (val == 255) {
                        cs_map_pixel_t *pobs = &state->obstacle_map[(y << CS_MAP_SIZE_BITS) + x];
                        *pobs = 0;
                        dmap_set_noobstacle(state, state->hole_map, x, y);
                        {
                            // add to diff table
                            cs_diff_point_t *point = &state->diff_points[state->diff_size++];
                            point->x = x; point->y = y; point->z = 0;
                            point->plusminus = 0;
                            if (state->diff_size == CS_MAX_DIFF_DATA)
                                cs_send_diff_data(state);
                        }
                    } else {
                        cs_map_pixel_t *pobs = &state->obstacle_map[(y << CS_MAP_SIZE_BITS) + x];
                        if (*pobs == CDM_UNEXPLORED) {
                            *pobs = 0; // Set as non obstacle
                            dmap_set_explored(state, state->hole_map, x, y, NULL);
                            // add to diff table
                            {
                                cs_diff_point_t *point = &state->diff_points[state->diff_size++];
                                point->x = x; point->y = y; point->z = -1;
                                point->plusminus = 1;
                                if (state->diff_size == CS_MAX_DIFF_DATA)
                                    cs_send_diff_data(state);
                            }
                        }
                    }
                }
            }
        }
        return 1;
    }
    return 0; 
}

int 
cs_zmap_set(cs_state_t *state, int x1, int y1, int x2, int y2, int z)
{
    int16_t *pix;
	int x, y;

    CS_CHECK_MAGIC;

    // Check validity of parameters
    if (x1 >= 0 && y1 >= 0 && x1 < CS_MAP_SIZE && y1 < CS_MAP_SIZE &&
        x2 >= 0 && y2 >= 0 && x2 < CS_MAP_SIZE && y2 < CS_MAP_SIZE) {
        int changed = 0;
        if (x2 < x1) {
            x2 = x1 ^ x2;
            x1 = x1 ^ x2;
            x2 = x1 ^ x2;
        }
        if (y2 < y1) {
            y2 = y1 ^ y2;
            y1 = y1 ^ y2;
            y2 = y1 ^ y2;
        }
        // OK. Now everything is in the right order.
        // Make sure that we are changing something in the z map
        for (y = y1; y <= y2; y++) {
            for (x = x1; x <= x2; x++) {
                pix = &state->zmap[(y << CS_MAP_SIZE_BITS) + x];
                if (*pix != z) {
                    changed = 1;
                    *pix = z;
                }
            }
        }
        // Send diff if needed
        if (changed) {
            // add to diff table
            cs_zmap_diff_zone_t *zone = &state->zmap_diff_zone[state->zmap_diff_size++];
            zone->x1 = x1; zone->x2 = x2; zone->y1 = y1; zone->y2 = y2;
            zone->z = z;
            if (state->zmap_diff_size == CS_MAX_DIFF_DATA)
                cs_send_zmap_diff_data(state);
        }
        return 1;
    }
    return 0;
}

void
cs_send_position_and_attitude(cs_state_t *state)
{
    int i;
#define POSITION_AND_ATTITUDE_DATA_SIZE 7
    int32_t buffer[POSITION_AND_ATTITUDE_DATA_SIZE];
    cs_position_t pos = cs_get_robot_position(state);
    
    // Fill the buffer
    buffer[0] = 2; // Position command
    buffer[1] = (int)(pos.x * 1000);
    buffer[2] = (int)(pos.y * 1000);
    buffer[3] = (int)(pos.theta * 100);
    buffer[4] = (int)(state->pitch * 100);
    buffer[5] = (int)(state->roll * 100);
    buffer[6] = (int)(state->altitude * 1000);
    // Add attitude data
    
    // Send to clients
    for (i = 0; i < state->nb_clients; i++)
        if (state->clients[i].send_fct)
            state->clients[i].send_fct((const char*)buffer, POSITION_AND_ATTITUDE_DATA_SIZE * sizeof(uint32_t), state->clients[i].context);
}

void
cs_send_diff_data(cs_state_t *state)
{
    int i;
    unsigned char buffer[sizeof(cs_diff_point_t) * CS_MAX_DIFF_DATA + 2 * sizeof(uint32_t)];
    
    if (state->diff_size) {
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
}

void
cs_send_zmap_diff_data(cs_state_t *state)
{
    int i;
    unsigned char buffer[sizeof(cs_zmap_diff_zone_t) * CS_MAX_DIFF_DATA + 2 * sizeof(uint32_t)];
    
    if (state->zmap_diff_size) {
        // Fill the buffer
        ((uint32_t*)buffer)[0] = 3;
        ((uint32_t*)buffer)[1] = state->diff_size;
        memcpy(buffer + 2 * sizeof(uint32_t), state->zmap_diff_zone, state->zmap_diff_size * sizeof(cs_zmap_diff_zone_t));

        // Send to clients
        for (i = 0; i < state->nb_clients; i++)
            if (state->clients[i].send_fct)
                state->clients[i].send_fct(buffer, 2 * sizeof(uint32_t) + state->zmap_diff_size * sizeof(cs_zmap_diff_zone_t), state->clients[i].context);

        state->zmap_diff_size = 0;
    }
}

cs_position_t
cs_get_robot_position(cs_state_t *state)
{
    return state->position;
}

double 
cs_get_attitude(cs_state_t *state, int which)
{
    CS_CHECK_MAGIC;
    switch (which) {
        case CS_PITCH:
            return state->pitch;
        case CS_ROLL:
            return state->roll;
        case CS_ALTITUDE:
            return state->altitude;
    }
    return 314.5;
}

int
cs_get_score(cs_state_t *state)
{
    CS_CHECK_MAGIC;
    return state->score;
}

double
cs_get_model_error(cs_state_t *state)
{
    CS_CHECK_MAGIC;
    return state->model_error;
}

int
cs_is_ok(cs_state_t *state)
{
    CS_CHECK_MAGIC;
    return state->ok;
}

cs_matrix_t 
cs_get_covariance(cs_state_t *state)
{
    return state->covariance;
}

cs_scan_t *
cs_get_scan(cs_state_t *state)
{
    return state->scan;
}

int
cs_get_debug_info(cs_state_t *state, int debug_info)
{
    CS_CHECK_MAGIC;
    if (debug_info >=0 && debug_info < CS_MAX_DEBUG_INFO)
        return state->debug_info[debug_info];
    return -1;
}

int
cs_set_debug_info(cs_state_t *state, int debug_info, int value)
{
    CS_CHECK_MAGIC;
    if (debug_info >=0 && debug_info < CS_MAX_DEBUG_INFO) {
        state->debug_info[debug_info] = value;
        return 1;
    }
    return 0;
}

int
cs_get_param(cs_state_t *state, int param)
{
    CS_CHECK_MAGIC;
    switch (param) {
        case CS_PARAM_MAP_SIZE:
            return CS_MAP_SIZE;
        case CS_PARAM_MAP_RESOLUTION:
            return state->resxy;
        case CS_PARAM_Z_RESOLUTION:
            return state->resz;
        case CS_PARAM_MAP_ORIGIN_X:
            return state->map_origin_x;
        case CS_PARAM_MAP_ORIGIN_Y:
            return state->map_origin_y;
        case CS_PARAM_SWITCH_TO_MAPPING_THRESHOLD:
            return state->switch_to_mapping_threshold;
        case CS_PARAM_SWITCH_TO_RELOC_THRESHOLD:
            return state->switch_to_reloc_threshold;
        case CS_PARAM_MINIMUM_DISTANCE:
            return state->laser_parameters.minimum_distance;
        case CS_PARAM_ANGLE_START:
            return state->laser_parameters.angle_start;
        case CS_PARAM_ANGLE_END:
            return state->laser_parameters.angle_end;
        case CS_PARAM_GROUND_REJECTION:
            return state->param_ground_rejection;
        case CS_PARAM_AUTO_SWITCH_TO_MAPPING:
            return state->auto_switch_to_mapping;
        case CS_PARAM_LATENCY:
            return state->latency;
        case CS_PARAM_SIGMA_XY:
            return (int)(state->sigma_xy * 100);
        case CS_PARAM_SIGMA_THETA:
            return (int)(state->sigma_theta * 100);
        case CS_PARAM_NEVAL:
            return state->neval;
    }
    return 0;
}

int 
cs_set_param(cs_state_t *state, int param, int value)
{
    CS_CHECK_MAGIC;
    switch (param) {
        case CS_PARAM_MAP_RESOLUTION :
            if (value > 0) {
                state->resxy = value;
                return cs_realloc_maps(state);
            }
            break;
        case CS_PARAM_Z_RESOLUTION :
            if (value > 0) {
                state->resz = value;
                return cs_realloc_maps(state);
            }
            break;
        case CS_PARAM_MAP_ORIGIN_X :
            state->map_origin_x = value;
            return 1;
        case CS_PARAM_MAP_ORIGIN_Y :
            state->map_origin_y = value;
            return 1;
        case CS_PARAM_MAP_SIZE :
            // Accepts only power of 2 sizes
            {
                int v = 1;
                while ((1 << v) < value) v++;
                if ((1 << v) == value) {
                    state->map_size_bits = v;
                    return cs_realloc_maps(state);
                } else return 0;
            }
            return 1;
        case CS_PARAM_UPDATE_MAP :
            state->update_map = value;
            return 1;
        case CS_PARAM_SWITCH_TO_MAPPING_THRESHOLD:
            if (value > 0) {
                state->switch_to_mapping_threshold = value;
                return 1;
            }
            return 1;
        case CS_PARAM_SWITCH_TO_RELOC_THRESHOLD :
            if (value > 0) {
                state->switch_to_reloc_threshold = value;
                return 1;
            }
            return 0;
        case CS_PARAM_MINIMUM_DISTANCE :
            if (value > 0) {
                state->laser_parameters.minimum_distance = value;
                return 1;
            }
            return 0;
        case CS_PARAM_ANGLE_START :
            state->laser_parameters.angle_start = value;
            return 1;
        case CS_PARAM_ANGLE_END :
            state->laser_parameters.angle_end = value;
            return 1;
        case CS_PARAM_GROUND_REJECTION :
            state->param_ground_rejection = value;
            return 1;
        case CS_PARAM_AUTO_SWITCH_TO_MAPPING :
            if (value > 0) {
                state->auto_switch_to_mapping = value;
                return 1;
            }
            return 0;
        case CS_PARAM_LATENCY :
            if (value > 2) {
                state->latency = value;
                return 1;
            }
            return 0;
        case CS_PARAM_SIGMA_XY :
            if (value > 0) {
                state->sigma_xy = value * 0.01;
                return 1;
            }
            return 0;
        case CS_PARAM_SIGMA_THETA :
            if (value > 0) {
                state->sigma_theta = value * 0.01;
                return 1;
            }
            return 0;
        case CS_PARAM_NEVAL :
            if (value > 0) {
                state->neval = value;
                return 1;
            }
            return 0;
        case CS_PARAM_GET_RID_OF_OUTLIERS :
            state->get_rid_of_outliers = value;
            return 1;
        case CS_PARAM_MINRES_XY :
            if (value > 0) {
                state->min_search_res_xy = value * 0.001;
                return 1;
            }
            return 0;
        case CS_PARAM_MINRES_THETA :
            if (value > 0) {
                state->min_search_res_theta = value * 0.01;
                return 1;
            }
            return 0;
        case CS_PARAM_TRANSIENT_OBSTACLES_DELAY :
            if (value > 0) {
                state->transient_obstacles_delay = value;
                return 1;
            }
            return 0;
        case CS_PARAM_TRANSIENT_OBSTACLES_THRESHOLD :
            if (value > 0) {
                state->transient_obstacles_threshold = value;
                return 1;
            }
            return 0;
        case CS_PARAM_MAGIC_RATIO_MAPPING :
            if (value > 0) {
                state->param_magic_ratio[CS_MODE_MAPPING] = value;
                return 1;
            }
            return 0;
        case CS_PARAM_MAGIC_RATIO_RELOC :
            if (value > 0) {
                state->param_magic_ratio[CS_MODE_RELOC] = value;
                return 1;
            }
            return 0;
    }
    return 0;
}

int 
cs_set_mode(cs_state_t *state, int mode)
{
    CS_CHECK_MAGIC;
    if (mode == CS_MODE_RELOC || mode == CS_MODE_MAPPING) {
        state->mode = mode;
        return 1;
    }
    return 0;
}

int 
cs_get_mode(cs_state_t *state)
{
    CS_CHECK_MAGIC;
    return state->mode;
}

cs_matrix_t 
cs_get_matrix(cs_state_t *state, int param)
{
    CS_CHECK_MAGIC;
    switch(param) {
        case CS_MATRIX_LASER_POSITION:
            return state->laser_position;
    }
    return NULL;
}

int 
cs_set_robot_position(cs_state_t *state, cs_position_t *position)
{
    CS_CHECK_MAGIC;
    if (position) {
        state->position = *position;
        return 1;
    } 
    return 0;
}

int
cs_protect(cs_state_t *state, int x1, int y1, int x2, int y2)
{
    int x, y;
    cs_map_pixel_t *ptr;
    CS_CHECK_MAGIC;

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
cs_unprotect(cs_state_t *state, int x1, int y1, int x2, int y2)
{
    int x, y;
    cs_map_pixel_t *ptr;
    CS_CHECK_MAGIC;

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

cs_map_pixel_t 
*cs_get_map(cs_state_t *state, int map_id)
{
    CS_CHECK_MAGIC;
    switch (map_id) {
        case CS_GRID_MAP:
            return state->grid_map;
        case CS_OBSTACLE_MAP:
            return state->obstacle_map;
        case CS_SHIELD_MAP:
            return state->shield_map;
        case CS_ZMAP:
            return (cs_map_pixel_t*)state->zmap;
        case 0xdead:
            return state->hole_map;
    }
    return NULL;
}

int 
cs_compress_map(cs_state_t *state, unsigned char *buffer, int buffer_size)
{
    int nb_runs = 0;
    int x, y;
    cs_dmap_run_t *runs = (cs_dmap_run_t *)buffer;
    if (buffer == NULL || buffer_size < 1000 * sizeof(cs_dmap_run_t))
        return 0;
    CS_CHECK_MAGIC;

    for (y = 0; y < CS_MAP_SIZE; y++) {
        cs_map_pixel_t *ptr = &state->obstacle_map[y * CS_MAP_SIZE];
        int code;
        // Line count run
        runs[nb_runs].code = y;
        runs[nb_runs++].length = 0;
        // Let's get first grid_map code
        if (*ptr == 0) code = 2; // No obstacle
        else if (*ptr == CDM_UNEXPLORED) code = 1; // Not explored
        else code = 0; // Obstacle
        ptr++;
        runs[nb_runs].code = code;
        runs[nb_runs].length = 1;
        for (x = 1; x < CS_MAP_SIZE; x++, ptr++) {
            // Let's get the following grid_map code
            if (*ptr == 0) code = 2; // No obstacle
            else if (*ptr == CDM_UNEXPLORED) code = 1; // Not explored
            else code = 0; // Obstacle
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

const char *
cs_get_version()
{
    return CS_VERSION;
}
