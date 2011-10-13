/* CoreSLAM
 * (c) Mines ParisTech 2010-2011 */

#ifdef WIN32
#define CS_SHARED __declspec(dllexport)
#endif // WIN32

#ifdef MULTICORE_SUPPORT
#include <pthread.h>
#endif

#include "core_slam.h"

#define CS_VERSION "v0.9 (Babylon)"

// Hidden parameters
#define CS_PARAM_LATENCY                        100
#define CS_PARAM_SIGMA_XY                       101
#define CS_PARAM_SIGMA_THETA                    102
#define CS_PARAM_NEVAL                          103
#define CS_PARAM_MINRES_XY                      104
#define CS_PARAM_MINRES_THETA                   105
#define CS_PARAM_GET_RID_OF_OUTLIERS            106
#define CS_PARAM_TRANSIENT_OBSTACLES_DELAY      107
#define CS_PARAM_TRANSIENT_OBSTACLES_THRESHOLD  108
#define CS_PARAM_MAX_MODEL_ERROR                109
#define CS_PARAM_MAGIC_RATIO_MAPPING            110
#define CS_PARAM_MAGIC_RATIO_RELOC              111

#define CS_DEBUG_INFO_ON_BORDERS  1

typedef struct {
    int scan_size;              // number of elements per scan
    int scan_angle;             // span of scan in degrees
    int scan_duration;          // duration of scan in ms
    int detection_margin;       // first scan element to consider
    int distance_no_detection;  // default value when the laser returns 0
    int max_distance;           // value over which data are invalid
    int minimum_distance;
    int filter;
    int upside_down;
    int angle_start;
    int angle_end;
} cs_laser_parameters_t;

typedef struct {
    unsigned int timestamp;
    double v, psidot;           // Used to correct the scans according to the speed of the robot
    cs_position_t position; 
    double pitch, roll, altitude;
    int d[CS_SCAN_SIZE];
    int score;
    int mapping;                // To be integrated in map or not
} cs_sensor_data_t;

// Fast priority queue based likelihood evaluation
typedef struct {
    cs_position_t pos;
    float map_pos_x, map_pos_y;
    float c, s;
    int which_theta;
    int already_done;
    int64_t sum;
    int nb_points;
    int score;
    int on_border;
    double model_error; // likelihood wrt Robot model
    uint32_t v[CS_SCAN_SIZE];
} cs_distance_scan_to_map_part_t;

// Client/server diff data structure
typedef struct {
    int plusminus;
    int x, y, z;
} cs_diff_point_t;

typedef struct {
    int x1, y1, x2, y2;
    int z;
} cs_zmap_diff_zone_t;

typedef struct {
    cs_send_to_client_fct send_fct;
    void *context;
} cs_client_t;

typedef struct {
    int timestamp;
    int map_index;
} cs_transient_obstacle_data_t;

#define CS_MAGIC            0xc03e51a3
#define CS_CHECK_MAGIC      if (state == NULL || state->magic != CS_MAGIC) { \
    fprintf(stderr, "CoreSLAM : Bad magic. State is corrupted\n"); \
    return 0; \
}

#define CS_MAX_LATENCY      10000 // 20 seconds at 10Hz
#define CS_FILTER_SIZE      2
#define CS_MAX_DEBUG_INFO   10
#define CS_MAX_DIFF_DATA    1024
#define CS_MAX_CLIENTS      16
#define CS_MAX_CORES        16

typedef void (*cs_pool_of_threads_fct_t)(void *);

struct cs_state {
    uint32_t magic;             // magic number
    
    cs_map_pixel_t *grid_map;        
    cs_map_pixel_t *hole_map;
    cs_map_pixel_t *obstacle_map; // This is a map where 0 are non obstacles, 255 are not explored and values in between are obstacles persistence counters
    cs_map_pixel_t *shield_map;
    int16_t        *zmap;       // Height / Altitude / Z map
    struct cs_state *owner;     // real owner of the map
    int update_map;             // update the map or not ?
    int mode;                   // CS_MAPPING, CS_RELOC
    
    cs_scan_t *scan;
    cs_laser_parameters_t laser_parameters;
    
    cs_sensor_data_t latent_sensor_data[CS_MAX_LATENCY + CS_FILTER_SIZE];
    cs_position_t position;     // robot position
    int score;                  // current matching score
    double model_error;         // current likelihood wrt robot model
    int ok;                     // is this OK ?
    cs_matrix_t covariance;
    int map_origin_x;           // map origin x (in mm)
    int map_origin_y;           // map origin y (in mm)
    int map_origin_z;           // map origin z (in mm)
    int counter;                // number of iterations of slam
    double speed;               // filtered robot speed
    int delayed_avg_score;      // average score, with latency
    int current_avg_score;      // average score 
    int previous_failed;        // did the previous match failed ?
    int reloc_counter;
    int stationnarity_counter;  // Did the robot move ?
    cs_position_t stationnary_position; 
    
    // The queue for distance map computation
    int *queue;
    int queue_size;
    int queue_start, queue_end;

    // The queue for transient obstacles removal
    cs_transient_obstacle_data_t *transient_obstacles_queue;
    int transient_obstacles_queue_size;
    int transient_obstacles_queue_start, transient_obstacles_queue_end;

    // Parameters :
    int map_size_bits;
    double sigma_xy;
    double sigma_theta;
    int latency;
    int positive_dynamics, negative_dynamics;
    int resxy, resz;            // mm per pixel
    int nb_layers;
    int neval;
    int switch_to_reloc_threshold;
    int switch_to_mapping_threshold;
    int auto_switch_to_mapping;
    double min_search_res_xy, min_search_res_theta;
    int get_rid_of_outliers;
    int transient_obstacles_delay;
    int transient_obstacles_threshold;
    int dmap_max_dto;
    double max_model_error;
    int param_ground_rejection;
    int param_magic_ratio[2];
    double param_stationnarity_posxy;
    double param_stationnarity_theta;
    int param_stationnarity_max_counter; 
    
    // Geometric parameters
    cs_matrix_t laser_position;
    double altitude, pitch, roll;
    int update_altitude_wrt_attitude;

    // Client/server data
    int diff_size;
    cs_diff_point_t diff_points[CS_MAX_DIFF_DATA];
    int zmap_diff_size;
    cs_zmap_diff_zone_t zmap_diff_zone[CS_MAX_DIFF_DATA];
    cs_client_t clients[CS_MAX_CLIENTS];
    int nb_clients;

    // Debug info :
    int debug_info[CS_MAX_DEBUG_INFO];

#ifdef MULTICORE_SUPPORT
    int nb_cores;
    pthread_t thread[CS_MAX_CORES];
    pthread_mutex_t mutex[CS_MAX_CORES];
    pthread_cond_t thread_do[CS_MAX_CORES];
    pthread_cond_t thread_done[CS_MAX_CORES];
    cs_pool_of_threads_fct_t thread_fct[CS_MAX_CORES];
    void *thread_context[CS_MAX_CORES];
    int destroy_pool_of_threads;
#endif
};

void cs_init_scan_order(cs_scan_t *scan, int *scan_order);
void cs_build_scan(cs_state_t *state, cs_sensor_data_t *sd, cs_scan_t *scan);
cs_sensor_data_t *cs_update_robot_position(cs_state_t *state, int *laser_data, int data_size, cs_position_t *robot_position, int options);
cs_position_t cs_multiscale_search(cs_state_t *state, cs_scan_t *scan, cs_position_t *start_pos);
void cs_send_diff_data(cs_state_t *state);
void cs_send_zmap_diff_data(cs_state_t *state);
void cs_send_position_and_attitude(cs_state_t *state);

int dmap_set_obstacle(cs_state_t *state, cs_map_pixel_t *map,  int x, int y);
void dmap_set_noobstacle(cs_state_t *state, cs_map_pixel_t *map, int x, int y);
int dmap_set_explored(cs_state_t *state, cs_map_pixel_t *map, int x, int y, int *discovered);

int cs_SDL_drawing_thread(void *s);
int cs_load_map_ex(cs_state_t *s, cs_map_pixel_t *map, int sx, int sy);
int cs_map_update(cs_state_t *state, cs_scan_t *scan, cs_position_t *position);
int cs_map_update_new_points(cs_state_t *state, cs_scan_t *scan, cs_position_t *position);

int cs_localize(cs_state_t *state, int options);
void cs_test_around_position(cs_state_t *state, cs_scan_t *scan, cs_position_t *start_pos, double max_xy, double max_theta);
cs_distance_scan_to_map_part_t **cs_eval_positions(cs_state_t *state, cs_scan_t *scan, int *scan_order, cs_distance_scan_to_map_part_t *pos, int npos, int neval, cs_3dpoint_t **rotated);

// Pool of threads management
int cs_get_nb_cores();
int cs_create_pool_of_threads(cs_state_t *state);
int cs_destroy_pool_of_threads(cs_state_t *state);
int cs_pool_of_threads_compute(cs_state_t *state, int thread, cs_pool_of_threads_fct_t fct, void *context);
int cs_pool_of_threads_completion_wait(cs_state_t *state);

// Constants
#define CS_MAP_LOADING_OBSTACLE_THRESHOLD 20

// Big powerful macros
#define CS_MAP_SIZE (1 << state->map_size_bits)
#define CS_MAP_SIZE_BITS (state->map_size_bits)
#define CS_MAP_SCALE (1.0 / state->resxy)

typedef struct {
    uint16_t code;
    uint16_t length;
} cs_dmap_run_t;
