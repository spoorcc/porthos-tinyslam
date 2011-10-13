/* CoreSLAM
 * (c) Mines ParisTech 2010-2011 */

#ifndef _CORE_SLAM_H_
#define _CORE_SLAM_H_

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CS_SCAN_SIZE    2048

#ifdef _MSC_VER
   typedef __int64 int64_t;	// Define it from MSVC's internal type
   typedef unsigned __int64 uint64_t;
   typedef __int64 int64_t;
   typedef unsigned int uint32_t;
   typedef int int32_t;
   typedef unsigned short uint16_t;
   typedef short int16_t;
#else
   #include <stdint.h>		// Use the C99 official header
#endif

typedef double cs_float_t;
typedef cs_float_t **cs_matrix_t;
typedef cs_float_t *cs_vector_t;

typedef struct {
    double x, y;    // in meters
    double theta;   // in degrees
} cs_position_t;

typedef struct {
    float x, y, z;
    int value;
} cs_3dpoint_t;

typedef struct {
    cs_3dpoint_t points[CS_SCAN_SIZE];
    int nb_points;
    int nb_good_points;
} cs_scan_t;

typedef void (*cs_send_to_client_fct)(const unsigned char *data, int size_of_data, void *context);

#define CS_LASER_HOKUYO_URG04LX 1
#define CS_HOKUYO_URG04LX       1
#define CS_LASER_HOKUYO_UTM30LX 2
#define CS_HOKUYO_UTM30LX       2
#define CS_SICK_LMS100          3
#define CS_SICK_LMS111          4
#define CS_SICK_LMS200          5
#define CS_MICROSOFT_KINECT     6
#define CS_NEATO_XV11			7

#define CDM_UNEXPLORED 255

#define CS_NO_MAP_UPDATE        1
#define CS_SENSOR_UPSIDE_DOWN   2       
#define CS_WHEEL_ROBOT          4

#define CS_PITCH    0
#define CS_ROLL     1
#define CS_ALTITUDE 2

typedef struct cs_state cs_state_t;
typedef unsigned char cs_map_pixel_t;

CS_SHARED const char* cs_get_version();

#define CS_LASER_OFFSET(x, z) ((x) | ((z) << 16))
CS_SHARED cs_state_t *cs_init(int laser_type, int laser_offset, int options);
CS_SHARED int cs_register_client(cs_state_t *state, cs_send_to_client_fct send_to_client, void *context, char config_buffer[256], int *client_config_data_size);
CS_SHARED int cs_compress_map(cs_state_t *state, unsigned char *buffer, int buffer_size);
CS_SHARED int cs_unregister_client(cs_state_t *state, int client);
CS_SHARED cs_state_t *cs_init_3d(int laser_type, int laser_offset, int resxy, int resz, int nb_layers, int options);
CS_SHARED int cs_done(cs_state_t *state);
CS_SHARED int cs_is_state(void *state);

CS_SHARED int cs_set_attitude(cs_state_t *state, double pitch, double roll);
#define CS_ROBOT_NOT_MOVING     1
CS_SHARED int cs_step(cs_state_t *state, int *laser_data, int data_size, cs_position_t *robot_position, int options);

CS_SHARED cs_position_t cs_get_robot_position(cs_state_t *state);
CS_SHARED double cs_get_attitude(cs_state_t *state, int which);
CS_SHARED int cs_get_score(cs_state_t *state);
CS_SHARED double cs_get_model_error(cs_state_t *state);
CS_SHARED int cs_is_ok(cs_state_t *state);
CS_SHARED cs_matrix_t cs_get_covariance(cs_state_t *state);
CS_SHARED cs_scan_t *cs_get_scan(cs_state_t *state);
CS_SHARED int cs_robot_position_to_map_coordinates(cs_state_t *cs_state, cs_position_t *position, int *x, int *y);
CS_SHARED int cs_map_coordinates_to_robot_position(cs_state_t *cs_state, int x, int y, cs_position_t *position);

CS_SHARED int cs_protect(cs_state_t *state, int x1, int y1, int x2, int y2);
CS_SHARED int cs_unprotect(cs_state_t *state, int x1, int y1, int x2, int y2);
CS_SHARED int cs_zmap_set(cs_state_t *state, int x1, int y1, int x2, int y2, int z);

CS_SHARED int cs_get_debug_info(cs_state_t *state, int debug_info);
CS_SHARED int cs_set_debug_info(cs_state_t *state, int debug_info, int value);

#define CS_PARAM_MAP_SIZE                       0
#define CS_PARAM_MAP_RESOLUTION                 1
#define CS_PARAM_Z_RESOLUTION                   2
#define CS_PARAM_MAP_ORIGIN_X                   3
#define CS_PARAM_MAP_ORIGIN_Y                   4
#define CS_PARAM_UPDATE_MAP                     5
#define CS_PARAM_SWITCH_TO_MAPPING_THRESHOLD    6
#define CS_PARAM_SWITCH_TO_RELOC_THRESHOLD      7
#define CS_PARAM_MINIMUM_DISTANCE               8
#define CS_PARAM_ANGLE_START                    9
#define CS_PARAM_ANGLE_END                      10
#define CS_PARAM_GROUND_REJECTION               11
#define CS_PARAM_AUTO_SWITCH_TO_MAPPING         12
CS_SHARED int cs_get_param(cs_state_t *state, int param);
CS_SHARED int cs_set_param(cs_state_t *state, int param, int value);

#define CS_MODE_MAPPING     0
#define CS_MODE_RELOC       1
CS_SHARED int cs_set_mode(cs_state_t *state, int param);
CS_SHARED int cs_get_mode(cs_state_t *state);

#define CS_MATRIX_LASER_POSITION                0
CS_SHARED cs_matrix_t cs_get_matrix(cs_state_t *state, int param);

CS_SHARED int cs_set_robot_position(cs_state_t *state, cs_position_t *position);

#define CS_NORMAL_MAP   0
#define CS_GRID_MAP     0
#define CS_DISTANCE_MAP 1
#define CS_DEBUG_MAP    2
#define CS_OBSTACLE_MAP 3
#define CS_SHIELD_MAP   4
#define CS_ZMAP         5
CS_SHARED cs_map_pixel_t *cs_get_map(cs_state_t *state, int map_id);

typedef void (*cs_progress_notification_cb_t)(int percentage, void *context);
/** Load a map from file.
 *  Currently supports PGM file loading.
 */
CS_SHARED int cs_load_map(cs_state_t *s, const char *filename, cs_progress_notification_cb_t progress_cb, void *cb_context);
CS_SHARED int cs_map_set(cs_state_t *state, int x, int y, int val);
CS_SHARED int cs_save_map(cs_state_t *state, char *filename, int width, int height, int options);

// Kalman filter related structures and functions

// The linear system has n states, m inputs, and r outputs. 
// A is an n by n matrix
// B is an n by m matrix (m being the measurment vector size)
// C is an r by n matrix
// xhat is an n by 1 vector
// y is an r by 1 vector
// yp is an r by 1 vector
// u is an m by 1 vector
// Sz is an r by r matrix (measurement noise)
// Sw is an n by n matrix (process noise)
// Sg in an m by m matrix (measurement noise)
// P is an n by n matrix
typedef struct {
    int n, m, r;
    cs_matrix_t A, B, C;
    cs_vector_t x, xhat, y, yp, u; // y = observation, yp = predicted observation
    cs_matrix_t Sz, Sw, Sg;
    cs_matrix_t P;
} cs_kalman_filter_data_t;

typedef struct {
    double b;                   // Width of robot (distance between two wheels)
    cs_kalman_filter_data_t kalman_filter;
} cs_robot_EKF_data_t;

CS_SHARED cs_vector_t cs_allocate_vector(int nh);
CS_SHARED int cs_free_vector(cs_vector_t v, int nh);

CS_SHARED int *cs_allocate_ivect(int nh);
CS_SHARED int cs_free_ivect(int *v, int nh);

CS_SHARED cs_matrix_t cs_allocate_matrix(int nrh, int nch);
CS_SHARED int cs_free_matrix(cs_matrix_t m, int nrh, int nch);

CS_SHARED int cs_matrix_empty(cs_matrix_t m, int nl, int nc);
CS_SHARED int cs_matrix_id(cs_matrix_t m, int nlc);
CS_SHARED int cs_vector_copy(cs_vector_t A, int n, cs_vector_t B);
CS_SHARED int cs_matrix_copy(cs_matrix_t A, int nl, int nc, cs_matrix_t B);

CS_SHARED int cs_matrix_multiply(cs_matrix_t A, cs_matrix_t B, int nlA, int ncA, int ncB, cs_matrix_t C);
CS_SHARED int cs_matrix_vect_multiply(cs_matrix_t A, cs_vector_t B, int nlA, int ncA, cs_vector_t C);
CS_SHARED int cs_matrix_add(cs_matrix_t A, cs_matrix_t B, int nl, int nc, cs_matrix_t C);
CS_SHARED int cs_vector_add(cs_vector_t A, cs_vector_t B, int nl, cs_vector_t C);
CS_SHARED int cs_matrix_sub(cs_matrix_t A, cs_matrix_t B, int nl, int nc, cs_matrix_t C);
CS_SHARED int cs_vector_sub(cs_vector_t A, cs_vector_t B, int nl, cs_vector_t C);
CS_SHARED int cs_matrix_transpose(cs_matrix_t A, int nl, int nc, cs_matrix_t B);
CS_SHARED int cs_matrix_invert(cs_matrix_t A, int nlc, cs_matrix_t Ainv);

CS_SHARED int cs_init_kalman_filter(cs_kalman_filter_data_t *data, int n, int m, int r);
CS_SHARED int cs_kalman_filter_predict(cs_kalman_filter_data_t *data);
CS_SHARED int cs_kalman_filter_update(cs_kalman_filter_data_t *data);
CS_SHARED int cs_done_kalman_filter(cs_kalman_filter_data_t *data);

CS_SHARED int cs_draw_covariance_matrix_RGBA(unsigned char *bitmap, int bsx, int bsy, cs_matrix_t covariance, double cx, double cy, double scale, unsigned int color);

CS_SHARED int cs_init_robot_EKF(cs_robot_EKF_data_t *ekf, cs_matrix_t initial_cov, double b, double sigma_odometry, cs_matrix_t system_noise);
CS_SHARED int cs_robot_EKF_predict(cs_robot_EKF_data_t *ekf, double dl, double dr); 
CS_SHARED int cs_robot_EKF_update(cs_robot_EKF_data_t *ekf, cs_position_t *position, cs_matrix_t covariance); 
CS_SHARED int cs_done_robot_EKF(cs_robot_EKF_data_t *ekf);

#ifdef __CPLUSPLUS
}
#endif

#endif // _CORE_SLAM_H_
