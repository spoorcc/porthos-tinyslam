/* CoreSLAM
 * (c) Mines ParisTech 2010-2011 */

#include <stdlib.h>
#include <stdio.h>
#include "core_slam_internals.h"

int 
cs_load_map_ex(cs_state_t *state, cs_map_pixel_t *map, int sx, int sy)
{
    int x, y;
    for (y = 0; y < sy; y++) {
        for (x = 0; x < sx; x++) {
            int val = map[x + y * sx];
            if (val != CDM_UNEXPLORED)
                if (val == 0) 
                    cs_map_set(state, x, y, -1);
                else
                    cs_map_set(state, x, y, val);
        }
    }
    
    state->mode = CS_MODE_RELOC;
    return 1;
}

/* Get the next non-comment line from the PBM file f into the buffer b.
 */
static void 
cs_pbm_getln(FILE *f, char *b)
{
    int i;
    char c;
    int dummy;
    
    // Read the next significant line (non-comment) from f into buffer b
    do {
	// Read the next line
	i = 0;
	do {
	    dummy = fscanf (f, "%c", &c);
	    b[i++] = c;
	    if (c == '\n') b[i] = '\0';
	} while (c != '\n');
    } while (b[0]=='\n' || b[0] == '#');
}

static void 
cs_get_num_pbm(FILE *f, char *b, int *bi, int *res)
{
    int i;
    char str[80];
    
    while (b[*bi]==' ' || b[*bi]=='\t' || b[*bi]=='\n' || b[*bi]=='\r') {
	if (b[*bi] == '\n' || b[*bi]=='\r') {
	    cs_pbm_getln (f, b);
	    *bi = 0;
	} else
	    *bi += 1;
    }
    
    i = 0;
    while (b[*bi]>='0' && b[*bi]<='9')
	str[i++] = b[(*bi)++];
    str[i] = '\0';
    sscanf(str, "%d", res);
}

int 
cs_load_map(cs_state_t *state, const char *filename, cs_progress_notification_cb_t progress_cb, void *cb_context)
{
    int i, j, k, n, m, bi, b, dummy;
    unsigned char ucval;
    int val;
    FILE *f;
    char buf1[256];
    
    f = fopen(filename, "rb");
    if (f == NULL) {
        fprintf(stderr, "Can't open specified file %s\n", filename);
	return 0;
    }
    
    cs_pbm_getln(f, buf1);
    if (buf1[0] == 'P') {
	switch (buf1[1]) {
	case '1':       k=1; break;
	case '2':       k=2; break;
	case '3':       k=3; break;
	case '4':       k=4; break;
	case '5':       k=5; break;
	case '6':       k=6; break;
	default:        k=0; fclose(f); 
                        fprintf(stderr, "%s is not a PBM/PGM/PPM file.\n", filename); return 0;
	}
    } else k=0;
    bi = 2;
    
    cs_get_num_pbm(f,buf1,&bi,&m);         // Number of columns
    cs_get_num_pbm(f,buf1,&bi,&n);         // Number of rows
    if (k!=1 && k!=4) cs_get_num_pbm(f,buf1,&bi,&b); // Max value
    else b=1;
    
    // Allocate the image
    if (k == 3 || k == 6) {       // Colour
        fprintf(stderr, "Not able to load color map\n");
        return 0;
    } else  {
        if (b > 255) {
            fprintf(stderr, "Not able to load 16-bits map\n");
            return 0;
        } else {
            int px = 0;

            for (i = 0; i < n; i++) {
                // Notify
                if (i >= n * px / 100) {
                    if (progress_cb) 
                        progress_cb(px, cb_context);
                    px++;
                }

                for (j = 0; j < m; j++) {
                    if (k < 3) {
                        dummy = fscanf(f, "%d", &val);
                    } else {
                        dummy = fscanf(f, "%c", &ucval);
                        val = ucval;
                    }
                    if (val != 128)
                        if (val < CS_MAP_LOADING_OBSTACLE_THRESHOLD)
                            cs_map_set(state, j, n - i - 1, -1); // -1 means not transient obstacle
                        else 
                            cs_map_set(state, j, n - i - 1, val);
                }
            }
        }
    }
    fclose(f);

    state->mode = CS_MODE_RELOC;
    return 1;
}

int
cs_save_map(cs_state_t *state, char *filename, int width, int height, int options)
{
    int x, y, xp, yp;
    FILE *output;
    unsigned char *map;
   
    if (options & CS_OBSTACLE_MAP) {
        map = state->obstacle_map;
    } else map = state->grid_map;

    output = fopen(filename, "wt");
    fprintf(output, "P2\n%d %d 255\n", width, height);
    y = (CS_MAP_SIZE - height) / 2;
    for (yp = 0; yp < height; y++, yp++) {
        x = (CS_MAP_SIZE - width) / 2; 
	for (xp = 0; xp < width; x++, xp++) {
            fprintf(output, "%d ", (int)(map[ (CS_MAP_SIZE - 1 - y) * CS_MAP_SIZE + x]));
	}
	fprintf(output, "\n");
    }
    fclose(output);
    return 1;
}

#ifdef MULTICORE_SUPPORT
#include <unistd.h>

typedef struct {
    cs_state_t *state;
    int thread_nb;
} cs_pool_of_threads_data_t;

int
cs_get_nb_cores()
{
    return sysconf(_SC_NPROCESSORS_ONLN);
}

void *
cs_pool_of_threads_process(void *d)
{
    cs_pool_of_threads_data_t *data = (cs_pool_of_threads_data_t*)d;
    cs_state_t *state = data->state;
    int thread_nb = data->thread_nb;
    free(d);

    do {
        pthread_mutex_lock(&state->mutex[thread_nb]);
        //fprintf(stderr, "POT : Thread %d waiting on completion\n", thread_nb);
        pthread_cond_wait(&state->thread_do[thread_nb], &state->mutex[thread_nb]);
        //fprintf(stderr, "POT : Starting code execution for thread %d\n", thread_nb);
        // Execute the thread code
        if (state->thread_fct[thread_nb]) {
            state->thread_fct[thread_nb](state->thread_context[thread_nb]);
            state->thread_fct[thread_nb] = NULL;
        }
        //fprintf(stderr, "POT : Finished code execution for thread %d\n", thread_nb);
        pthread_cond_signal(&state->thread_done[thread_nb]);
        pthread_mutex_unlock(&state->mutex[thread_nb]);
    } while (!state->destroy_pool_of_threads);
    
    // Get out of the thread
    pthread_cond_destroy(&state->thread_do[thread_nb]);
    pthread_cond_destroy(&state->thread_done[thread_nb]);
    pthread_mutex_destroy(&state->mutex[thread_nb]);
    pthread_exit(NULL);
}

int
cs_create_pool_of_threads(cs_state_t *state)
{
    int i;
    state->nb_cores = cs_get_nb_cores();
    if (state->nb_cores > 2) state->nb_cores /= 2; // To avoid hyperthreading
    state->destroy_pool_of_threads = 0;
    for (i = 0; i < state->nb_cores; i++) {
        cs_pool_of_threads_data_t *data = (cs_pool_of_threads_data_t*)malloc(sizeof(cs_pool_of_threads_data_t));
        pthread_mutex_init(&state->mutex[i], NULL);
        pthread_cond_init(&state->thread_do[i], NULL);
        pthread_cond_init(&state->thread_done[i], NULL);
        state->thread_fct[i] = NULL;
        state->thread_context[i] = NULL;
        data->state = state;
        data->thread_nb = i;
        pthread_create(&state->thread[i], NULL, cs_pool_of_threads_process, (void*)data);
    }
    return state->nb_cores;
}

int
cs_destroy_pool_of_threads(cs_state_t *state)
{
    int i;
    void *status;
    state->destroy_pool_of_threads = 1;
    for (i = 0; i < state->nb_cores; i++) {
        pthread_mutex_lock(&state->mutex[i]);
        state->thread_fct[i] = NULL;
        state->thread_context[i] = NULL;
        pthread_cond_signal(&state->thread_do[i]);
        pthread_mutex_unlock(&state->mutex[i]);
        pthread_join(state->thread[i], &status);
    }
    return 1;
}

int
cs_pool_of_threads_compute(cs_state_t *state, int thread, cs_pool_of_threads_fct_t fct, void *context)
{
    if (thread < 0 || thread >= state->nb_cores)
        return 0;
    pthread_mutex_lock(&state->mutex[thread]);
    state->thread_fct[thread] = fct;
    state->thread_context[thread] = context;
    pthread_cond_signal(&state->thread_do[thread]);
    //fprintf(stderr, "POT : Signaled thread %d\n", thread);
    pthread_mutex_unlock(&state->mutex[thread]);
    return 1;
}

int 
cs_pool_of_threads_completion_wait(cs_state_t *state)
{
    int i;
    for (i = 0; i < state->nb_cores; i++) {
        //fprintf(stderr, "POT : Waiting for completion of thread %d\n", i);
        pthread_mutex_lock(&state->mutex[i]);
        if (state->thread_fct[i]) // If the first thread has not acquired the mutex and executed its code
            // Then wait for completion of the first thread
            pthread_cond_wait(&state->thread_done[i], &state->mutex[i]);
        pthread_mutex_unlock(&state->mutex[i]);
    }
    return 1;
}
#endif // MULTICORE_SUPPORT

