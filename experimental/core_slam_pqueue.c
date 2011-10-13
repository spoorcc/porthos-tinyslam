/* CoreSLAM
 * (c) Mines ParisTech 2010-2011 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "core_slam_internals.h"

#define CS_USE_PQUEUE
#define CS_APPLY_LOG
//#define CS_FEW_MATCHING_POINTS_PENALTY
//#undef USE_SSE3

#ifdef USE_SSE3
#include <pmmintrin.h>
#endif
#ifdef USE_NEON
#include <arm_neon.h>
#endif

static int
cs_distance_scan_to_map_part_compare(void const *a, void const *b)
{
    cs_distance_scan_to_map_part_t *ela = *(cs_distance_scan_to_map_part_t **)a;
    cs_distance_scan_to_map_part_t *elb = *(cs_distance_scan_to_map_part_t **)b;
    if (ela->score > elb->score) return 1;
    if (ela->score < elb->score) return -1;
    return 0;
}

#ifdef CS_APPLY_LOG
#define CS_MAX_DISTANCE 30000
static int cs_log[CS_MAX_DISTANCE] = {-1};
#endif

#define EXCH(a, b) tmp = pqueue[a]; pqueue[a] = pqueue[b]; pqueue[b] = tmp;

static void
cs_pqueue_fixup(cs_distance_scan_to_map_part_t **pqueue, int k)
{
    cs_distance_scan_to_map_part_t *tmp;
    while (k > 0 && pqueue[(k-1)/2]->score > pqueue[k]->score) {
	EXCH((k-1)/2, k);
	k = (k-1)/2;
    }
}

static void
cs_pqueue_fixdown(cs_distance_scan_to_map_part_t **pqueue, int k, int N)
{
    int j;
    cs_distance_scan_to_map_part_t *tmp;
    while (2*k+1 < N) {
	j = 2*k+1;
	if (j < N-1 && pqueue[j]->score > pqueue[j+1]->score) j++;
	if (!(pqueue[k]->score > pqueue[j]->score)) break;
	EXCH(k, j);
	k = j;
    }
}

static void
cs_pqueue_insert(cs_distance_scan_to_map_part_t **pqueue, cs_distance_scan_to_map_part_t *item, int *N)
{
    pqueue[*N] = item;
    cs_pqueue_fixup(pqueue, *N);
    (*N)++;
}

static cs_distance_scan_to_map_part_t *
cs_pqueue_getmin(cs_distance_scan_to_map_part_t **pqueue, int *N)
{
    cs_distance_scan_to_map_part_t *tmp;
    if (*N == 0) return NULL;
    (*N)--;
    EXCH(0, *N);
    cs_pqueue_fixdown(pqueue, 0, *N);
    return pqueue[*N];
}

#ifdef USE_SSE3
typedef union {
    struct { int32_t y; int32_t x; } pos;
    __m128i xmm;
} cs_pos_mmx_t;
#endif

int
cs_distance_scan_to_map_part(cs_state_t *state, cs_scan_t *scan, int *scan_order, cs_distance_scan_to_map_part_t *part, cs_3dpoint_t **rotated)
{
    int i, j, x, y;
#define HOW_MANY 220
#ifdef USE_SSE3
    __m128 a, b, c;
    a = _mm_set_ps(part->c, -part->s, part->s, part->c);
    c = _mm_set_ps(part->map_pos_x, part->map_pos_y, part->map_pos_x, part->map_pos_y);
#endif
#ifdef USE_NEON
    float32x4_t a, b;
    float32x2_t c;
    a = vsetq_lane_f32(part->c, a, 0);
    a = vsetq_lane_f32(-part->s, a, 1);
    a = vsetq_lane_f32(part->s, a, 2);
    a = vsetq_lane_f32(part->c, a, 3);
    c = vset_lane_f32(part->map_pos_x, c, 0); 
    c = vset_lane_f32(part->map_pos_y, c, 1);
#endif

    // Translate and rotate scan to robot position
    // and compute the distance
    if (part->already_done == scan->nb_good_points) return 1;
    i = 0;
    do {
        int value;
        if (rotated) {
            x = (int)(part->map_pos_x + rotated[part->which_theta][part->already_done].x);
            y = (int)(part->map_pos_y + rotated[part->which_theta][part->already_done].y);
            value = rotated[part->which_theta][part->already_done++].value;
        } else {
            j = scan_order[part->already_done++];
            value = scan->points[j].value;

#ifdef USE_SSE3
            {
                cs_pos_mmx_t pos;
                b = _mm_set_ps(scan->points[j].x, scan->points[j].y, scan->points[j].x, scan->points[j].y);
                b = _mm_mul_ps(a, b);
                b = _mm_hadd_ps(b, b);
                b = _mm_add_ps(b, c);
                pos.xmm = _mm_cvtps_epi32(b);
                x = pos.pos.x;
                y = pos.pos.y;
            }
#else
#ifdef USE_NEON
            int32x2_t j;
            float32x2_t d;
            d = vset_lane_f32(scan->points[j].x, d, 0); 
            d = vset_lane_f32(scan->points[j].y, d, 1);
            b = vcombine_f32(d, d);
            b = vmulq_f32(a, b);
            d = vpadd_f32(vget_low_f32(b), vget_high_f32(b));
            d = vadd_f32(c, d);
            j = vcvt_s32_f32(d);
            x = vget_lane_u32(j, 0);
            y = vget_lane_u32(j, 1);
#else 
            x = (int)(part->map_pos_x + part->c * scan->points[j].x - part->s * scan->points[j].y + 0.5f);
            y = (int)(part->map_pos_y + part->s * scan->points[j].x + part->c * scan->points[j].y + 0.5f);
            //printf("SSE3 : %d %d %d %d\n", x, y, pos.pos.x, pos.pos.y);
            //assert(x == pos.pos.x && y == pos.pos.y);
#endif // USE_NEON
#endif // USE_SSE3
        }
        i++;             
        // Check boundaries
        if (x >= 0 && x < CS_MAP_SIZE && y >= 0 && y < CS_MAP_SIZE) { 
        //if (!((x | y) & ~(CS_MAP_SIZE - 1))) { // Faster ?
            //int v = state_hole_map[y * CS_MAP_SIZE + x];
            uint32_t v = state->hole_map[(y << state->map_size_bits) + x]; // Faster?
            if (v != CDM_UNEXPLORED) {
#ifdef CS_APPLY_LOG
                if (value < CS_MAX_DISTANCE) {
                    uint32_t vlog = (v * cs_log[value]) >> 16;
                    part->v[part->nb_points] = vlog;
                    part->sum += vlog; 
                    part->nb_points++;
                }
#else
                part->sum += v; // * log(scan->value[j]); // Option to give more importance to points that are far away
                part->v[part->nb_points] = v;
                part->nb_points++;
#endif // CS_APPLY_LOG
            }
        }
        // Don't consider points outside map
    } while (i != HOW_MANY && part->already_done != scan->nb_good_points);
    
    //if (part->nb_points > 0)
    if (part->nb_points * state->param_magic_ratio[state->mode] > (part->already_done << 3))
        part->score = (int)((part->sum << 16) / part->nb_points); // Beware with the 16bits shift if we change the way the sum is computer
    else part->score = 200000000;
    assert(part->score >= 0);

    if (part->already_done == scan->nb_good_points) return 1;
    return 0;
}

typedef struct {
    cs_state_t *state;
    cs_distance_scan_to_map_part_t *pos;
    cs_scan_t *scan;
    int *scan_order;
    int start, stop;
    cs_3dpoint_t **rotated;
} cs_dstmp_multicore_context_data_t;

void 
cs_distance_scan_to_map_part_multicore(void *c)
{
    cs_dstmp_multicore_context_data_t *context = (cs_dstmp_multicore_context_data_t*)c;
    int i;
    for (i = context->start; i < context->stop; i++) {
        cs_distance_scan_to_map_part(context->state, context->scan, context->scan_order, &context->pos[i], context->rotated);
    }
}

#define SWAP(x, y) tmp = list[x]; list[x] = list[y]; list[y] = tmp;

int 
partition(int list[], int left, int right, int pivotIndex)
{
    int pivotValue = list[pivotIndex];
    int i, tmp, storeIndex = left;
    SWAP(pivotIndex, right);
    for (i = left; i < right; i++)
        if (list[i] < pivotValue) {
            SWAP(storeIndex, i);
            storeIndex++;
        }
    SWAP(right, storeIndex);
    return storeIndex;
}

int
quickfindfirstK(int list[], int left, int right, int k)
{
    do {
        int pivotIndex = left, pivotNewIndex;
        while (list[pivotIndex] == 0 && pivotIndex < right) pivotIndex++;
        pivotNewIndex = partition(list, left, right, pivotIndex);
        if (pivotNewIndex > k)
            right = pivotNewIndex - 1;
        else if (pivotNewIndex < k)  
            left = pivotNewIndex + 1;
        else break;
    } while (right > left);
    return list[k];
}

// Warning : returns an array that must be freed later on by the caller
cs_distance_scan_to_map_part_t **
cs_eval_positions(cs_state_t *state, cs_scan_t *scan, int *scan_order, cs_distance_scan_to_map_part_t *pos, int npos, int neval, cs_3dpoint_t **rotated)
{
    int i, pqueue_size = 0;
    cs_distance_scan_to_map_part_t *cpart, **pqueue = (cs_distance_scan_to_map_part_t**)malloc(sizeof(cs_distance_scan_to_map_part_t*) * npos);
        
    for (i = 0; i != npos; i++) {
        pos[i].map_pos_x = (float)(pos[i].pos.x * 1000 * CS_MAP_SCALE);
        pos[i].map_pos_y = (float)(pos[i].pos.y * 1000 * CS_MAP_SCALE);
        if (!rotated) {
            pos[i].c = (float)(cos(pos[i].pos.theta * M_PI / 180) * CS_MAP_SCALE);
            pos[i].s = (float)(sin(pos[i].pos.theta * M_PI / 180) * CS_MAP_SCALE);
        }
        pos[i].already_done = 0;
        pos[i].score = 0;
        pos[i].sum = 0;
        pos[i].nb_points = 0;
    }

    // Compute a first part of all hypotheses
#ifdef MULTICORE_SUPPORT
    /*if (state->counter == 0)*/ {
        int core;
        cs_dstmp_multicore_context_data_t context[CS_MAX_CORES];
        for (core = 0; core < state->nb_cores; core++) {
            int nb_to_compute = npos / state->nb_cores;
            context[core].state = state;
            context[core].pos = pos;
            context[core].scan = scan;
            context[core].scan_order = scan_order;
            context[core].start = core * nb_to_compute;
            context[core].stop = (core + 1) * nb_to_compute;
            context[core].rotated = rotated;
            if (context[core].stop > npos)
                context[core].stop = npos;
            cs_pool_of_threads_compute(state, core, cs_distance_scan_to_map_part_multicore, (void*)&context[core]);
        }
        cs_pool_of_threads_completion_wait(state);
    }
#else
    for (i = 0; i != npos; i++)
        cs_distance_scan_to_map_part(state, scan, scan_order, &pos[i], rotated);
#endif

    // Insert in queue those that have not been fully computed (presumably all...)
    for (i = 0; i != npos; i++)
        if (pos[i].already_done < scan->nb_good_points)
            cs_pqueue_insert(pqueue, &pos[i], &pqueue_size);
    
    for (i = 0; i < neval - npos && pqueue_size; i++) {
        // Get the best...
        cpart = pqueue[0];
        if (!cs_distance_scan_to_map_part(state, scan, scan_order, cpart, rotated)) {
            cs_pqueue_fixdown(pqueue, 0, pqueue_size);
        } else {
            cs_pqueue_getmin(pqueue, &pqueue_size);
        }
    }
   
    if (state->get_rid_of_outliers) { 
        if (state->mode != CS_MODE_RELOC) { // Because it is slow...
            // Keep only 95% of all points of a scan
            for (i = 0; i != npos; i++) {
                cs_distance_scan_to_map_part_t *p = &pos[i];
                if (p->nb_points * state->param_magic_ratio[state->mode] > (p->already_done << 3)) {
                    int c, end;
                    // Sort all computed values
                    //qsort(p->v, p->nb_points, sizeof(uint32_t), cs_distance_scan_to_map_part_vsort);
                    p->sum = 0;
                    end = p->nb_points * 95 / 100;
/*
#define QSORT_TYPE uint32_t
#define QSORT_BASE (p->v)
#define QSORT_NELT (p->nb_points)
#define QSORT_LT(a, b) (*a < *b)
#include "qsort.c"
*/
                    quickfindfirstK(p->v, 0, p->nb_points - 1, end);
                    for (c = 0; c < end; c++) 
                        p->sum += p->v[c];
                    p->score = (int)((p->sum << 16) / end); // Beware with the 16bits shift if we change the way the sum is computer
                } else p->score = 200000000;
            }
        }
    }

    for (i = 0; i != npos; i++) {
        pqueue[i] = &pos[i];
#ifdef CS_FEW_MATCHING_POINTS_PENALTY
        // New : Take into account the number of matching points so that results with few matching points get penalted
        //if (pqueue[i]->nb_points > 0) {
        if (pqueue[i]->nb_points * state->param_magic_ratio[state->mode] > (pqueue[i]->already_done << 3)) {
            //pqueue[i]->score = (((int64_t)pqueue[i]->score) << 9) / pqueue[i]->nb_points;
            int64_t score = (((int64_t)pqueue[i]->score) * pqueue[i]->already_done) / pqueue[i]->nb_points;
            if (score > 200000000) score = 200000000;
            pqueue[i]->score = (int32_t)score;
        }
        else pqueue[i]->score = 200000000; // Quite a bad score
#endif // CS_FEW_MATCHING_POINTS_PENALTY 
        assert(pqueue[i]->score >= 0);
    }
    qsort(pqueue, npos, sizeof(cs_distance_scan_to_map_part_t*), cs_distance_scan_to_map_part_compare);
    return pqueue;
}

void
cs_init_scan_order(cs_scan_t *scan, int *scan_order)
{
    int i, j, tmp;
    int nb_points = scan->nb_good_points;

    for (i = 0, j = 0; i < scan->nb_points; i++) {
        if (scan->points[i].value)
            scan_order[j++] = i;
    } 
    assert(j == scan->nb_good_points);

    // Not really random scan order...
    for (i = 0; i < nb_points / 5; i++) {
        j = i * 5;
        tmp = scan_order[i];
        scan_order[i] = scan_order[j];
        scan_order[j] = tmp;
    }
    // The remaining in random
    for (; i < nb_points - 1; i++) {
        j = rand() % (nb_points - i);
        tmp = scan_order[i];
        scan_order[i] = scan_order[i + j];
        scan_order[i + j] = tmp;
    }
#ifdef CS_APPLY_LOG
    if (cs_log[0] == -1) {
        int i;
        for (i = 0; i != CS_MAX_DISTANCE; i++) {
            cs_log[i] = (int)((log(i) * 65536) + 0.5);
        }
    }
#endif
}

double
cs_robot_model_validation(cs_position_t *pos1, cs_position_t *pos2)
{
    double xA = pos2->x - pos1->x;
    double yA = pos2->y - pos1->y;
    double normA = sqrt(xA * xA + yA * yA);
    if (normA != 0) {
        double alpha = (pos2->theta + pos1->theta) / 2;
        double alpharad = alpha * M_PI / 180;
        double gamma = atan2(yA, xA) - alpharad;
        return fabs(tan(gamma));

/*
        double calpha = cos(alpharad);
        double salpha = sin(alpharad);
        return fabs(xA * salpha + yA * calpha) / normA;
*/
    } else return 0;
    
}

#ifdef CS_USE_PQUEUE
static cs_position_t
cs_multiscale_search_recurs(cs_state_t *state, cs_scan_t *scan, cs_position_t *start_pos, cs_position_t *original_pos, double max_xy, double max_theta, int maxtests, int *bd, double *d2model, int *debug_info, int step)
{
    const int CS_MS_STEP = 7;
#define CS_MS_STEP_THETA 6
#define CS_MS_NEVALX (population_size * 2)
    cs_position_t bestpos;
    int bestscore;
    double bestd2model;
    int i, j, ix, iy, itheta;
    int debug = 0;
    cs_distance_scan_to_map_part_t *population, **pqueue;
    int scan_order[CS_SCAN_SIZE];
    cs_3dpoint_t *rotated[CS_MS_STEP_THETA];
    cs_3dpoint_t *rotated_points;
    int population_size;
    double co, so;

    //fprintf(stderr, "Testing scale %lf, %lf\n", max_xy, max_theta);
    cs_init_scan_order(scan, scan_order);

    if (maxtests < 0) {
        debug = 1;
        maxtests = -maxtests;
        //printf("Debug!\n");
    }

    population = (cs_distance_scan_to_map_part_t*)malloc(sizeof(cs_distance_scan_to_map_part_t) * CS_MS_STEP * CS_MS_STEP * CS_MS_STEP_THETA);
    rotated_points = (cs_3dpoint_t*)malloc(scan->nb_good_points * CS_MS_STEP_THETA * sizeof(cs_3dpoint_t));
    for (i = 0; i < CS_MS_STEP_THETA; i++) 
        rotated[i] = &rotated_points[i * scan->nb_good_points];

    // Get an initial population
#define ON_BORDER_X 1
#define ON_BORDER_Y 2
#define ON_BORDER_THETA 4
#define ON_BORDER_INF 8

    co = cos(original_pos->theta * M_PI / 180);
    so = sin(original_pos->theta * M_PI / 180);
          
    population_size = 0;  
    for (itheta = 0; itheta != CS_MS_STEP_THETA; itheta++) {
        float theta = start_pos->theta + (itheta - (CS_MS_STEP_THETA - 1) * 0.5) * max_theta / (CS_MS_STEP_THETA - 1);
        {
            // Pre-rotation of points
            float c = (float)(cos(theta * M_PI / 180) * CS_MAP_SCALE);
            float s = (float)(sin(theta * M_PI / 180) * CS_MAP_SCALE);
            cs_3dpoint_t *r = rotated[itheta];
            for (i = 0; i < scan->nb_good_points; i++) {
                j = scan_order[i];
                r[i].value = scan->points[j].value;
                r[i].x = c * scan->points[j].x - s * scan->points[j].y + 0.5f;
                r[i].y = s * scan->points[j].x + c * scan->points[j].y + 0.5f;
            }
        }
        for (ix = 0; ix != CS_MS_STEP; ix++) {
            double dx = (ix - (CS_MS_STEP - 1) * 0.5) * max_xy / (CS_MS_STEP - 1);
            for (iy = 0; iy != CS_MS_STEP; iy++, population_size++) {
                double dy = (iy - (CS_MS_STEP - 1) * 0.5) * max_xy / (CS_MS_STEP - 1);
                double dx2, dy2;
                // Rotate around original angle
                dx2 = co * dx - so * dy;
                dy2 = so * dx + co * dy;    
                if (CS_MS_STEP != 1) {
                    population[population_size].pos.x = start_pos->x + dx2;
                    population[population_size].pos.y = start_pos->y + dy2;
                    population[population_size].pos.theta = theta;
                }
                population[population_size].which_theta = itheta;
                population[population_size].model_error = cs_robot_model_validation(original_pos, &population[population_size].pos);
                if (state->mode != CS_MODE_RELOC && population[population_size].model_error > state->max_model_error) {
                    population_size--; 
                    continue;
                }
                population[population_size].on_border = 0;
                if (ix == 0 || ix == CS_MS_STEP - 1) population[population_size].on_border |= ON_BORDER_X;
                if (iy == 0 || iy == CS_MS_STEP - 1) population[population_size].on_border |= ON_BORDER_Y;
                if (itheta == 0 || itheta == CS_MS_STEP_THETA - 1) population[population_size].on_border |= ON_BORDER_THETA;
                if (ix == 0 || iy == 0 || itheta == 0) population[population_size].on_border |= ON_BORDER_INF;
            }
        }
    }

    // Evaluate the population
    pqueue = cs_eval_positions(state, scan, scan_order, population, population_size, CS_MS_NEVALX, rotated);
    bestpos = pqueue[0]->pos;
    bestscore = pqueue[0]->score;
    bestd2model = pqueue[0]->model_error;

    // Compute covariance matrix
    if (step == 1) {
        int x, y;
        for (y = 1; y <= 3; y++) {
            for (x = 1; x <= 3; x++) {
                state->covariance[y][x] = 0;
            }
        }

        for (i = 0; i != population_size; i++) {
            double diffscore = population[i].score - bestscore;
            double escore = exp(-diffscore * diffscore / 1e12);
            double dx = population[i].pos.x - bestpos.x;
            double dy = population[i].pos.y - bestpos.y;
            double dtheta = population[i].pos.theta - bestpos.theta;
            state->covariance[1][1] += escore * dx * dx;
            state->covariance[1][2] += escore * dx * dy;
            state->covariance[2][1] += escore * dx * dy;
            state->covariance[1][3] += escore * dx * dtheta;
            state->covariance[3][1] += escore * dx * dtheta;
            state->covariance[2][2] += escore * dy * dy;
            state->covariance[2][3] += escore * dy * dtheta;
            state->covariance[3][2] += escore * dy * dtheta;
            state->covariance[3][3] += escore * dtheta * dtheta;
        }
        for (y = 1; y <= 3; y++) {
            for (x = 1; x <= 3; x++) {
                state->covariance[y][x] /= population_size;
            }
        }
    }

    // Record algorithm data for documentation
#if 0
    {
        static FILE *handle = NULL;
        static int first = 1;
#define FRAME 10
        if (first) { // state->counter == FRAME) {
            fprintf(stderr, "CoreSLAM pqueue : saving intermediate results\n");
            if (!handle) handle = fopen("results_multiscale.dat", "wt");
            for (i = 0; i != population_size; i++) {
                fprintf(handle, "%lg %lg %lg %d %lg\n", 
                        pqueue[i]->pos.x, pqueue[i]->pos.y, pqueue[i]->pos.theta, pqueue[i]->score, pqueue[i]->model_error);
            }
            fprintf(handle,"\n\n");
            first = 0;
        } else
        //if (state->counter > FRAME) 
        {
            if (handle) {
                fclose(handle);
                handle = NULL;
            }
        }
    }
#endif

    if (debug) {
        for (i = 0; i != population_size; i++) {
            printf("%lg %lg %lg (%d - %d)\n", 
                    pqueue[i]->pos.x, pqueue[i]->pos.y, pqueue[i]->pos.theta,
                    pqueue[i]->score, pqueue[i]->already_done);
        }
        printf("------\n");
    }

    if (pqueue[0]->score > 0 && maxtests > CS_MS_NEVALX) {
        // Go on with another population
        // First, we have to check that we are on the borders of the testing zone or not
        if (pqueue[0]->on_border) {
            // Record this information
            cs_position_t pos2 = bestpos;
            if (debug_info) (*debug_info)++;
            
            // This is a solution on border. So we have to look at the same scale in the right direction
            if (pqueue[0]->on_border & ON_BORDER_X) {
                // Keep y and theta. Just translate x
                if (debug) printf("*** On border X\n");
                pos2.x += ((pqueue[0]->on_border & ON_BORDER_INF)?-1:1) * max_xy * (CS_MS_STEP / 2 - 1) / (CS_MS_STEP - 1);
            }
            if (pqueue[0]->on_border & ON_BORDER_Y) {
                // Keep x and theta. Just translate y
                if (debug) printf("*** On border Y\n");
                pos2.y += ((pqueue[0]->on_border & ON_BORDER_INF)?-1:1) * max_xy * (CS_MS_STEP / 2 - 1) / (CS_MS_STEP - 1);
            }
            if (pqueue[0]->on_border & ON_BORDER_THETA) {
                // Keep x and theta. Just translate y
                if (debug) printf("*** On border THETA\n");
                pos2.theta += ((pqueue[0]->on_border & ON_BORDER_INF)?-1:1) * max_theta * (CS_MS_STEP_THETA / 2 - 1) / (CS_MS_STEP_THETA - 1);
            }
            free(population);
            population = NULL;
            free(rotated_points);
            rotated_points = NULL;
            free(pqueue);
            pqueue = NULL;
            {
                cs_position_t new_bestpos;
                int new_bestscore;
                double new_bestd2model;
                int remaining_tests = maxtests - CS_MS_NEVALX;
                if (debug) remaining_tests = -remaining_tests;
                new_bestpos = cs_multiscale_search_recurs(state, scan, &pos2, original_pos, max_xy, max_theta, remaining_tests, &new_bestscore, &new_bestd2model, debug_info, step);
                if (new_bestscore < bestscore) {
                    bestpos = new_bestpos;
                    bestscore = new_bestscore;
                    bestd2model = new_bestd2model;
                }
            }
        } else {
            // Continue with a lower resolution
            free(population);
            population = NULL;
            free(rotated_points);
            rotated_points = NULL;
            free(pqueue);
            pqueue = NULL;
            // Stop the search if the resolution is going lower than a threshold
            if (max_xy > state->min_search_res_xy || max_theta > state->min_search_res_theta) {
                cs_position_t new_bestpos;
                int new_bestscore;
                double new_bestd2model;
                int remaining_tests = maxtests - CS_MS_NEVALX;
                if (debug) remaining_tests = -remaining_tests;
                new_bestpos = cs_multiscale_search_recurs(state, scan, &bestpos, original_pos, 2 * max_xy / (CS_MS_STEP - 1), 2.5 * max_theta / (CS_MS_STEP_THETA - 1), remaining_tests, &new_bestscore, &new_bestd2model, debug_info, step + 1);
                // Factor should be 2 for inside cube mapping. We put 2.5 to cope with the possible noise in the likelihood function
                if (new_bestscore < bestscore) {
                    bestpos = new_bestpos;
                    bestscore = new_bestscore;
                    bestd2model = new_bestd2model;
                }
            }
        }
    }
    if (population) free(population);
    if (pqueue) free(pqueue);
    if (rotated_points) free(rotated_points);
    if (bd) *bd = bestscore;
    if (d2model) *d2model = bestd2model;
    return bestpos;
}
#else
static cs_position_t
cs_multiscale_search_recurs(cs_state_t *state, cs_scan_t *scan, cs_position_t *start_pos, cs_position_t *original_pos, double max_xy, double max_theta, int maxtests, int *bd, double *d2model, int *debug_info, int step)
{
    const int CS_MS_STEP = 7;
    const int CS_MS_STEP_THETA = 6;
#define CS_MS_NEVALX (population_size * 2)
    cs_position_t bestpos;
    int bestscore;
    double bestd2model;
    int i, j, ix, iy, itheta;
    int debug = 0;
    cs_distance_scan_to_map_part_t *population;
    int scan_order[CS_SCAN_SIZE];
    cs_3dpoint_t rotated[CS_SCAN_SIZE];
    int population_size;
    double co, so;
    int best_on_border;

    //fprintf(stderr, "Testing scale %lf, %lf\n", max_xy, max_theta);
    cs_init_scan_order(scan, scan_order);

    if (maxtests < 0) {
        debug = 1;
        maxtests = -maxtests;
        //printf("Debug!\n");
    }

    population = (cs_distance_scan_to_map_part_t*)malloc(sizeof(cs_distance_scan_to_map_part_t) * CS_MS_STEP * CS_MS_STEP * CS_MS_STEP_THETA);

    // Get an initial population
#define ON_BORDER_X 1
#define ON_BORDER_Y 2
#define ON_BORDER_THETA 4
#define ON_BORDER_INF 8

    co = cos(original_pos->theta * M_PI / 180);
    so = sin(original_pos->theta * M_PI / 180);
          
    population_size = 0;  
    for (itheta = 0; itheta != CS_MS_STEP_THETA; itheta++) {
        float theta = start_pos->theta + (itheta - (CS_MS_STEP_THETA - 1) * 0.5) * max_theta / (CS_MS_STEP_THETA - 1);
        {
            // Pre-rotation of points
            float c = (float)(cos(theta * M_PI / 180) * CS_MAP_SCALE);
            float s = (float)(sin(theta * M_PI / 180) * CS_MAP_SCALE);
            cs_3dpoint_t *r = rotated;
            for (i = 0; i < scan->nb_good_points; i++) {
                j = scan_order[i];
                r[i].value = scan->points[j].value;
                r[i].x = c * scan->points[j].x - s * scan->points[j].y + 0.5f;
                r[i].y = s * scan->points[j].x + c * scan->points[j].y + 0.5f;
            }
        }
        for (ix = 0; ix != CS_MS_STEP; ix++) {
            double dx = (ix - (CS_MS_STEP - 1) * 0.5) * max_xy / (CS_MS_STEP - 1);
            for (iy = 0; iy != CS_MS_STEP; iy++, population_size++) {
                double dy = (iy - (CS_MS_STEP - 1) * 0.5) * max_xy / (CS_MS_STEP - 1);
                double dx2, dy2;
                // Rotate around original angle
                dx2 = co * dx - so * dy;
                dy2 = so * dx + co * dy;    
                if (CS_MS_STEP != 1) {
                    population[population_size].pos.x = start_pos->x + dx2;
                    population[population_size].pos.y = start_pos->y + dy2;
                    population[population_size].pos.theta = theta;
                }
                population[population_size].model_error = cs_robot_model_validation(original_pos, &population[population_size].pos);
                if (state->mode != CS_MODE_RELOC && population[population_size].model_error > state->max_model_error) {
                    population_size--; continue;
                }
                population[population_size].on_border = 0;
                if (ix == 0 || ix == CS_MS_STEP - 1) population[population_size].on_border |= ON_BORDER_X;
                if (iy == 0 || iy == CS_MS_STEP - 1) population[population_size].on_border |= ON_BORDER_Y;
                if (itheta == 0 || itheta == CS_MS_STEP_THETA - 1) population[population_size].on_border |= ON_BORDER_THETA;
                if (ix == 0 || iy == 0 || itheta == 0) population[population_size].on_border |= ON_BORDER_INF;

                // Direct evaluation
                {
                    cs_distance_scan_to_map_part_t *part = &population[population_size];
                    part->map_pos_x = (float)(part->pos.x * 1000 * CS_MAP_SCALE);
                    part->map_pos_y = (float)(part->pos.y * 1000 * CS_MAP_SCALE);
                    if (!rotated) {
                        part->c = (float)(cos(part->pos.theta * M_PI / 180) * CS_MAP_SCALE);
                        part->s = (float)(sin(part->pos.theta * M_PI / 180) * CS_MAP_SCALE);
                    }
                    part->already_done = 0;
                    part->score = 0;
                    part->sum = 0;
                    part->nb_points = 0;

                    int x, y, value;
                    for (i = 0; i < scan->nb_good_points; i++) {
                        x = (int)(part->map_pos_x + rotated[i].x);
                        y = (int)(part->map_pos_y + rotated[i].y);
                        value = rotated[i].value;
                        // Check boundaries
                        if (x >= 0 && x < CS_MAP_SIZE && y >= 0 && y < CS_MAP_SIZE) { 
                            //if (!((x | y) & ~(CS_MAP_SIZE - 1))) { // Faster ?
                            //int v = state_hole_map[y * CS_MAP_SIZE + x];
                            uint32_t v = state->hole_map[(y << state->map_size_bits) + x]; // Faster?
                            if (v != CDM_UNEXPLORED) {
#ifdef CS_APPLY_LOG
                                if (value < CS_MAX_DISTANCE) {
                                    uint32_t vlog = (v * cs_log[value]) >> 16;
                                    part->v[part->nb_points] = vlog;
                                    part->sum += vlog; 
                                    part->nb_points++;
                                }
#else
                                part->sum += v; // * log(scan->value[j]); // Option to give more importance to points that are far away
                                part->v[part->nb_points] = v;
                                part->nb_points++;
#endif // CS_APPLY_LOG
                            }
                        }
                    }

                    //pqueue[i]->score = (((int64_t)pqueue[i]->score) << 9) / pqueue[i]->nb_points;
                    if (part->nb_points * state->param_magic_ratio[state->mode] > (part->already_done << 3))
                        part->score = (int)((part->sum << 16) / part->nb_points); // Beware with the 16bits shift if we change the way the sum is computer
                    else part->score = 200000000;
                    
                    if (population_size == 0 | part->score < bestscore) {
                        bestscore = part->score;
                        bestpos = part->pos;
                        bestd2model = part->model_error;
                        best_on_border = part->on_border;
                    }
                }
            }
        }
    }

    // Compute covariance matrix
    if (step == 1) {
        int x, y;
        for (y = 1; y <= 3; y++) {
            for (x = 1; x <= 3; x++) {
                state->covariance[y][x] = 0;
            }
        }

        for (i = 0; i != population_size; i++) {
            double diffscore = population[i].score - bestscore;
            double escore = exp(-diffscore * diffscore / 1e12);
            double dx = population[i].pos.x - bestpos.x;
            double dy = population[i].pos.y - bestpos.y;
            double dtheta = population[i].pos.theta - bestpos.theta;
            state->covariance[1][1] += escore * dx * dx;
            state->covariance[1][2] += escore * dx * dy;
            state->covariance[2][1] += escore * dx * dy;
            state->covariance[1][3] += escore * dx * dtheta;
            state->covariance[3][1] += escore * dx * dtheta;
            state->covariance[2][2] += escore * dy * dy;
            state->covariance[2][3] += escore * dy * dtheta;
            state->covariance[3][2] += escore * dy * dtheta;
            state->covariance[3][3] += escore * dtheta * dtheta;
        }
        for (y = 1; y <= 3; y++) {
            for (x = 1; x <= 3; x++) {
                state->covariance[y][x] /= population_size;
            }
        }
    }

    if (bestscore > 0 && maxtests > CS_MS_NEVALX) {
        // Go on with another population
        // First, we have to check that we are on the borders of the testing zone or not
        if (best_on_border) {
            // Record this information
            cs_position_t pos2 = bestpos;
            if (debug_info) (*debug_info)++;
            
            // This is a solution on border. So we have to look at the same scale in the right direction
            if (best_on_border & ON_BORDER_X) {
                // Keep y and theta. Just translate x
                if (debug) printf("*** On border X\n");
                pos2.x += ((best_on_border & ON_BORDER_INF)?-1:1) * max_xy * (CS_MS_STEP / 2 - 1) / (CS_MS_STEP - 1);
            }
            if (best_on_border & ON_BORDER_Y) {
                // Keep x and theta. Just translate y
                if (debug) printf("*** On border Y\n");
                pos2.y += ((best_on_border & ON_BORDER_INF)?-1:1) * max_xy * (CS_MS_STEP / 2 - 1) / (CS_MS_STEP - 1);
            }
            if (best_on_border & ON_BORDER_THETA) {
                // Keep x and theta. Just translate y
                if (debug) printf("*** On border THETA\n");
                pos2.theta += ((best_on_border & ON_BORDER_INF)?-1:1) * max_theta * (CS_MS_STEP_THETA / 2 - 1) / (CS_MS_STEP_THETA - 1);
            }
            free(population);
            population = NULL;
            {
                cs_position_t new_bestpos;
                int new_bestscore;
                double new_bestd2model;
                int remaining_tests = maxtests - CS_MS_NEVALX;
                if (debug) remaining_tests = -remaining_tests;
                new_bestpos = cs_multiscale_search_recurs(state, scan, &pos2, original_pos, max_xy, max_theta, remaining_tests, &new_bestscore, &new_bestd2model, debug_info, step);
                if (new_bestscore < bestscore) {
                    bestpos = new_bestpos;
                    bestscore = new_bestscore;
                    bestd2model = new_bestd2model;
                }
            }
        } else {
            // Continue with a lower resolution
            free(population);
            population = NULL;
            // Stop the search if the resolution is going lower than a threshold
            if (max_xy > state->min_search_res_xy || max_theta > state->min_search_res_theta) {
                cs_position_t new_bestpos;
                int new_bestscore;
                double new_bestd2model;
                int remaining_tests = maxtests - CS_MS_NEVALX;
                if (debug) remaining_tests = -remaining_tests;
                new_bestpos = cs_multiscale_search_recurs(state, scan, &bestpos, original_pos, 2 * max_xy / (CS_MS_STEP - 1), 2.5 * max_theta / (CS_MS_STEP_THETA - 1), remaining_tests, &new_bestscore, &new_bestd2model, debug_info, step + 1);
                // Factor should be 2 for inside cube mapping. We put 2.5 to cope with the possible noise in the likelihood function
                if (new_bestscore < bestscore) {
                    bestpos = new_bestpos;
                    bestscore = new_bestscore;
                    bestd2model = new_bestd2model;
                }
            }
        }
    }
    if (population) free(population);
    if (bd) *bd = bestscore;
    if (d2model) *d2model = bestd2model;
    return bestpos;
}
#endif

cs_position_t
cs_multiscale_search(cs_state_t *state, cs_scan_t *scan, cs_position_t *start_pos)
{
    return cs_multiscale_search_recurs(state, scan, start_pos, start_pos, state->sigma_xy, state->sigma_theta, state->neval, &state->score, &state->model_error, &state->debug_info[CS_DEBUG_INFO_ON_BORDERS], 0);
}

void
cs_test_around_position(cs_state_t *state, cs_scan_t *scan, cs_position_t *start_pos, double max_xy, double max_theta)
{
    const int CS_MS_POPULATION_SIZE = 100;
    const int CS_MS_NEVAL = CS_MS_POPULATION_SIZE * 10;
    cs_distance_scan_to_map_part_t *population, **pqueue;
    FILE *handle;
    char filename[256];
    int i;
    int scan_order[CS_SCAN_SIZE];

    cs_init_scan_order(scan, scan_order);
    population = (cs_distance_scan_to_map_part_t*)malloc(sizeof(cs_distance_scan_to_map_part_t) * CS_MS_POPULATION_SIZE);

    // Get an initial population
    for (i = 0; i != CS_MS_POPULATION_SIZE; i++) {
        population[i].pos = *start_pos;
        population[i].pos.x += (i - (CS_MS_POPULATION_SIZE - 1) * 0.5) * max_xy / (CS_MS_POPULATION_SIZE - 1);
    }

    // Evaluate the population
    pqueue = cs_eval_positions(state, scan, scan_order, population, CS_MS_POPULATION_SIZE, CS_MS_NEVAL, NULL);
    
    // Record results
    sprintf(filename, "core_slam_debug_x_%d.txt", state->counter);
    handle = fopen(filename, "wt");
    for (i = 0; i != CS_MS_POPULATION_SIZE; i++) {
        fprintf(handle, "%lg %i\n", population[i].pos.x, population[i].score);
    }
    fclose(handle);
    free(pqueue);

    // Get an initial population
    for (i = 0; i != CS_MS_POPULATION_SIZE; i++) {
        population[i].pos = *start_pos;
        population[i].pos.y += (i - (CS_MS_POPULATION_SIZE - 1) * 0.5) * max_xy / (CS_MS_POPULATION_SIZE - 1);
    }

    // Evaluate the population
    pqueue = cs_eval_positions(state, scan, scan_order, population, CS_MS_POPULATION_SIZE, CS_MS_NEVAL, NULL);
    
    // Record results
    sprintf(filename, "core_slam_debug_y_%d.txt", state->counter);
    handle = fopen(filename, "wt");
    for (i = 0; i != CS_MS_POPULATION_SIZE; i++) {
        fprintf(handle, "%lg %i\n", population[i].pos.y, population[i].score);
    }
    fclose(handle);
    free(pqueue);

    // Get an initial population
    for (i = 0; i != CS_MS_POPULATION_SIZE; i++) {
        population[i].pos = *start_pos;
        population[i].pos.theta += (i - (CS_MS_POPULATION_SIZE - 1) * 0.5) * max_theta / (CS_MS_POPULATION_SIZE - 1);
    }

    // Evaluate the population
    pqueue = cs_eval_positions(state, scan, scan_order, population, CS_MS_POPULATION_SIZE, CS_MS_NEVAL, NULL);
    
    // Record results
    sprintf(filename, "core_slam_debug_theta_%d.txt", state->counter);
    handle = fopen(filename, "wt");
    for (i = 0; i != CS_MS_POPULATION_SIZE; i++) {
        fprintf(handle, "%lg %i\n", population[i].pos.theta, population[i].score);
    }
    fclose(handle);
    free(pqueue);

    free(population);
}


