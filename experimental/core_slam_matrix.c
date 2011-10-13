#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "core_slam_internals.h"

// Arithmetic routines
cs_vector_t 
cs_allocate_vector(int nh)
{
    cs_vector_t v;
    v=(cs_vector_t )malloc(nh * sizeof(cs_float_t));
    if (!v) {
        fprintf(stderr, "Memory allocation failure");
	return NULL;
    }
    return v - 1;
}

int *
cs_allocate_ivect(int nh)
{
    int *v;
    v=(int *)malloc(nh * sizeof(int));
    if (!v) {
        fprintf(stderr, "Memory allocation failure");
	return NULL;
    }
    return v - 1;
}

cs_matrix_t 
cs_allocate_matrix(int nrh, int nch)
{
    int i;
    cs_matrix_t m;

    m = (cs_matrix_t )malloc(nrh * (sizeof(cs_float_t*) + nch * sizeof(cs_float_t)));
    if (!m) {
        fprintf(stderr, "Memory allocation failure");
	return NULL;
    }
    m--;

    for(i = 1; i <= nrh; i++) {
	m[i] = (cs_vector_t )(((char*)(m + 1)) + nrh * sizeof(cs_float_t*) + (i - 1) * nch * sizeof(cs_float_t));
	m[i]--;
    }
    return m;
}

int 
cs_free_vector(cs_vector_t v, int nh)
{
    free((void*)(v + 1));
    return 1;
}

int 
cs_free_ivect(int *v, int nh)
{
    free((void*)(v + 1));
    return 1;
}

int 
cs_free_matrix(cs_matrix_t m, int nrh, int nch)
{
    free((void*)(m + 1));
    return 1;
}

int 
cs_matrix_empty(cs_matrix_t m, int nl, int nc)
{
    int x, y;
    for (y = 1; y <= nl; y++) {
        for (x = 1; x <= nc; x++) {
            m[y][x] = 0;
        }
    }
    return 1;
}

int 
cs_matrix_id(cs_matrix_t m, int nlc)
{
    int x, y;
    for (y = 1; y <= nlc; y++) {
        for (x = 1; x <= nlc; x++) {
            m[y][x] = (y == x)? 1 : 0;
        }
    }
    return 1;
}

int 
cs_matrix_copy(cs_matrix_t A, int nl, int nc, cs_matrix_t B)
{
    int x, y;
    for (y = 1; y <= nl; y++) {
        for (x = 1; x <= nc; x++) {
            B[y][x] = A[y][x];
        }
    }
    return 1;
}

int 
cs_vector_copy(cs_vector_t A, int n, cs_vector_t B)
{
    int x;
    for (x = 1; x <= n; x++) B[x] = A[x];
    return 1;
}

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}

// Linear equation solution by Gauss-Jordan elimination, equation (2.1.1) above. a[1..n][1..n]
// is the input matrix. b[1..n][1..m] is input containing the m right-hand side vectors. On
// output, a is replaced by its matrix inverse, and b is replaced by the corresponding set of solution
// vectors.
static int
cs_gaussj(cs_matrix_t a, int n, cs_matrix_t b, int m)
{
    int *indxc,*indxr,*ipiv;
    int i,icol,irow,j,k,l,ll;
    cs_float_t big,dum,pivinv,temp;

    indxc = cs_allocate_ivect(n); // The integer arrays ipiv, indxr, and indxc are used for 
    indxr = cs_allocate_ivect(n); // bookkeeping on the pivoting.
    ipiv = cs_allocate_ivect(n);
    
    for (j=1;j<=n;j++) ipiv[j]=0;
    for (i=1;i<=n;i++) { //This is the main loop over the columns to be
        big=0.0; // reduced.
        for (j=1;j<=n;j++) // This is the outer loop of the search for a pivot
            if (ipiv[j] != 1) // element.
                for (k=1;k<=n;k++) {
                    if (ipiv[k] == 0) {
                        if (fabs(a[j][k]) >= big) {
                            big=fabs(a[j][k]);
                            irow=j;
                            icol=k;
                        }
                    } else if (ipiv[k] > 1) {
                        fprintf(stderr, "cs_gaussj: Singular Matrix-1");
                        return 0;
                    }
                }
        ++(ipiv[icol]);
        // We now have the pivot element, so we interchange rows, if needed, to put the pivot
        // element on the diagonal. The columns are not physically interchanged, only relabeled:
        // indxc[i], the column of the ith pivot element, is the ith column that is reduced, while
        // indxr[i] is the row in which that pivot element was originally located. If indxr[i] 6=
        // indxc[i] there is an implied column interchange. With this form of bookkeeping, the
        // solution b's will end up in the correct order, and the inverse matrix will be scrambled
        // by columns.
        if (irow != icol) {
            for (l=1;l<=n;l++) SWAP(a[irow][l],a[icol][l])
            if (b) for (l=1;l<=m;l++) SWAP(b[irow][l],b[icol][l])
        }
        indxr[i]=irow; // We are now ready to divide the pivot row by the
        indxc[i]=icol; // pivot element, located at irow and icol.
        if (a[icol][icol] == 0.0) {
            fprintf(stderr, "cs_gaussj: Singular Matrix-2");
            return 0;
        }
        pivinv=1.0/a[icol][icol];
        a[icol][icol]=1.0;
        for (l=1;l<=n;l++) a[icol][l] *= pivinv;
        if (b) for (l=1;l<=m;l++) b[icol][l] *= pivinv;
        for (ll=1;ll<=n;ll++) // Next, we reduce the rows...
            if (ll != icol) { // ...except for the pivot one, of course.
                dum=a[ll][icol];
                a[ll][icol]=0.0;
                for (l=1;l<=n;l++) a[ll][l] -= a[icol][l]*dum;
                if (b) for (l=1;l<=m;l++) b[ll][l] -= b[icol][l]*dum;
            }
    }
    // This is the end of the main loop over columns of the reduction. It only remains to unscramble
    // the solution in view of the column interchanges. We do this by interchanging pairs of
    // columns in the reverse order that the permutation was built up.
    for (l=n;l>=1;l--) {
        if (indxr[l] != indxc[l])
            for (k=1;k<=n;k++)
                SWAP(a[k][indxr[l]],a[k][indxc[l]]);
    } // And we are done.
    
    cs_free_ivect(ipiv, n);
    cs_free_ivect(indxr, n);
    cs_free_ivect(indxc, n);
    return 1;
}

// C = A * B
int
cs_matrix_multiply(cs_matrix_t A, cs_matrix_t B, int nlA, int ncA, int ncB, cs_matrix_t C)
{
    int x, y, z;
    for (y = 1; y <= nlA; y++) {
        for (z = 1; z <= ncB; z++) {
            C[y][z] = A[y][1] * B[1][z];
            for (x = 2; x <= ncA; x++) {
                C[y][z] += A[y][x] * B[x][z];
            }
        }
    }   
    return 1;
}

int
cs_matrix_vect_multiply(cs_matrix_t A, cs_vector_t B, int nlA, int ncA, cs_vector_t C)
{
    int x, y;
    for (y = 1; y <= nlA; y++) {
        C[y] = A[y][1] * B[1];
        for (x = 2; x <= ncA; x++) {
            C[y] += A[y][x] * B[x];
        }
    }   
    return 1;
}

int
cs_matrix_add(cs_matrix_t A, cs_matrix_t B, int nl, int nc, cs_matrix_t C)
{
    int x, y;
    for (y = 1; y <= nl; y++) {
        for (x = 1; x <= nc; x++) {
            C[y][x] = A[y][x] + B[y][x];
        }
    }    
    return 1;
}

int
cs_vector_add(cs_vector_t A, cs_vector_t B, int nl, cs_vector_t C)
{
    int y;
    for (y = 1; y <= nl; y++) {
        C[y] = A[y] + B[y];
    }    
    return 1;
}

int
cs_matrix_sub(cs_matrix_t A, cs_matrix_t B, int nl, int nc, cs_matrix_t C)
{
    int x, y;
    for (y = 1; y <= nl; y++) {
        for (x = 1; x <= nc; x++) {
            C[y][x] = A[y][x] - B[y][x];
        }
    }    
    return 1;
}

int
cs_vector_sub(cs_vector_t A, cs_vector_t B, int nl, cs_vector_t C)
{
    int y;
    for (y = 1; y <= nl; y++) {
        C[y] = A[y] - B[y];
    }    
    return 1;
}

int 
cs_matrix_transpose(cs_matrix_t A, int nl, int nc, cs_matrix_t B)
{
    int x, y;
    for (y = 1; y <= nl; y++) {
        for (x = 1; x <= nc; x++) {
            B[x][y] = A[y][x];
        }
    }    
    return 1;
}
    
int
cs_matrix_invert(cs_matrix_t A, int nlc, cs_matrix_t Ainv)
{
    int x, y;
    // Copy the matrix
    for (y = 1; y <= nlc; y++) {
        for (x = 1; x <= nlc; x++) {
            Ainv[y][x] = A[y][x];
        }
    }    
    return cs_gaussj(Ainv, nlc, NULL, 0);
}

#define CS_ALLOC_MATRIX(name, nrh, nch) \
    cs_matrix_t name = cs_allocate_matrix(nrh, nch)
#define CS_ALLOC_VECTOR(name, nrh) \
    cs_vector_t name = cs_allocate_vector(nrh)
#define CS_FREE_MATRIX(name, nrh, nch) \
    cs_free_matrix(name, nrh, nch)
#define CS_FREE_VECTOR(name, nrh) \
    cs_free_vector(name, nrh)

int 
cs_init_kalman_filter(cs_kalman_filter_data_t *data, int n, int m, int r)
{
    int i;
    data->n = n;
    data->m = m;
    data->r = r;
    data->A = cs_allocate_matrix(n, n);
    data->B = cs_allocate_matrix(n, m);
    data->C = cs_allocate_matrix(r, n);
    data->x = cs_allocate_vector(n);
    data->xhat = cs_allocate_vector(n);
    data->y = cs_allocate_vector(r);
    data->yp = cs_allocate_vector(r);
    data->u = cs_allocate_vector(m);
    for (i = 1; i <= m; i++) 
        data->u[i] = 0;
    data->Sz = cs_allocate_matrix(r, r);
    data->Sw = cs_allocate_matrix(n, n);
    data->Sg = cs_allocate_matrix(m, m);
    data->P = cs_allocate_matrix(n, n);
    return 1;
}

int 
cs_done_kalman_filter(cs_kalman_filter_data_t *data)
{
    int m, n, r;
    m = data->m;
    n = data->n;
    r = data->r;
    cs_free_matrix(data->A, n, n);
    cs_free_matrix(data->B, n, m);
    cs_free_matrix(data->C, r, n);
    cs_free_vector(data->x, n);
    cs_free_vector(data->xhat, n);
    cs_free_vector(data->y, r);
    cs_free_vector(data->yp, r);
    cs_free_vector(data->u, m);
    cs_free_matrix(data->Sz, r, r);
    cs_free_matrix(data->Sw, n, n);
    cs_free_matrix(data->Sg, m, m);
    cs_free_matrix(data->P, n, n);
    return 1;
}

int
cs_kalman_filter_predict(cs_kalman_filter_data_t *data)
{
    CS_ALLOC_VECTOR(Ax, data->n); // This is the vector A*xhat
    CS_ALLOC_VECTOR(Bu, data->n); // This is the vector B*u

    CS_ALLOC_MATRIX(AP, data->n, data->n); // This is the matrix A*P
    CS_ALLOC_MATRIX(AT, data->n, data->n); // This is the matrix AT
    CS_ALLOC_MATRIX(APAT, data->n, data->n); // This is the matrix A*P*AT
    CS_ALLOC_MATRIX(APATSw, data->n, data->n); // This is the matrix A*P*AT+Sw

    CS_ALLOC_MATRIX(BSg, data->n, data->m); // This is the matrix B*Sg
    CS_ALLOC_MATRIX(BT, data->m, data->n); // This is the matrix BT
    CS_ALLOC_MATRIX(BSgBT, data->n, data->n); // This is the matrix B*Sg*BT

    // Linear state prediction (can be overriden by EKF non linear equations)
    cs_matrix_vect_multiply(data->A, data->x, data->n, data->n, Ax);
    cs_matrix_vect_multiply(data->B, data->u, data->n, data->m, Bu);
    cs_vector_add(Ax, Bu, data->n, data->xhat);

    // Covariance matrix P prediction
    cs_matrix_multiply(data->A, data->P, data->n, data->n, data->n, AP);
    cs_matrix_transpose(data->A, data->n, data->n, AT);
    cs_matrix_multiply(AP, AT, data->n, data->n, data->n, APAT);
    cs_matrix_add(APAT, data->Sw, data->n, data->n, APATSw);

    cs_matrix_multiply(data->B, data->Sg, data->n, data->m, data->n, BSg);
    cs_matrix_transpose(data->B, data->n, data->m, BT);
    cs_matrix_multiply(BSg, BT, data->n, data->m, data->n, BSgBT);
    cs_matrix_add(APATSw, BSgBT, data->n, data->n, data->P);
    
    // Observation prediction (linear. Can be overriden by EKF)
    cs_matrix_vect_multiply(data->C, data->xhat, data->r, data->n, data->yp);

    CS_FREE_VECTOR(Ax, data->n); // This is the vector A*xhat
    CS_FREE_VECTOR(Bu, data->n); // This is the vector B*u

    CS_FREE_MATRIX(AP, data->n, data->n); // This is the matrix A*P
    CS_FREE_MATRIX(AT, data->n, data->n); // This is the matrix AT
    CS_FREE_MATRIX(APAT, data->n, data->n); // This is the matrix A*P*AT
    CS_FREE_MATRIX(APATSw, data->n, data->n); // This is the matrix A*P*AT+Sw

    CS_FREE_MATRIX(BSg, data->n, data->m); // This is the matrix B*Sg
    CS_FREE_MATRIX(BT, data->m, data->n); // This is the matrix BT
    CS_FREE_MATRIX(BSgBT, data->n, data->n); // This is the matrix B*Sg*BT
    return 1;
}

int
cs_kalman_filter_update(cs_kalman_filter_data_t *data)
{
    CS_ALLOC_MATRIX(CT, data->n, data->r); // This is the matrix CT
    CS_ALLOC_MATRIX(CP, data->r, data->n); // This is the matrix C*P
    CS_ALLOC_MATRIX(CPCT, data->r, data->r); // This is the matrix C*P*CT
    CS_ALLOC_MATRIX(CPCTSz, data->r, data->r); // This is the matrix C*P*CT+Sz
    CS_ALLOC_MATRIX(CPCTSzInv, data->r, data->r); // This is the matrix (C*P*CT+Sz)-1
    CS_ALLOC_MATRIX(PCT, data->n, data->r); // This is the matrix P*CT
    CS_ALLOC_MATRIX(K, data->n, data->r); // This is the Kalman gain.
    CS_ALLOC_VECTOR(yyp, data->r); // This is the vector y - yp
    CS_ALLOC_VECTOR(Kyyp, data->n); // This is the vector K * (y - yp)
    CS_ALLOC_MATRIX(I, data->n, data->n); // This is identity matrix
    CS_ALLOC_MATRIX(KC, data->n, data->n); // This is the K * C matrix

    cs_matrix_transpose(data->C, data->r, data->n, CT);
    cs_matrix_multiply(data->C, data->P, data->r, data->n, data->n, CP);
    cs_matrix_multiply(CP, CT, data->r, data->n, data->r, CPCT);
    cs_matrix_add(CPCT, data->Sz, data->r, data->r, CPCTSz);
    cs_matrix_invert(CPCTSz, data->r, CPCTSzInv);

    cs_matrix_multiply(data->P, CT, data->n, data->n, data->r, PCT);
    cs_matrix_multiply(PCT, CPCTSzInv, data->n, data->r, data->r, K);

    cs_vector_sub(data->y, data->yp, data->r, yyp);
    cs_matrix_vect_multiply(K, yyp, data->n, data->r, Kyyp);
    cs_vector_add(data->xhat, Kyyp, data->n, data->xhat);
    cs_vector_copy(data->xhat, data->n, data->x);
    
    // Covariance matrix P update
    cs_matrix_id(I, data->n);
    cs_matrix_multiply(K, data->C, data->n, data->r, data->n, KC);
    cs_matrix_sub(I, KC, data->n, data->n, KC);
    cs_matrix_multiply(KC, data->P, data->n, data->n, data->n, data->P);
    
    CS_FREE_MATRIX(CT, data->n, data->r); // This is the matrix CT
    CS_FREE_MATRIX(CP, data->r, data->n); // This is the matrix C*P
    CS_FREE_MATRIX(CPCT, data->r, data->r); // This is the matrix C*P*CT
    CS_FREE_MATRIX(CPCTSz, data->r, data->r); // This is the matrix C*P*CT+Sz
    CS_FREE_MATRIX(CPCTSzInv, data->r, data->r); // This is the matrix (C*P*CT+Sz)-1
    CS_FREE_MATRIX(PCT, data->n, data->r); // This is the matrix P*CT
    CS_FREE_MATRIX(K, data->n, data->r); // This is the Kalman gain.
    CS_FREE_VECTOR(yyp, data->r); // This is the vector y - yp
    CS_FREE_VECTOR(Kyyp, data->n); // This is the vector K * (y - yp)
    CS_FREE_MATRIX(I, data->n, data->n); // This is identity matrix
    CS_FREE_MATRIX(KC, data->n, data->n); // This is the K * C matrix
    return 1;
}

/* Code snippet from Dan Simon (incorrect)
int
cs_kalman_filter(cs_kalman_filter_data_t *data) 
{
    CS_ALLOC_MATRIX(AP, data->n, data->n); // This is the matrix A*P
    CS_ALLOC_MATRIX(CT, data->n, data->r); // This is the matrix CT
    CS_ALLOC_MATRIX(APCT, data->n, data->r); // This is the matrix A*P*CT
    CS_ALLOC_MATRIX(CP, data->r, data->n); // This is the matrix C*P
    CS_ALLOC_MATRIX(CPCT, data->r, data->r); // This is the matrix C*P*CT
    CS_ALLOC_MATRIX(CPCTSz, data->r, data->r); // This is the matrix C*P*CT+Sz
    CS_ALLOC_MATRIX(CPCTSzInv, data->r, data->r); // This is the matrix (C*P*CT+Sz)-1
    CS_ALLOC_MATRIX(K, data->n, data->r); // This is the Kalman gain.
    CS_ALLOC_VECTOR(Cxhat, data->r); // This is the vector C*xhat
    CS_ALLOC_VECTOR(yCxhat, data->r); // This is the vector y-C*xhat
    CS_ALLOC_VECTOR(KyCxhat, data->n); // This is the vector K*(y-C*xhat)
    CS_ALLOC_VECTOR(Axhat, data->n); // This is the vector A*xhat
    CS_ALLOC_VECTOR(Bu, data->n); // This is the vector B*u
    CS_ALLOC_VECTOR(AxhatBu, data->n); // This is the vector A*xhat+B*u
    CS_ALLOC_MATRIX(AT, data->n, data->n); // This is the matrix AT
    CS_ALLOC_MATRIX(APAT, data->n, data->n); // This is the matrix A*P*AT
    CS_ALLOC_MATRIX(APATSw, data->n, data->n); // This is the matrix A*P*AT+Sw
    CS_ALLOC_MATRIX(CPAT, data->r, data->n); // This is the matrix C*P*AT
    CS_ALLOC_MATRIX(SzInv, data->r, data->r); // This is the matrix Sz-1
    CS_ALLOC_MATRIX(APCTSzInv, data->n, data->r); // This is the matrix A*P*CT*Sz-1
    CS_ALLOC_MATRIX(APCTSzInvCPAT, data->n, data->n); // This is the matrix A*P*CT*Sz-1*C*P*AT

    // The following sequence of function calls computes the K matrix.
    cs_matrix_multiply(data->A, data->P, data->n, data->n, data->n, AP);
    cs_matrix_transpose(data->C, data->r, data->n, CT);
    cs_matrix_multiply(AP, CT, data->n, data->n, data->r, APCT);
    cs_matrix_multiply(data->C, data->P, data->r, data->n, data->n, CP);
    cs_matrix_multiply(CP, CT, data->r, data->n, data->r, CPCT);
    cs_matrix_add(CPCT, data->Sz, data->r, data->r, CPCTSz);
    cs_matrix_invert(CPCTSz, data->r, CPCTSzInv);
    cs_matrix_multiply(APCT, CPCTSzInv, data->n, data->r, data->r, K);

    // The following sequence of function calls updates the xhat vector.
    cs_matrix_vect_multiply(data->C, data->xhat, data->r, data->n, Cxhat);
    cs_vector_sub(data->y, Cxhat, data->r, yCxhat);
    cs_matrix_vect_multiply(K, yCxhat, data->n, data->r, KyCxhat);
    cs_matrix_vect_multiply(data->A, data->xhat, data->n, data->n, Axhat);
    cs_matrix_vect_multiply(data->B, data->u, data->n, data->m, Bu);
    cs_vector_add(Axhat, Bu, data->n, AxhatBu);
    cs_vector_add(AxhatBu, KyCxhat, data->n, data->xhat);

    // The following sequence of function calls updates the P matrix.
    cs_matrix_transpose(data->A, data->n, data->n, AT);
    cs_matrix_multiply(AP, AT, data->n, data->n, data->n, APAT);
    cs_matrix_add(APAT, data->Sw, data->n, data->n, APATSw);
    cs_matrix_transpose(APCT, data->n, data->r, CPAT);
    cs_matrix_invert(data->Sz, data->r, SzInv);
    cs_matrix_multiply(APCT, SzInv, data->n, data->r, data->r, APCTSzInv);
    cs_matrix_multiply(APCTSzInv, CPAT, data->n, data->r, data->n, APCTSzInvCPAT);
    cs_matrix_sub(APATSw, APCTSzInvCPAT, data->n, data->n, data->P);
    
    CS_FREE_MATRIX(AP, data->n, data->n); // This is the matrix A*P
    CS_FREE_MATRIX(CT, data->n, data->r); // This is the matrix CT
    CS_FREE_MATRIX(APCT, data->n, data->r); // This is the matrix A*P*CT
    CS_FREE_MATRIX(CP, data->r, data->n); // This is the matrix C*P
    CS_FREE_MATRIX(CPCT, data->r, data->r); // This is the matrix C*P*CT
    CS_FREE_MATRIX(CPCTSz, data->r, data->r); // This is the matrix C*P*CT+Sz
    CS_FREE_MATRIX(CPCTSzInv, data->r, data->r); // This is the matrix (C*P*CT+Sz)-1
    CS_FREE_MATRIX(K, data->n, data->r); // This is the Kalman gain.
    CS_FREE_VECTOR(Cxhat, data->r); // This is the vector C*xhat
    CS_FREE_VECTOR(yCxhat, data->r); // This is the vector y-C*xhat
    CS_FREE_VECTOR(KyCxhat, data->n); // This is the vector K*(y-C*xhat)
    CS_FREE_VECTOR(Axhat, data->n); // This is the vector A*xhat
    CS_FREE_VECTOR(Bu, data->n); // This is the vector B*u
    CS_FREE_VECTOR(AxhatBu, data->n); // This is the vector A*xhat+B*u
    CS_FREE_MATRIX(AT, data->n, data->n); // This is the matrix AT
    CS_FREE_MATRIX(APAT, data->n, data->n); // This is the matrix A*P*AT
    CS_FREE_MATRIX(APATSw, data->n, data->n); // This is the matrix A*P*AT+Sw
    CS_FREE_MATRIX(CPAT, data->r, data->n); // This is the matrix C*P*AT
    CS_FREE_MATRIX(SzInv, data->r, data->r); // This is the matrix Sz-1
    CS_FREE_MATRIX(APCTSzInv, data->n, data->r); // This is the matrix A*P*CT*Sz-1
    CS_FREE_MATRIX(APCTSzInvCPAT, data->n, data->n); // This is the matrix A*P*CT*Sz-1*C*P*AT
    return 1;
}
*/

int 
cs_draw_covariance_matrix_RGBA(unsigned char *bitmap, int bsx, int bsy, cs_matrix_t covariance, double cx, double cy, double scale, unsigned int color)
{
    double sxy = covariance[1][2];
    double sxx = covariance[1][1];
    double syy = covariance[2][2];
    double lambda1, lambda2, delta, phi;
    double theta;
    if (sxx == syy) 
        theta = M_PI / 4;
    else
        theta = 0.5 * atan(2 * sxy / (sxx - syy));
    if (syy > sxx) theta += M_PI / 2;
    delta = sqrt((sxx - syy) * (sxx - syy) + 4 * sxy * sxy);
    lambda1 = scale * sqrt(0.5 * ((sxx + syy) + delta));
    lambda2 = scale * sqrt(0.5 * ((sxx + syy) - delta));

    for (phi = 0; phi < 2 * M_PI; phi += 2 * M_PI / 360) {
        int xpp, ypp;
        double x = lambda1 * cos(phi);
        double y = lambda2 * sin(phi);
        double xp = x * cos(theta) - y * sin(theta);
        double yp = x * sin(theta) + y * cos(theta);
        xpp = (int)(xp + cx + 0.5);
        ypp = (int)(yp + cy + 0.5);
        if (xpp >= 0 && xpp < bsx && ypp >= 0 && ypp < bsy) {
            unsigned char *ptr = bitmap + ((bsy - 1 - ypp) * bsx + xpp) * 4;
            *(unsigned int*)ptr = color;
        }
    }
    return 1;
}

int
cs_init_robot_EKF(cs_robot_EKF_data_t *ekf, cs_matrix_t initial_cov, double b, double sigma_odometry, cs_matrix_t system_noise)
{
    ekf->b = b;
    cs_init_kalman_filter(&ekf->kalman_filter, 3, 2, 3);
    cs_matrix_copy(initial_cov, 3, 3, ekf->kalman_filter.P);
    cs_matrix_copy(system_noise, 3, 3, ekf->kalman_filter.Sw);
    cs_matrix_id(ekf->kalman_filter.C, 3);
    ekf->kalman_filter.x[1] = 0;
    ekf->kalman_filter.x[2] = 0;
    ekf->kalman_filter.x[3] = 0;
    ekf->kalman_filter.Sg[1][1] = sigma_odometry;
    ekf->kalman_filter.Sg[1][2] = 0;
    ekf->kalman_filter.Sg[2][1] = 0; 
    ekf->kalman_filter.Sg[2][2] = sigma_odometry;
    return 1;    
}

int 
cs_robot_EKF_predict(cs_robot_EKF_data_t *ekf, double dl, double dr)
{
    double dDk = (dl + dr) * 0.5;
    double dPk = (dr - dl) / ekf->b;
    cs_matrix_t A = ekf->kalman_filter.A;
    cs_matrix_t B = ekf->kalman_filter.B;
    cs_vector_t xhat = ekf->kalman_filter.xhat;
    cs_vector_t x = ekf->kalman_filter.x;
    cs_vector_t yp = ekf->kalman_filter.yp;
    double Pk = x[3] * M_PI / 180;

    A[1][1] = 1; A[1][2] = 0; A[1][3] = - dDk * sin(Pk + dPk * 0.5);
    A[2][1] = 0; A[2][2] = 1; A[2][3] =   dDk * cos(Pk + dPk * 0.5);
    A[3][1] = 0; A[3][2] = 0; A[3][3] = 1;

    B[1][1] = 0.5 * cos(Pk + dPk * 0.5) + dDk / (2 * ekf->b) * sin(Pk + dPk / 2);
    B[1][2] = 0.5 * cos(Pk + dPk * 0.5) - dDk / (2 * ekf->b) * sin(Pk + dPk / 2);
    B[2][1] = 0.5 * sin(Pk + dPk * 0.5) - dDk / (2 * ekf->b) * cos(Pk + dPk / 2);
    B[2][2] = 0.5 * sin(Pk + dPk * 0.5) + dDk / (2 * ekf->b) * cos(Pk + dPk / 2);
    B[3][1] = -1 / ekf->b; B[3][2] = 1 / ekf->b;

    cs_kalman_filter_predict(&ekf->kalman_filter);

    // Subsisture linear prediction by our non-linear prediction
    xhat[1] = x[1] + dDk * cos(Pk + dPk / 2);
    xhat[2] = x[2] + dDk * sin(Pk + dPk / 2);
    xhat[3] = x[3] + dPk * 180 / M_PI;
    cs_vector_copy(xhat, 3, yp);

    return 1;
}

int 
cs_robot_EKF_update(cs_robot_EKF_data_t *ekf, cs_position_t *position, cs_matrix_t covariance)
{
    cs_vector_t y = ekf->kalman_filter.y;
    cs_vector_t yp = ekf->kalman_filter.yp;
    double diff;

    y[1] = position->x;
    y[2] = position->y,
    y[3] = position->theta;
    // Make sure the difference regarding theta will be OK i.e. between -PI and + PI (adapt the computed yp to the observed y)
    diff = y[3] - yp[3];
    if (diff > M_PI) y[3] -= 2 * M_PI;
    if (diff < -M_PI) yp[3] -= 2 * M_PI;

    cs_matrix_copy(covariance, 3, 3, ekf->kalman_filter.Sz);
    cs_kalman_filter_update(&ekf->kalman_filter);
    return 1;
}

int 
cs_done_robot_EKF(cs_robot_EKF_data_t *ekf)
{
    cs_done_kalman_filter(&ekf->kalman_filter);
    return 1;
}

