#include <stdio.h>

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

int
main()
{
    int i, v[100];
    for (i = 0; i < 100; i++) v[i] = 100 - i;
    printf("%d\n", quickfindfirstK(v, 0, 99, 95));
    for (i = 0; i < 95; i++)
        printf("%d ", v[i]);
    printf("\n");
    return 1;
}
