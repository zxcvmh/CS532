/*
 * CS532 Autonomous JetBot — C-Accelerated A* Core Engine
 * Standard embedded robotics implementation for Jetson Nano ARM64.
 * Zero dynamic heap allocations during search.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define MAX_CELLS 100000
#define SQRT2 1.41421356f

typedef struct {
    float f;
    int idx;
} HeapNode;

typedef struct {
    HeapNode* data;
    int size;
    int capacity;
} MinHeap;

static inline void heap_push(MinHeap* h, float f, int idx) {
    int i = h->size++;
    while (i > 0) {
        int p = (i - 1) >> 1;
        if (h->data[p].f <= f) break;
        h->data[i] = h->data[p];
        i = p;
    }
    h->data[i].f = f;
    h->data[i].idx = idx;
}

static inline HeapNode heap_pop(MinHeap* h) {
    HeapNode ret = h->data[0];
    HeapNode x = h->data[--h->size];
    int i = 0;
    while ((i << 1) + 1 < h->size) {
        int left = (i << 1) + 1;
        int right = left + 1;
        int best = left;
        if (right < h->size && h->data[right].f < h->data[left].f) {
            best = right;
        }
        if (x.f <= h->data[best].f) break;
        h->data[i] = h->data[best];
        i = best;
    }
    h->data[i] = x;
    return ret;
}

#ifdef _WIN32
__declspec(dllexport)
#endif
int astar_c_search(
    const uint8_t* costmap,
    int width,
    int height,
    int start_idx,
    int goal_idx,
    float h_weight,
    int* out_path,
    int max_path_len
) {
    int total_cells = width * height;
    if (total_cells > MAX_CELLS) return -1;
    if (start_idx < 0 || start_idx >= total_cells || goal_idx < 0 || goal_idx >= total_cells) return -1;

    static float g_cost[MAX_CELLS];
    static int parent[MAX_CELLS];
    static uint8_t closed[MAX_CELLS];
    static HeapNode heap_buf[MAX_CELLS * 4];

    for (int i = 0; i < total_cells; ++i) {
        g_cost[i] = 1e9f;
        parent[i] = -1;
        closed[i] = 0;
    }

    MinHeap heap;
    heap.data = heap_buf;
    heap.size = 0;
    heap.capacity = MAX_CELLS * 4;

    int goal_c = goal_idx % width;
    int goal_r = goal_idx / width;
    int start_c = start_idx % width;
    int start_r = start_idx / width;

    int dx0 = abs(start_c - goal_c);
    int dy0 = abs(start_r - goal_r);
    float h0 = (dx0 < dy0) ? (dy0 + 0.41421356f * dx0) : (dx0 + 0.41421356f * dy0);

    g_cost[start_idx] = 0.0f;
    heap_push(&heap, h0 * h_weight, start_idx);

    int card_dc[4] = {1, -1, 0, 0};
    int card_dr[4] = {0, 0, 1, -1};
    int card_didx[4] = {1, -1, width, -width};

    int diag_dc[4] = {1, 1, -1, -1};
    int diag_dr[4] = {1, -1, 1, -1};
    int diag_didx[4] = {width + 1, -width + 1, width - 1, -width - 1};
    int diag_o1[4] = {1, 1, -1, -1};
    int diag_o2[4] = {width, -width, width, -width};

    int found = 0;
    while (heap.size > 0) {
        HeapNode top = heap_pop(&heap);
        int curr = top.idx;
        if (closed[curr]) continue;
        closed[curr] = 1;

        if (curr == goal_idx) {
            found = 1;
            break;
        }

        int curr_c = curr % width;
        int curr_r = curr / width;
        float curr_g = g_cost[curr];

        // 1. Cardinal
        for (int k = 0; k < 4; ++k) {
            int nc = curr_c + card_dc[k];
            int nr = curr_r + card_dr[k];
            if (nc < 0 || nc >= width || nr < 0 || nr >= height) continue;
            int n_idx = curr + card_didx[k];
            if (closed[n_idx] || costmap[n_idx] >= 200) continue;

            float tg = curr_g + 1.0f;
            if (tg < g_cost[n_idx]) {
                g_cost[n_idx] = tg;
                parent[n_idx] = curr;
                int dx = abs(nc - goal_c);
                int dy = abs(nr - goal_r);
                float h = (dx < dy) ? (dy + 0.41421356f * dx) : (dx + 0.41421356f * dy);
                heap_push(&heap, tg + h * h_weight, n_idx);
            }
        }

        // 2. Diagonal
        for (int k = 0; k < 4; ++k) {
            int nc = curr_c + diag_dc[k];
            int nr = curr_r + diag_dr[k];
            if (nc < 0 || nc >= width || nr < 0 || nr >= height) continue;
            int n_idx = curr + diag_didx[k];
            if (closed[n_idx] || costmap[n_idx] >= 200) continue;

            if (costmap[curr + diag_o1[k]] >= 200 || costmap[curr + diag_o2[k]] >= 200) continue;

            float tg = curr_g + SQRT2;
            if (tg < g_cost[n_idx]) {
                g_cost[n_idx] = tg;
                parent[n_idx] = curr;
                int dx = abs(nc - goal_c);
                int dy = abs(nr - goal_r);
                float h = (dx < dy) ? (dy + 0.41421356f * dx) : (dx + 0.41421356f * dy);
                heap_push(&heap, tg + h * h_weight, n_idx);
            }
        }
    }

    if (!found) return 0;

    // Reconstruct backwards
    int temp[4096];
    int len = 0;
    int c = goal_idx;
    while (c != -1 && len < 4096) {
        temp[len++] = c;
        c = parent[c];
    }

    int copy_len = len < max_path_len ? len : max_path_len;
    for (int i = 0; i < copy_len; ++i) {
        out_path[i] = temp[len - 1 - i];
    }
    return copy_len;
}
