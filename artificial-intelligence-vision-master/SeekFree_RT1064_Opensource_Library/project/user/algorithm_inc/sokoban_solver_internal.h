#ifndef SOKOBAN_SOLVER_INTERNAL_H
#define SOKOBAN_SOLVER_INTERNAL_H

#include "sokoban_engine.h"

typedef struct
{
    uint8_t pos;
    uint8_t target_info;
    uint8_t direction;
} ReconCandidate;

void sokoban_solver_prepare_map(SokobanContext *ctx, const uint8_t *raw_map);
bool sokoban_solver_try_infer_identities(SokobanContext *ctx, State *current_state);
bool sokoban_solver_select_recon_candidate(uint8_t start_pos, uint8_t current_direction,
                                           const ReconCandidate *candidates, uint8_t candidate_count,
                                           const uint8_t *obstacles, ReconCandidate *selected);
bool sokoban_solver_solve_recon(SokobanContext *ctx, State *start_state,
                                const bool *obs_points, const bool *virtual_obs_points);
int sokoban_neighbor_index(int idx, int direction);
int sokoban_recon_direction_to_angle(uint8_t direction);
uint8_t sokoban_recon_angle_to_direction(int current_angle);

#endif // SOKOBAN_SOLVER_INTERNAL_H
