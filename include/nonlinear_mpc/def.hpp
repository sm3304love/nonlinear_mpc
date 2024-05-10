#pragma once

#include <array>
#include <cmath>

#define SAMPLE_TIME 0.01
#define TIME_HORIZON 0.03

static constexpr int dof = 6;

static constexpr int num_states = 9;
static constexpr int num_outputs = 0;
static constexpr int num_inputs = 9;
static constexpr int num_arm_links = 7;
static constexpr int obstacle_aviodance = 8;

static constexpr int pred_hor = std::ceil(TIME_HORIZON / SAMPLE_TIME);
static constexpr int ctrl_hor = pred_hor;

constexpr int ineq_c = (pred_hor + 1) * (2 * num_inputs + 2 * num_states + obstacle_aviodance);

constexpr int eq_c = 0;