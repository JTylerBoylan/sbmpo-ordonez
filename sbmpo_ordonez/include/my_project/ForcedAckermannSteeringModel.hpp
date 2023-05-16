#ifndef SBMPO_FORCED_ACKERMANN_STEERING_MODEL_HPP_
#define SBMPO_FORCED_ACKERMANN_STEERING_MODEL_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/models/AckermannSteeringModel.hpp>
#include <math.h>

namespace sbmpo_ordonez {

using namespace sbmpo;
using namespace sbmpo_models;

class ForcedAckermannSteeringModel : public AckermannSteeringModel {

enum ForceControls {Fx, dGdt};

public:

    ForcedAckermannSteeringModel() 
    : AckermannSteeringModel() {
        this->max_velocity_ = 5.0;
        this->min_velocity_ = -2.0;

        mass_ = 500.0f;
    }

    State next_state(const State& state, const Control& control, const float time_span) override {
        const float acc = control[Fx] / mass_;
        const Control new_control = {acc, control[dGdt]};
        return AckermannSteeringModel::next_state(state, new_control, time_span);
    }

    float heuristic(const State& state, const State& goal) override {
        const float dx = goal[X] - state[X];
        const float dy = goal[Y] - state[Y];
        const float dq = std::abs(atan2f(dy,dx) - state[Q]);
        return sqrtf(dx*dx + dy*dy)/max_velocity_ + std::abs(dq < M_PI ? dq : M_2PI - dq)/(max_velocity_*max_turn_angle_*inv_wheel_base_length_);
    }

private:

    float mass_;

};

}

#endif