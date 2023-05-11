#ifndef SBMPO_MODEL_FORCE_MOMENTUM_UNICYCLE_HPP_
#define SBMPO_MODEL_FORCE_MOMENTUM_UNICYCLE_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Model.hpp>

#include <math.h>

#define M_2PI 6.283185307179586F

namespace sbmpo_ordonez {

using namespace sbmpo;

class ForceMomentumUnicycleModel : public Model {

    public:

    // States of the Model
    enum States {X, Y, Q, V};

    // Controls of the Model
    enum Controls {Fx, dQdt};

    // Constructor
    ForceMomentumUnicycleModel() {

        integration_steps_ = 50;

        threshold_X_ = 0.5f;
        threshold_Y_ = 0.5f;

        max_steering_rate_ = 1.82;

        min_vel_ = -2.0f;
        max_vel_ = 5.0f;

        mass_ = 500.0f;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {

        // Integrate control into state (Euler)
        State next_state = state;
        const float time_increment = time_span / integration_steps_;
        const float acc = control[Fx] / mass_;
        for (int i = 0; i < integration_steps_; i++) {
            next_state[X] += cosf(next_state[Q]) * next_state[V] * time_increment;
            next_state[Y] += sinf(next_state[Q]) * next_state[V] * time_increment;
            next_state[Q] += control[dQdt] * time_increment;
            next_state[V] += acc * time_increment;
        }

        // Angle wrap
        if (state[Q] > M_PI)         next_state[Q] -= M_2PI;
        else if (state[Q] <= -M_PI)  next_state[Q] += M_2PI;

        return next_state;  

    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {
        return time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {
        const float dx = goal[X] - state[X];
        const float dy = goal[Y] - state[Y];
        const float dq = std::abs(atan2f(dy,dx) - state[Q]);
        return sqrtf(dx*dx + dy*dy)/max_vel_; // + std::abs(dq < M_PI ? dq : M_2PI - dq)/(max_steering_rate_);
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {
        return state[V] >= min_vel_ &&
               state[V] <= max_vel_;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {
        return std::abs(goal[X] - state[X]) <= threshold_X_ &&
               std::abs(goal[Y] - state[Y]) <= threshold_Y_;
    }

protected:

    float integration_steps_;

    float mass_;

    float min_vel_;
    float max_vel_;
    float max_steering_rate_;
    float min_turn_radius_;

    float threshold_X_;
    float threshold_Y_;

};

}

#endif