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

        threshold_X_ = 0.2f;
        threshold_Y_ = 0.2f;
        threshold_V_ = 1.0f;

        max_acc_ = 2.0f; // adjust according to force samples
        min_acc_ = -2.0f;
        max_steering_rate_ = 1.82;

        min_vel_ = -2.0f;
        max_vel_ = 5.0f;

        mass_ = 250.0f;
        minimum_turn_radius_ = 2.74f;

        I_ = 5.0f;
        g_ = 9.8f;
        r_ = 0.2;
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
        return sqrtf(dx*dx + dy*dy)/max_vel_ + std::abs(dq < M_PI ? dq : M_2PI - dq)/(max_steering_rate_);
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {

        return state[V] > min_vel_ &&
               state[V] < max_vel_;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {
       
        return std::abs(goal[X] - state[X]) <= threshold_X_ &&
               std::abs(goal[Y] - state[Y]) <= threshold_Y_;
    }

    /// @brief Set the goal threshold X value
    void set_goal_threshold_X(float goal_threshold_X)
    {
        threshold_X_ = goal_threshold_X;
    }

    /// @brief Set the goal threshold Y value
    void set_goal_threshold_Y(float goal_threshold_Y)
    {
        threshold_Y_ = goal_threshold_Y;
    }

    /// @brief Set the goal threshold V value
    void set_goal_threshold_V(float goal_threshold_V)
    {
        threshold_V_ = goal_threshold_V;
    }

    /// @brief Set the minimum acceleration value
    void set_min_acceleration(float min_acceleration)
    {
        min_acc_ = min_acceleration;
    }

    /// @brief Set the maximum acceleration value
    void set_max_acceleration(float max_acceleration)
    {
        max_acc_ = max_acceleration;
    }

    /// @brief Set the minimum acceleration value
    void set_min_velocity(float min_velocity)
    {
        min_vel_ = min_velocity;
    }

    /// @brief Set the maximum velocity value
    void set_max_velocity(float max_velocity)
    {
        max_vel_ = max_velocity;
    }

    /// @brief Set the mass value
    void set_mass(float mass)
    {
        mass_ = mass;
    }

    /// @brief Set the gravitational value
    void set_gravity(float gravity)
    {
        g_ = gravity;
    }

    /// @brief Set the radius value
    void set_radius(float radius)
    {
        r_ = radius;
    }

    /// @brief Set the moment of inertia value
    void set_I(float I)
    {
        I_ = I;
    }

    /// @brief Set the vehicle minimum turn radius
    void set_minimum_turn_radius(float minimum_turn_radius)
    {
        minimum_turn_radius_ = minimum_turn_radius;
    }

    /// @brief Set the number of integration steps (Euler)
    void set_integration_steps(int integration_steps)
    {
        integration_steps_ = integration_steps;
    }


    /// @brief Set the max steering rate
    void set_max_steering_rate(float max_steering_rate)
    {
        max_steering_rate_ = max_steering_rate;
    }

protected:

    float threshold_X_;
    float threshold_Y_;
    float threshold_V_;
    float min_acc_;
    float max_acc_;
    float min_vel_;
    float max_vel_;
    float minimum_turn_radius_;
    float mass_;
    float I_;
    float r_;
    float g_;
    float integration_steps_;
    float max_steering_rate_;

};

}

#endif