#ifndef SBMPO_FORCED_ACKERMANN_STEERING_MODEL_HPP_
#define SBMPO_FORCED_ACKERMANN_STEERING_MODEL_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/models/AckermannSteeringModel.hpp>

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

private:

    float mass_;

};

}

#endif