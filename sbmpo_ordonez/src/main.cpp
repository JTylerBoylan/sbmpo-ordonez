#include <my_project/ForceMomentumUnicycleModel.hpp>
#include <my_project/ForcedAckermannSteeringModel.hpp>
#include <sbmpo/benchmarks/Benchmark.hpp>
#include <sbmpo/benchmarks/Obstacles2DBenchmark.hpp>

using namespace sbmpo;
using namespace sbmpo_ordonez;
using namespace sbmpo_benchmarks;

int main (int argc, char ** argv) {

    // Path to csv workspace
    std::string csv_folder = "/sbmpo_ws/sbmpo_ordonez/csv/";

    // Create new UnicycleSteering benchmark
    //Benchmark<ForceMomentumUnicycleModel> benchmarker(csv_folder);
    //Benchmark<ForcedAckermannSteeringModel> benchmarker(csv_folder);
    Obstacles2DBenchmark<ForcedAckermannSteeringModel> benchmarker(csv_folder);

    // Change benchmark parameters
    benchmarker.model()->set_map_bounds({-20.0f, -20.f, 20.0f, 20.0f});
    benchmarker.model()->set_goal_threshold(0.5f);
    benchmarker.set_runs_per_param(1);

    // Run benchmark (saves to csv folder)
    benchmarker.benchmark();

    return 0;

}