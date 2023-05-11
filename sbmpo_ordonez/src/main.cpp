#include <sbmpo_ordonez/ForceMomentumUnicycleModel.hpp>
#include <sbmpo_ordonez/ForcedAckermannSteeringModel.hpp>
#include <sbmpo/benchmarks/Benchmark.hpp>

using namespace sbmpo;
using namespace sbmpo_ordonez;
using namespace sbmpo_benchmarks;

int main (int argc, char ** argv) {

    // Path to csv workspace
    std::string csv_folder;

    // Check arguments
    if (argc > 1) {
        csv_folder = argv[1];
    } else {
        printf("\nMissing CSV folder path.\n");
        return 0;
    }

    // Create new UnicycleSteering benchmark
    //Benchmark<ForceMomentumUnicycleModel> benchmarker(csv_folder);
    Benchmark<ForcedAckermannSteeringModel> benchmarker(csv_folder);

    // Change benchmark parameters
    benchmarker.set_runs_per_param(1);

    // Run benchmark (saves to csv folder)
    benchmarker.benchmark();

    return 0;

}