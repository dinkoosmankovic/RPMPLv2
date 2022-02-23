#include "Benchmark.h"
#include "State.h"
#include "RRTConnect.h"

benchmark::Benchmark::Benchmark(/* args */)
{
}

benchmark::Benchmark::~Benchmark()
{
}

void benchmark::Benchmark::addBenchmarkContext(benchmark::BenchmarkContext context)
{
    contexts.emplace_back(context);
}

void benchmark::Benchmark::runContext(BenchmarkContext context)
{
    scenario::Scenario scenario = context.first;
    std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
    std::shared_ptr<planning::AbstractPlanner> planner = nullptr;
    if (context.second == "RRTConnect")
        planner = std::make_shared<planning::rrt::RRTConnect>(ss, scenario.getStart(), scenario.getGoal());
    for (size_t i = 0; i < numberOfRuns; ++i)
    {
        bool res = planner->solve();
        planner->outputPlannerData(benchmarkFile, false, true);
    }
    planner->clearPlanner();
    planner.reset();
    planner = nullptr;
}
void benchmark::Benchmark::runBenchmark()
{
    for (size_t i = 0; i < contexts.size(); ++i)
    {
        runContext(contexts[i]);
    }
}
