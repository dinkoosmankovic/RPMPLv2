#include <vector>
#include <memory>
#include <string>
#include "AbstractPlanner.h"
#include "Scenario.h"

namespace benchmark
{
    typedef std::pair<scenario::Scenario, std::string> BenchmarkContext;
    class Benchmark
    {
    private:
        size_t numberOfRuns = 1;
        std::string benchmarkFile = "/tmp/benchmark.log";
        std::vector<benchmark::BenchmarkContext> contexts;

    private:
        void runContext(BenchmarkContext context);

    public:
        Benchmark(/* args */);
        ~Benchmark();

        void addBenchmarkContext(benchmark::BenchmarkContext context);
        void runBenchmark();
    };
}