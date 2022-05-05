#include <src/conf/ConfigurationBuilder.h>
#include "src/conf/Configuration.h"

int main() {
    ConfParams params = ConfParams(1, 5, 32,
                                   50, 4, 100, 50);

    //CPU
    {
        Configuration conf_CPU = Configuration();
        conf_CPU.mode = CPU;
        ConfigurationName confName = ConfigurationName::Simple;
        ConfigurationBuilder(params).build(conf_CPU, confName);
        conf_CPU.start();
    }

    //GPU
    {
        Configuration conf_GPU = Configuration();
        conf_GPU.mode = GPU;
        ConfigurationName confName = ConfigurationName::Simple;
        ConfigurationBuilder(params).build(conf_GPU, confName);
        conf_GPU.start();
    }

    return 0;
}





