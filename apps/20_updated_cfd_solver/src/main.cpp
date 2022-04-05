#include <src/conf/ConfigurationBuilder.h>
#include "src/conf/Configuration.h"

int main() {
    ConfParams params = ConfParams(1, 5, 100,
                                   1000, 4, 100, 50);
    Configuration conf = Configuration();
    ConfigurationName confName = ConfigurationName::CircleWall;
    ConfigurationBuilder(params).build(conf, confName);
    conf.start();
    return 0;
}





