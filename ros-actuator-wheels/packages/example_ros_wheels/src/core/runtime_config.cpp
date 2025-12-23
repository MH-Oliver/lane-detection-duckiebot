#include "runtime_config.h"

double RuntimeConfig::execution_duration = 45.0;

void RuntimeConfig::setDuration(double seconds) {
    execution_duration = seconds;
}