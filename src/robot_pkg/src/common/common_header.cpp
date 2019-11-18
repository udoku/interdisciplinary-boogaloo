#include "common_header.hpp"

/**
 * Returns the current ms count from the linux Epoch
 */
long long getCurrentMsCount() {
    timeval t1;
    gettimeofday(&t1, NULL);
    return (t1.tv_sec * 1000.0) + (t1.tv_usec / 1000.0);
}