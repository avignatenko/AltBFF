#include "IRunner.h"

thread_local Runner* Runner::threadInstance = nullptr;
