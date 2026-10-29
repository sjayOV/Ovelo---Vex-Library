#pragma once

#include "api.h"

// PROS locates these competition callbacks by their C linkage.
extern "C" {
void initialize(void);
void disabled(void);
void competition_initialize(void);
void autonomous(void);
void opcontrol(void);
}
