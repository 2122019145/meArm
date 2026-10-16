/* tiny probe: does g++ accept the firmware header at all? */
#include "constant_and_positions.h"
int main(void) { return (int)(arm1.armLength1 + KIN_RARM_MIN); }
