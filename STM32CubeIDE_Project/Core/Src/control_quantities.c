#include "control_quantities.h"
#include <string.h>

void Control_Quantites_Init(ControlQuantities_t *ControlQuantities) {
	memset(ControlQuantities, 0, sizeof(*ControlQuantities));
	ControlQuantities->desired_control_period_ms = CONTROL_PERIOD_MS;

}
