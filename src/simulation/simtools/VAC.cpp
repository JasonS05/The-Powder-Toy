#include "simulation/ToolCommon.h"
#include "simulation/Air.h"

static int perform(SimTool *tool, Simulation * sim, Particle * cpart, int x, int y, int brushX, int brushY, float strength);

void SimTool::Tool_VAC()
{
	Identifier = "DEFAULT_TOOL_VAC";
	Name = "VAC";
	Colour = 0x303030_rgb;
	Description = "Vacuum, reduces air pressure.";
	Perform = &perform;
}

static int perform(SimTool *tool, Simulation * sim, Particle * cpart, int x, int y, int brushX, int brushY, float strength)
{
	sim->AddPressure(x/CELL, y/CELL, -strength * 0.05f);

	return 1;
}
