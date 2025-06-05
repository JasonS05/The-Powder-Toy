#include "simulation/ToolCommon.h"

static int perform(SimTool *tool, Simulation * sim, Particle * cpart, int x, int y, int brushX, int brushY, float strength);

void SimTool::Tool_AMBP()
{
	Identifier = "DEFAULT_TOOL_AMBP";
	Name = "AMBP";
	Colour = 0xFFDD00_rgb;
	Description = "Increases ambient air temperature.";
	Perform = &perform;
}

static int perform(SimTool *tool, Simulation *sim, Particle *cpart, int x, int y, int brushX, int brushY, float strength)
{
	if (!sim->aheat_enable)
	{
		return 0;
	}

	sim->AddAmbientHeat(x / CELL, y / CELL, strength * 2.0f);

	return 1;
}
