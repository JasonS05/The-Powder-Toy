#include "Air.h"
#include "Simulation.h"
#include "ElementClasses.h"
#include "common/tpt-rand.h"
#include <cmath>
#include <algorithm>
#include "compute/Funcs.h"

using namespace Voxren::Gl;

void Air::Clear()
{
	std::fill(&sim.pv[0][0], &sim.pv[0][0]+NCELL, 0.0f);
	std::fill(&sim.vy[0][0], &sim.vy[0][0]+NCELL, 0.0f);
	std::fill(&sim.vx[0][0], &sim.vx[0][0]+NCELL, 0.0f);
}

void Air::ClearAirH()
{
	std::fill(&sim.hv[0][0], &sim.hv[0][0]+NCELL, ambientAirTemp);
}

// Used when updating temp or velocity from far away
const float advDistanceMult = 0.7f;

// ambient heat update
void Air::update_airh(void)
{
	// TODO
}

float pclamp(float pressure)
{
	return std::clamp(pressure, MIN_PRESSURE, MAX_PRESSURE);
}

void Air::update_air(void) 
{
	auto &vx = sim.vx;
	auto &vy = sim.vy;
	auto &pv = sim.pv;
	auto &fvx = sim.fvx;
	auto &fvy = sim.fvy;
	auto &bmap = sim.bmap;
	if (airMode != AIR_NOUPDATE) //airMode 4 is no air/pressure update
	{
		for (auto j=1; j<YCELLS-1; j++) // clear pressures and velocities inside walls and limit pressures
		{
			for (auto i=1; i<XCELLS-1; i++)
			{
				if (bmap_blockair[j][i])
				{
					vx[j][i] = 0.0f;
					vy[j][i] = 0.0f;
					pv[j][i] = 0.0f;
				}

				pv[j][i] = pclamp(pv[j][i]);
			}
		}

		for (auto y = 0; y < YCELLS; y++) // update velocity
		{
			for (auto x = 0; x < XCELLS; x++)
			{
				if (bmap[y][x] == WL_FAN)
				{
					vx[y][x] += fvx[y][x];
					vy[y][x] += fvy[y][x];
				}
			}
		}

		air_shader.init(); // does nothing after the first invocation
		air_shader.upload(sim, this);
		air_shader.run(8);
		air_shader.download(sim);
	}
}

void Air::Invert()
{
	auto &vx = sim.vx;
	auto &vy = sim.vy;
	auto &pv = sim.pv;
	for (auto nx = 0; nx<XCELLS; nx++)
	{
		for (auto ny = 0; ny<YCELLS; ny++)
		{
			pv[ny][nx] = -pv[ny][nx];
			vx[ny][nx] = -vx[ny][nx];
			vy[ny][nx] = -vy[ny][nx];
		}
	}
}

// called when loading saves / stamps to ensure nothing "leaks" the first frame
void Air::ApproximateBlockAirMaps()
{
	auto &sd = SimulationData::CRef();
	auto &elements = sd.elements;
	for (int i = 0; i <= sim.parts.lastActiveIndex; i++)
	{
		int type = sim.parts[i].type;
		if (!type)
			continue;
		// Real TTAN would only block if there was enough TTAN
		// but it would be more expensive and complicated to actually check that
		// so just block for a frame, if it wasn't supposed to block it will continue allowing air next frame
		if (type == PT_TTAN)
		{
			int x = ((int)(sim.parts[i].x+0.5f))/CELL, y = ((int)(sim.parts[i].y+0.5f))/CELL;
			if (InBounds(x, y))
			{
				bmap_blockair[y][x] = 1;
				bmap_blockairh[y][x] = 0x8;
			}
		}
		// mostly accurate insulator blocking, besides checking GEL
		else if (sim.IsHeatInsulator(sim.parts[i]) || elements[type].HeatConduct <= (sim.rng()%250))
		{
			int x = ((int)(sim.parts[i].x+0.5f))/CELL, y = ((int)(sim.parts[i].y+0.5f))/CELL;
			if (InBounds(x, y) && !(bmap_blockairh[y][x]&0x8))
				bmap_blockairh[y][x]++;
		}
	}
}

Air::Air(Simulation & simulation):
	sim(simulation),
	airMode(AIR_ON),
	ambientAirTemp(R_TEMP + 273.15f)
{
	//Simulation should do this.
	std::fill(&bmap_blockair [0][0], &bmap_blockair [0][0] + NCELL, 0);
	std::fill(&bmap_blockairh[0][0], &bmap_blockairh[0][0] + NCELL, 0);
	std::fill(&sim.vx[0][0], &sim.vx[0][0] + NCELL, 0.0f);
	std::fill(&ovx   [0][0], &ovx   [0][0] + NCELL, 0.0f);
	std::fill(&sim.vy[0][0], &sim.vy[0][0] + NCELL, 0.0f);
	std::fill(&ovy   [0][0], &ovy   [0][0] + NCELL, 0.0f);
	std::fill(&sim.hv[0][0], &sim.hv[0][0] + NCELL, 0.0f);
	std::fill(&ohv   [0][0], &ohv   [0][0] + NCELL, 0.0f);
	std::fill(&sim.pv[0][0], &sim.pv[0][0] + NCELL, 0.0f);
	std::fill(&opv   [0][0], &opv   [0][0] + NCELL, 0.0f);
}
