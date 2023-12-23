#include "Air.h"
#include "Simulation.h"
#include "ElementClasses.h"
#include "common/tpt-rand.h"
#include <cmath>
#include <algorithm>

void Air::Clear()
{
	std::fill(&sim.pv[0][0], &sim.pv[0][0]+NCELL, 0.0f);
	std::fill(&sim.vy[0][0], &sim.vy[0][0]+NCELL, 0.0f);
	std::fill(&sim.vx[0][0], &sim.vx[0][0]+NCELL, 0.0f);
	std::fill(&sim.ec[0][0], &sim.ec[0][0]+NCELL, false); // this line may be broken due to rebase, can't be bothered to fix since it's not relevant because of later commits
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

typedef struct
{
	float xmomentum;
	float ymomentum;
	float density;
} AirValues;

const float dt = 0.1f;
const float extremalDiffusionQuantity = 0.3f;

float calculatePressure(float density)
{
	return density;
}

AirValues calculateHorizontalEdgeFlux(AirValues leftCell,  AirValues rightCell, bool extremal)
{
	auto averageDensity = std::sqrt(leftCell.density * rightCell.density);

	AirValues average =
	{
		0.5f * (leftCell.xmomentum / leftCell.density + rightCell.xmomentum / rightCell.density) * averageDensity,
		0.5f * (leftCell.ymomentum / leftCell.density + rightCell.ymomentum / rightCell.density) * averageDensity,
		averageDensity
	};

	float left_vx = leftCell.xmomentum / leftCell.density;

	AirValues left_flux =
	{
		leftCell.xmomentum * left_vx + calculatePressure(leftCell.density),
		leftCell.ymomentum * left_vx,
		leftCell.xmomentum
	};

	float right_vx = rightCell.xmomentum / rightCell.density;

	AirValues right_flux =
	{
		rightCell.xmomentum * right_vx + calculatePressure(rightCell.density),
		rightCell.ymomentum * right_vx,
		rightCell.xmomentum
	};

	AirValues approximateValues =
	{
		average.xmomentum + 0.5f * dt * (left_flux.xmomentum - right_flux.xmomentum),
		average.ymomentum + 0.5f * dt * (left_flux.ymomentum - right_flux.ymomentum),
		std::max(average.density + 0.5f * dt * (left_flux.density - right_flux.density), 0.1f)
	};

	float approximate_vx = approximateValues.xmomentum / approximateValues.density;
	float diffusion = extremal? extremalDiffusionQuantity : 0.0f;

	return
	{
		approximateValues.xmomentum * approximate_vx + diffusion * (leftCell.xmomentum - rightCell.xmomentum) + calculatePressure(approximateValues.density),
		approximateValues.ymomentum * approximate_vx + diffusion * (leftCell.ymomentum - rightCell.ymomentum),
		approximateValues.xmomentum + diffusion * (leftCell.density - rightCell.density)
	};
}

AirValues calculateVerticalEdgeFlux(AirValues lowerCell,  AirValues upperCell, bool extremal)
{
	auto averageDensity = std::sqrt(lowerCell.density * upperCell.density);

	AirValues average =
	{
		0.5f * (lowerCell.xmomentum / lowerCell.density + upperCell.xmomentum / upperCell.density) * averageDensity,
		0.5f * (lowerCell.ymomentum / lowerCell.density + upperCell.ymomentum / upperCell.density) * averageDensity,
		averageDensity
	};

	float lower_vy = lowerCell.ymomentum / lowerCell.density;

	AirValues lower_flux =
	{
		lowerCell.xmomentum * lower_vy,
		lowerCell.ymomentum * lower_vy + calculatePressure(lowerCell.density),
		lowerCell.ymomentum
	};

	float upper_vy = upperCell.ymomentum / upperCell.density;

	AirValues upper_flux =
	{
		upperCell.xmomentum * upper_vy,
		upperCell.ymomentum * upper_vy + calculatePressure(upperCell.density),
		upperCell.ymomentum
	};

	AirValues approximateValues =
	{
		average.xmomentum + 0.5f * dt * (lower_flux.xmomentum - upper_flux.xmomentum),
		average.ymomentum + 0.5f * dt * (lower_flux.ymomentum - upper_flux.ymomentum),
		std::max(average.density + 0.5f * dt * (lower_flux.density - upper_flux.density), 0.1f)
	};

	float approximate_vy = approximateValues.ymomentum / approximateValues.density;
	float diffusion = extremal? extremalDiffusionQuantity : 0.0f;

	return
	{
		approximateValues.xmomentum * approximate_vy + diffusion * (lowerCell.xmomentum - upperCell.xmomentum),
		approximateValues.ymomentum * approximate_vy + diffusion * (lowerCell.ymomentum - upperCell.ymomentum) + calculatePressure(approximateValues.density),
		approximateValues.ymomentum + diffusion * (lowerCell.density - upperCell.density)
	};
}

float pclamp(float pressure)
{
	return std::clamp(pressure, -10.0f, MAX_PRESSURE);
}

void Air::update_air(void) 
{
	for (int i = 0; i < 3; i++) update_air_();
}

void Air::update_air_(void)
{
	auto &vx = sim.vx;
	auto &vy = sim.vy;
	auto &pv = sim.pv;
	auto &fvx = sim.fvx;
	auto &fvy = sim.fvy;
	auto &bmap = sim.bmap;
	if (airMode != AIR_NOUPDATE) //airMode 4 is no air/pressure update
	{
		for (auto i=0; i<YCELLS; i++) //reduces pressure/velocity on the edges every frame
		{
			pv[i][0] = pv[i][0]*0.8f;
			pv[i][1] = pv[i][1]*0.8f;
			pv[i][XCELLS-2] = pv[i][XCELLS-2]*0.8f;
			pv[i][XCELLS-1] = pv[i][XCELLS-1]*0.8f;
			vx[i][0] = vx[i][0]*0.9f;
			vx[i][1] = vx[i][1]*0.9f;
			vx[i][XCELLS-2] = vx[i][XCELLS-2]*0.9f;
			vx[i][XCELLS-1] = vx[i][XCELLS-1]*0.9f;
			vy[i][0] = vy[i][0]*0.9f;
			vy[i][1] = vy[i][1]*0.9f;
			vy[i][XCELLS-2] = vy[i][XCELLS-2]*0.9f;
			vy[i][XCELLS-1] = vy[i][XCELLS-1]*0.9f;
		}
		for (auto i=0; i<XCELLS; i++) //reduces pressure/velocity on the edges every frame
		{
			pv[0][i] = pv[0][i]*0.8f;
			pv[1][i] = pv[1][i]*0.8f;
			pv[YCELLS-2][i] = pv[YCELLS-2][i]*0.8f;
			pv[YCELLS-1][i] = pv[YCELLS-1][i]*0.8f;
			vx[0][i] = vx[0][i]*0.9f;
			vx[1][i] = vx[1][i]*0.9f;
			vx[YCELLS-2][i] = vx[YCELLS-2][i]*0.9f;
			vx[YCELLS-1][i] = vx[YCELLS-1][i]*0.9f;
			vy[0][i] = vy[0][i]*0.9f;
			vy[1][i] = vy[1][i]*0.9f;
			vy[YCELLS-2][i] = vy[YCELLS-2][i]*0.9f;
			vy[YCELLS-1][i] = vy[YCELLS-1][i]*0.9f;
		}

		for (auto j=1; j<YCELLS-1; j++) // clear some pressures and velocities near walls and limit pressures/velocities
		{
			for (auto i=1; i<XCELLS-1; i++)
			{
				if (bmap_blockair[j][i])
				{
					vx[j][i] = 0.0f;
					vx[j][i-1] = 0.0f;
					vx[j][i+1] = 0.0f;
					vy[j][i] = 0.0f;
					vy[j-1][i] = 0.0f;
					vy[j+1][i] = 0.0f;
					pv[j][i] = 0.0f;
				}

				pv[j][i] = pclamp(pv[j][i]);

				float speed = std::hypot(vx[j][i], vy[j][i]) / (pv[j][i] + 10.1f);

				if (speed > 3.0f) {
					vx[j][i] *= 3.0f / speed;
					vy[j][i] *= 3.0f / speed;
				}
			}
		}

		for (auto y = 0; y < YCELLS; y++)
		{
			for (auto x = 0; x < XCELLS; x++)
			{
				// x +/- 1
				auto xm1 = std::max(x - 1, 0);
				auto xp1 = std::min(x + 1, XCELLS - 1);

				if (bmap_blockair[y][xm1]) xm1 = x;
				if (bmap_blockair[y][xp1]) xp1 = x;

				// y +/- 1
				auto ym1 = std::max(y - 1, 0);
				auto yp1 = std::min(y + 1, YCELLS - 1);

				if (bmap_blockair[ym1][x]) ym1 = y;
				if (bmap_blockair[yp1][x]) yp1 = y;

				ec[y][x] =
					(pv[y][x] > pv[y][xm1] && pv[y][x] > pv[y][xp1]) ||
					(pv[y][x] < pv[y][xm1] && pv[y][x] < pv[y][xp1]) ||
					(pv[y][x] > pv[ym1][x] && pv[y][x] > pv[yp1][x]) ||
					(pv[y][x] < pv[ym1][x] && pv[y][x] < pv[yp1][x]);
			}
		}

		for (auto y=0; y<YCELLS; y++) //update velocity and pressure
		{
			for (auto x=0; x<XCELLS; x++)
			{
				auto dx = vx[y][x];
				auto dy = vy[y][x];
				auto dp = pv[y][x];

				// x +/- 1
				auto xm1 = std::max(x - 1, 0);
				auto xp1 = std::min(x + 1, XCELLS - 1);

				if (bmap_blockair[y][xm1]) xm1 = x;
				if (bmap_blockair[y][xp1]) xp1 = x;

				// y +/- 1
				auto ym1 = std::max(y - 1, 0);
				auto yp1 = std::min(y + 1, YCELLS - 1);

				if (bmap_blockair[ym1][x]) ym1 = y;
				if (bmap_blockair[yp1][x]) yp1 = y;

				AirValues thisCell =
				{
					vx[y][x],
					-vy[y][x],
					pv[y][x] + 10.1f
				};

				AirValues leftCell =
				{
					vx[y][xm1],
					-vy[y][xm1],
					pv[y][xm1] + 10.1f
				};

				AirValues rightCell =
				{
					vx[y][xp1],
					-vy[y][xp1],
					pv[y][xp1] + 10.1f
				};

				AirValues lowerCell =
				{
					vx[yp1][x],
					-vy[yp1][x],
					pv[yp1][x] + 10.1f
				};

				AirValues upperCell =
				{
					vx[ym1][x],
					-vy[ym1][x],
					pv[ym1][x] + 10.1f
				};

				auto leftFlux = calculateHorizontalEdgeFlux(leftCell, thisCell, ec[y][xm1] || ec[y][x]);
				auto rightFlux = calculateHorizontalEdgeFlux(thisCell, rightCell, ec[y][x] || ec[y][xp1]);
				auto lowerFlux = calculateVerticalEdgeFlux(lowerCell, thisCell, ec[yp1][x] || ec[y][x]);
				auto upperFlux = calculateVerticalEdgeFlux(thisCell, upperCell, ec[y][x] || ec[ym1][x]);

				dx += dt * (leftFlux.xmomentum + lowerFlux.xmomentum - rightFlux.xmomentum - upperFlux.xmomentum);
				dy -= dt * (leftFlux.ymomentum + lowerFlux.ymomentum - rightFlux.ymomentum - upperFlux.ymomentum);
				dp += dt * (leftFlux.density + lowerFlux.density - rightFlux.density - upperFlux.density);

				if (bmap[y][x] == WL_FAN)
				{
					dx += fvx[y][x];
					dy += fvy[y][x];
				}

				// pressure caps
				if (std::isnan(dp)) dp = 0.0f;
				if (dp > MAX_PRESSURE) dp = MAX_PRESSURE;
				if (dp < -10.0f) dp = -10.0f;

				// velocity caps
				if (std::isnan(dx)) dx = 0.0f;
				if (std::isnan(dy)) dy = 0.0f;

				float speed = std::hypot(dx, dy) / (dp + 10.1f);
				if (speed > 3.0f) {
					dx *= 3.0f / speed;
					dy *= 3.0f / speed;
				}

				switch (airMode)
				{
				default:
				case AIR_ON:  //Default
					break;
				case AIR_PRESSUREOFF:  //0 Pressure
					dp = 0.0f;
					break;
				case AIR_VELOCITYOFF:  //0 Velocity
					dx = 0.0f;
					dy = 0.0f;
					break;
				case AIR_OFF: //0 Air
					dx = 0.0f;
					dy = 0.0f;
					dp = 0.0f;
					break;
				case AIR_NOUPDATE: //No Update
					break;
				}

				ovx[y][x] = dx;
				ovy[y][x] = dy;
				opv[y][x] = dp;
			}
		}
		memcpy(vx, ovx, sizeof(vx));
		memcpy(vy, ovy, sizeof(vy));
		memcpy(pv, opv, sizeof(pv));
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
