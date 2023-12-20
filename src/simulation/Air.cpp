#include "Air.h"
#include "Simulation.h"
#include "ElementClasses.h"
#include "common/tpt-rand.h"
#include <cmath>
#include <algorithm>

void Air::make_kernel(void) //used for velocity
{
	/*float s = 0.0f;
	for (auto j=-1; j<2; j++)
	{
		for (auto i=-1; i<2; i++)
		{
			kernel[(i+1)+3*(j+1)] = expf(-2.0f*(i*i+j*j));
			s += kernel[(i+1)+3*(j+1)];
		}
	}
	s = 1.0f / s;
	for (auto j=-1; j<2; j++)
	{
		for (auto i=-1; i<2; i++)
		{
			kernel[(i+1)+3*(j+1)] *= s;
		}
	}*/
	/*kernel[0] = 0.0;
	kernel[1] = 0.0;
	kernel[2] = 0.0;
	kernel[3] = 0.0;
	kernel[4] = 1.0;
	kernel[5] = 0.0;
	kernel[6] = 0.0;
	kernel[7] = 0.0;
	kernel[8] = 0.0;*/
}

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
	auto &vx = sim.vx;
	auto &vy = sim.vy;
	auto &hv = sim.hv;
	/*for (auto i=0; i<YCELLS; i++) //sets air temp on the edges every frame
	{
		hv[i][0] = ambientAirTemp;
		hv[i][1] = ambientAirTemp;
		hv[i][XCELLS-2] = ambientAirTemp;
		hv[i][XCELLS-1] = ambientAirTemp;
	}
	for (auto i=0; i<XCELLS; i++) //sets air temp on the edges every frame
	{
		hv[0][i] = ambientAirTemp;
		hv[1][i] = ambientAirTemp;
		hv[YCELLS-2][i] = ambientAirTemp;
		hv[YCELLS-1][i] = ambientAirTemp;
	}
	for (auto y=0; y<YCELLS; y++) //update air temp and velocity
	{
		for (auto x=0; x<XCELLS; x++)
		{
			auto dh = 0.0f;
			auto dx = 0.0f;//vx[y][x];
			auto dy = 0.0f;//vy[y][x];
			for (auto j=-1; j<2; j++)
			{
				for (auto i=-1; i<2; i++)
				{
					if (y+j>0 && y+j<YCELLS-2 && x+i>0 && x+i<XCELLS-2 && !(bmap_blockairh[y+j][x+i]&0x8))
					{
						auto f = kernel[i+1+(j+1)*3];
						dh += hv[y+j][x+i]*f;
						dx += vx[y+j][x+i]*f;
						dy += vy[y+j][x+i]*f;
					}
					else
					{
						auto f = kernel[i+1+(j+1)*3];
						dh += hv[y][x]*f;
						dx += vx[y][x]*f;
						dy += vy[y][x]*f;
					}
				}
			}

			// Trying to take air temp from far away.
			// The code is almost identical to the "far away" velocity code from update_air
			auto tx = x - dx*advDistanceMult;
			auto ty = y - dy*advDistanceMult;
			if ((std::abs(dx*advDistanceMult)>1.0f || std::abs(dy*advDistanceMult)>1.0f) && (tx>=2 && tx<XCELLS-2 && ty>=2 && ty<YCELLS-2))
			{
				float stepX, stepY;
				int stepLimit;
				if (std::abs(dx)>std::abs(dy))
				{
					stepX = (dx<0.0f) ? 1.f : -1.f;
					stepY = -dy/fabsf(dx);
					stepLimit = (int)(fabsf(dx*advDistanceMult));
				}
				else
				{
					stepY = (dy<0.0f) ? 1.f : -1.f;
					stepX = -dx/fabsf(dy);
					stepLimit = (int)(fabsf(dy*advDistanceMult));
				}
				tx = float(x);
				ty = float(y);
				auto step = 0;
				for (; step<stepLimit; ++step)
				{
					tx += stepX;
					ty += stepY;
					if (bmap_blockairh[(int)(ty+0.5f)][(int)(tx+0.5f)]&0x8)
					{
						tx -= stepX;
						ty -= stepY;
						break;
					}
				}
				if (step==stepLimit)
				{
					// No wall found
					tx = x - dx*advDistanceMult;
					ty = y - dy*advDistanceMult;
				}
			}

			auto i = (int)tx;
			auto j = (int)ty;
			tx -= i;
			ty -= j;
			if (!(bmap_blockairh[y][x]&0x8) && i>=2 && i<XCELLS-3 && j>=2 && j<YCELLS-3)
			{
				auto odh = dh;
				dh *= 1.0f - AIR_VADV;
				dh += AIR_VADV*(1.0f-tx)*(1.0f-ty)*((bmap_blockairh[j][i]&0x8) ? odh : hv[j][i]);
				dh += AIR_VADV*tx*(1.0f-ty)*((bmap_blockairh[j][i+1]&0x8) ? odh : hv[j][i+1]);
				dh += AIR_VADV*(1.0f-tx)*ty*((bmap_blockairh[j+1][i]&0x8) ? odh : hv[j+1][i]);
				dh += AIR_VADV*tx*ty*((bmap_blockairh[j+1][i+1]&0x8) ? odh : hv[j+1][i+1]);
			}

			// Temp caps
			if (dh > MAX_TEMP) dh = MAX_TEMP;
			if (dh < MIN_TEMP) dh = MIN_TEMP;

			ohv[y][x] = dh;

			// Air convection.
			// We use the Boussinesq approximation, i.e. we assume density to be nonconstant only
			// near the gravity term of the fluid equation, and we suppose that it depends linearly on the
			// difference between the current temperature (hv[y][x]) and some "stationary" temperature (ambientAirTemp).
			if (x>=2 && x<XCELLS-2 && y>=2 && y<YCELLS-2)
			{
				float convGravX, convGravY;
				sim.GetGravityField(x*CELL, y*CELL, -1.0f, -1.0f, convGravX, convGravY);
				auto weight = (hv[y][x] - ambientAirTemp) / 10000.0f;

				// Our approximation works best when the temperature difference is small, so we cap it from above.
				if (weight > 0.01f) weight = 0.01f;

				vx[y][x] += weight * convGravX;
				vy[y][x] += weight * convGravY;
			}
		}
	}
	memcpy(hv, ohv, sizeof(hv));*/
}

// comment out the following line to switch from lax-wendroff to lax-friedrichs
#define WENDROFF WENDROFF

typedef struct {
	float xmomentum;
	float ymomentum;
	float density;
} AirValues;

const float dt = 0.1f;

const float pressureDiffusion = 0.0f;
const float velocityDiffusion = 0.0f;
const float extremalDiffusionQuantity = 0.3f;

const float friedrichDiffusion = 0.03f;

float calculatePressure(float density) {
	//density /= 15.0f;
	//return 15.0f * (density - 2.0f * density * density + density * density * density);
	return density;
}

AirValues calculateHorizontalEdgeFlux(AirValues leftCell,  AirValues rightCell, bool extremal) {
	auto averageDensity = std::sqrt(leftCell.density * rightCell.density);

	AirValues average = {
		0.5f * (leftCell.xmomentum / leftCell.density + rightCell.xmomentum / rightCell.density) * averageDensity,
		0.5f * (leftCell.ymomentum / leftCell.density + rightCell.ymomentum / rightCell.density) * averageDensity,
		averageDensity
	};

	float left_vx = leftCell.xmomentum / leftCell.density;

	AirValues left_flux = {
		leftCell.xmomentum * left_vx + calculatePressure(leftCell.density),
		leftCell.ymomentum * left_vx,
		leftCell.xmomentum
	};

	float right_vx = rightCell.xmomentum / rightCell.density;

	AirValues right_flux = {
		rightCell.xmomentum * right_vx + calculatePressure(rightCell.density),
		rightCell.ymomentum * right_vx,
		rightCell.xmomentum
	};

	AirValues approximateValues = {
		average.xmomentum + 0.5f * dt * (left_flux.xmomentum - right_flux.xmomentum),
		average.ymomentum + 0.5f * dt * (left_flux.ymomentum - right_flux.ymomentum),
		std::max(average.density + 0.5f * dt * (left_flux.density - right_flux.density), 0.1f)
	};

	#ifdef WENDROFF
	float approximate_vx = approximateValues.xmomentum / approximateValues.density;
	float extremalDiffusion = 0.0f;
	if (extremal) extremalDiffusion = extremalDiffusionQuantity;// + std::hypot(approximateValues.xmomentum, approximateValues.ymomentum) / approximateValues.density;

	return {
		approximateValues.xmomentum * approximate_vx + (velocityDiffusion + extremalDiffusion) * (leftCell.xmomentum - rightCell.xmomentum) + calculatePressure(approximateValues.density),
		approximateValues.ymomentum * approximate_vx + (velocityDiffusion + extremalDiffusion) * (leftCell.ymomentum - rightCell.ymomentum),
		approximateValues.xmomentum + (pressureDiffusion + extremalDiffusion) * (leftCell.density - rightCell.density)
	};
	#else
	float newFriedrichDiffusion = friedrichDiffusion + std::max(std::hypot(leftCell.xmomentum, leftCell.ymomentum) / leftCell.density, std::hypot(rightCell.xmomentum, rightCell.ymomentum) / rightCell.density) * 0.5f;
	return {
		0.5f * (right_flux.xmomentum + left_flux.xmomentum) + newFriedrichDiffusion * (leftCell.xmomentum - rightCell.xmomentum),
		0.5f * (right_flux.ymomentum + left_flux.ymomentum) + newFriedrichDiffusion * (leftCell.ymomentum - rightCell.ymomentum),
		0.5f * (right_flux.density + left_flux.density) + newFriedrichDiffusion * (leftCell.density - rightCell.density)
	};
	#endif
}

AirValues calculateVerticalEdgeFlux(AirValues lowerCell,  AirValues upperCell, bool extremal) {
	auto averageDensity = std::sqrt(lowerCell.density * upperCell.density);

	AirValues average = {
		0.5f * (lowerCell.xmomentum / lowerCell.density + upperCell.xmomentum / upperCell.density) * averageDensity,
		0.5f * (lowerCell.ymomentum / lowerCell.density + upperCell.ymomentum / upperCell.density) * averageDensity,
		averageDensity
	};

	float lower_vy = lowerCell.ymomentum / lowerCell.density;

	AirValues lower_flux = {
		lowerCell.xmomentum * lower_vy,
		lowerCell.ymomentum * lower_vy + calculatePressure(lowerCell.density),
		lowerCell.ymomentum
	};

	float upper_vy = upperCell.ymomentum / upperCell.density;

	AirValues upper_flux = {
		upperCell.xmomentum * upper_vy,
		upperCell.ymomentum * upper_vy + calculatePressure(upperCell.density),
		upperCell.ymomentum
	};

	AirValues approximateValues = {
		average.xmomentum + 0.5f * dt * (lower_flux.xmomentum - upper_flux.xmomentum),
		average.ymomentum + 0.5f * dt * (lower_flux.ymomentum - upper_flux.ymomentum),
		std::max(average.density + 0.5f * dt * (lower_flux.density - upper_flux.density), 0.1f)
	};

	#ifdef WENDROFF
	float approximate_vy = approximateValues.ymomentum / approximateValues.density;
	float extremalDiffusion = 0.0f;
	if (extremal) extremalDiffusion = extremalDiffusionQuantity;// + std::hypot(approximateValues.xmomentum, approximateValues.ymomentum) / approximateValues.density;

	return {
		approximateValues.xmomentum * approximate_vy + (velocityDiffusion + extremalDiffusion) * (lowerCell.xmomentum - upperCell.xmomentum),
		approximateValues.ymomentum * approximate_vy + (velocityDiffusion + extremalDiffusion) * (lowerCell.ymomentum - upperCell.ymomentum) + calculatePressure(approximateValues.density),
		approximateValues.ymomentum + (pressureDiffusion + extremalDiffusion) * (lowerCell.density - upperCell.density)
	};
	#else
	float newFriedrichDiffusion = friedrichDiffusion + std::max(std::hypot(lowerCell.xmomentum, lowerCell.ymomentum) / lowerCell.density, std::hypot(upperCell.xmomentum, upperCell.ymomentum) / upperCell.density) * 0.5f;
	return {
		0.5f * (lower_flux.xmomentum + upper_flux.xmomentum) + newFriedrichDiffusion * (lowerCell.xmomentum - upperCell.xmomentum),
		0.5f * (lower_flux.ymomentum + upper_flux.ymomentum) + newFriedrichDiffusion * (lowerCell.ymomentum - upperCell.ymomentum),
		0.5f * (lower_flux.density + upper_flux.density) + newFriedrichDiffusion * (lowerCell.density - upperCell.density)
	};
	#endif
}

float pclamp(float pressure) {
	return std::clamp(pressure, -10.0f, MAX_PRESSURE);
}

void Air::update_air(void) {
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

		for (auto y = 0; y < YCELLS; y++) {
			for (auto x = 0; x < XCELLS; x++) {
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
				/*||
					(vx[y][x] > vx[y][xm1] && vx[y][x] > vx[y][xp1]) ||
					(vx[y][x] < vx[y][xm1] && vx[y][x] < vx[y][xp1]) ||
					(vx[y][x] > vx[ym1][x] && vx[y][x] > vx[yp1][x]) ||
					(vx[y][x] < vx[ym1][x] && vx[y][x] < vx[yp1][x])
				||
					(vy[y][x] > vy[y][xm1] && vy[y][x] > vy[y][xp1]) ||
					(vy[y][x] < vy[y][xm1] && vy[y][x] < vy[y][xp1]) ||
					(vy[y][x] > vy[ym1][x] && vy[y][x] > vy[yp1][x]) ||
					(vy[y][x] < vy[ym1][x] && vy[y][x] < vy[yp1][x]);*/
			}
		}

		/*for (auto y=1; y<YCELLS-1; y++) //pressure adjustments from velocity
		{
			for (auto x=1; x<XCELLS-1; x++)
			{
				auto dp = 0.0f;
				dp += vx[y][x-1] - vx[y][x+1];
				dp += vy[y-1][x] - vy[y+1][x];
				pv[y][x] *= AIR_PLOSS;
				pv[y][x] += dp*AIR_TSTEPP * 0.5f;
			}
		}

		for (auto y=1; y<YCELLS-1; y++) //velocity adjustments from pressure
		{
			for (auto x=1; x<XCELLS-1; x++)
			{
				auto dx = 0.0f;
				auto dy = 0.0f;
				dx += pv[y][x-1] - pv[y][x+1];
				dy += pv[y-1][x] - pv[y+1][x];
				vx[y][x] *= AIR_VLOSS;
				vy[y][x] *= AIR_VLOSS;
				vx[y][x] += dx*AIR_TSTEPV * 0.5f;
				vy[y][x] += dy*AIR_TSTEPV * 0.5f;
				if (bmap_blockair[y][x-1] || bmap_blockair[y][x] || bmap_blockair[y][x+1])
					vx[y][x] = 0;
				if (bmap_blockair[y-1][x] || bmap_blockair[y][x] || bmap_blockair[y+1][x])
					vy[y][x] = 0;
			}
		}*/

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

				AirValues thisCell = {
					vx[y][x],
					-vy[y][x],
					pv[y][x] + 10.1f
				};

				AirValues leftCell = {
					vx[y][xm1],
					-vy[y][xm1],
					pv[y][xm1] + 10.1f
				};

				AirValues rightCell = {
					vx[y][xp1],
					-vy[y][xp1],
					pv[y][xp1] + 10.1f
				};

				AirValues lowerCell = {
					vx[yp1][x],
					-vy[yp1][x],
					pv[yp1][x] + 10.1f
				};

				AirValues upperCell {
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

				//dy += dt * 0.01f * (dp + 10.01f);

				/*for (auto j=-1; j<2; j++)
				{
					for (auto i=-1; i<2; i++)
					{
						if (y+j>0 && y+j<YCELLS-1 &&
						        x+i>0 && x+i<XCELLS-1 &&
						        !bmap_blockair[y+j][x+i])
						{
							auto f = kernel[i+1+(j+1)*3];
							dx += vx[y+j][x+i]*f;
							dy += vy[y+j][x+i]*f;
							dp += pv[y+j][x+i]*f;
						}
						else
						{
							auto f = kernel[i+1+(j+1)*3];
							dx += vx[y][x]*f;
							dy += vy[y][x]*f;
							dp += pv[y][x]*f;
						}
					}
				}

				auto tx = x - dx*advDistanceMult;
				auto ty = y - dy*advDistanceMult;
				if ((std::abs(dx*advDistanceMult)>1.0f || std::abs(dy*advDistanceMult)>1.0f) && (tx>=2 && tx<XCELLS-2 && ty>=2 && ty<YCELLS-2))
				{
					// Trying to take velocity from far away, check whether there is an intervening wall.
					// Step from current position to desired source location, looking for walls, with either the x or y step size being 1 cell
					float stepX, stepY;
					int stepLimit;
					if (std::abs(dx)>std::abs(dy))
					{
						stepX = (dx<0.0f) ? 1.f : -1.f;
						stepY = -dy/fabsf(dx);
						stepLimit = (int)(fabsf(dx*advDistanceMult));
					}
					else
					{
						stepY = (dy<0.0f) ? 1.f : -1.f;
						stepX = -dx/fabsf(dy);
						stepLimit = (int)(fabsf(dy*advDistanceMult));
					}
					tx = float(x);
					ty = float(y);
					auto step = 0;
					for (; step<stepLimit; ++step)
					{
						tx += stepX;
						ty += stepY;
						if (bmap_blockair[(int)(ty+0.5f)][(int)(tx+0.5f)])
						{
							tx -= stepX;
							ty -= stepY;
							break;
						}
					}
					if (step==stepLimit)
					{
						// No wall found
						tx = x - dx*advDistanceMult;
						ty = y - dy*advDistanceMult;
					}
				}
				auto i = (int)tx;
				auto j = (int)ty;
				tx -= i;
				ty -= j;
				if (!bmap_blockair[y][x] && i>=2 && i<XCELLS-3 && j>=2 && j<YCELLS-3)
				{
					dx *= 1.0f - AIR_VADV;
					dy *= 1.0f - AIR_VADV;

					dx += AIR_VADV*(1.0f-tx)*(1.0f-ty)*vx[j][i];
					dy += AIR_VADV*(1.0f-tx)*(1.0f-ty)*vy[j][i];

					dx += AIR_VADV*tx*(1.0f-ty)*vx[j][i+1];
					dy += AIR_VADV*tx*(1.0f-ty)*vy[j][i+1];

					dx += AIR_VADV*(1.0f-tx)*ty*vx[j+1][i];
					dy += AIR_VADV*(1.0f-tx)*ty*vy[j+1][i];

					dx += AIR_VADV*tx*ty*vx[j+1][i+1];
					dy += AIR_VADV*tx*ty*vy[j+1][i+1];
				}*/

				if (bmap[y][x] == WL_FAN)
				{
					dx += fvx[y][x];
					dy += fvy[y][x];
				}

				// pressure caps
				if (std::isnan(dp)) dp = 0.0f;
				if (dp > MAX_PRESSURE) dp = MAX_PRESSURE;
				if (dp < -10.0f /*MIN_PRESSURE*/) dp = -10.0f /*MIN_PRESSURE*/;

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
	make_kernel();
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
