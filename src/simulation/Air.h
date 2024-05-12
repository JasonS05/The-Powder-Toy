#pragma once
#include "SimulationConfig.h"
#include "compute/AirShader.h"

class Simulation;

class Air
{
public:
	AirShader air_shader;

	Simulation & sim;
	int airMode;
	float ambientAirTemp;
	float ovx[YCELLS][XCELLS];
	float ovy[YCELLS][XCELLS];
	float opv[YCELLS][XCELLS];
	float ohv[YCELLS][XCELLS]; // Ambient Heat
	bool ec[YCELLS][XCELLS]; // extremal cells (cells with values greater than their neighbors)
	unsigned char bmap_blockair[YCELLS][XCELLS];
	unsigned char bmap_blockairh[YCELLS][XCELLS];
	void make_kernel(void);
	void update_airh(void);
	void update_air(void);
	void Clear();
	void ClearAirH();
	void Invert();
	void ApproximateBlockAirMaps();
	Air(Simulation & sim);
};
