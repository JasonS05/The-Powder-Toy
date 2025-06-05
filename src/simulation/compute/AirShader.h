#ifndef AIR_SHADER_H
#define AIR_SHADER_H

#include "../Simulation.h"
#include "ComputeShader.h"
#include "../../SimulationConfig.h"
#include <cstddef>

struct CellData {
	float vx;
	float vy;
	float pv;
	float hv;
	float wall; // ought to be bool but the shader expects everything to be floats
};

struct Vec4 {
	float x;
	float y;
	float z;
	float w;
};

struct ConfigStruct {
	unsigned int XCELLS = ::XCELLS;
	unsigned int YCELLS = ::YCELLS;
	unsigned int dataSize = sizeof(CellData) / sizeof(float);
	unsigned int vxOffset = offsetof(CellData, vx) / sizeof(float);
	unsigned int vyOffset = offsetof(CellData, vy) / sizeof(float);
	unsigned int pvOffset = offsetof(CellData, pv) / sizeof(float);
	unsigned int hvOffset = offsetof(CellData, hv) / sizeof(float);
	unsigned int wallOffset = offsetof(CellData, wall) / sizeof(float);
	float dt = 0.05;
	float velocityCap = 5.0; // 6 is too high, 5 seems stable
	int degreesOfFreedom = 5; // determines the adiabatic index according to (n + 2) / n, in this case it's 1.4 which matches IRL air
	float pressureScale = std::log(50.0) / MAX_PRESSURE; // determines the exponential relationship between TPT pressure and simulation pressure, larger number means more extreme pressure swings for the same change in TPT pressure
	float MAX_TPT_PRESSURE = MAX_PRESSURE;
	float MIN_TPT_PRESSURE = MIN_PRESSURE;
};

inline constexpr std::size_t CELL_BUFFER_SIZE = XCELLS * YCELLS * sizeof(CellData);
inline constexpr std::size_t FLUX_BUFFER_SIZE = XCELLS * YCELLS * 4 * sizeof(Vec4);

class Air;
class AirShader {
public:
	AirShader();
	~AirShader();

	AirShader(const AirShader &other) = delete;
	AirShader &operator=(const AirShader &other) = delete;

	const ConfigStruct config;

	void init();
	void run(int repetitions, Air *air);
	void upload(Simulation &sim, Air *air);
	void download(Simulation &sim);

private:
	bool initialized = false;
	ComputeShader shader;
	unsigned int ssbo_out, ssbo_flux1, ssbo_flux2, ssbo_flux3, ssbo_flux4, ssbo_in, ssbo_config;
	CellData tmp_buf[XCELLS * YCELLS];
};


#endif
