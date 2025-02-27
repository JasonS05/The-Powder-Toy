#ifndef AIR_SHADER_H
#define AIR_SHADER_H

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
    unsigned int XCELLS = XCELLS;
    unsigned int YCELLS = YCELLS;
    unsigned int dataSize = sizeof(CellData) / sizeof(float);
	unsigned int vxOffset = offsetof(CellData, vx) / sizeof(float);
	unsigned int vyOffset = offsetof(CellData, vy) / sizeof(float);
	unsigned int pvOffset = offsetof(CellData, pv) / sizeof(float);
	unsigned int hvOffset = offsetof(CellData, hv) / sizeof(float);
	unsigned int wallOffset = offsetof(CellData, wall) / sizeof(float);
	float dt = 0.1;
	float velocityCap = (0.5 / dt - 1.0) / sqrt(2.0);
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

    void init();
    void upload(Air * air);
	void run(int repetitions);
    void download(Air * air);

private:
    bool initialized = false;
    ComputeShader shader1, shader2, shader3;
    ConfigStruct config;
    unsigned int ssbo_out, ssbo_flux1, ssbo_flux2, ssbo_flux3, ssbo_flux4, ssbo_in, ssbo_config;
    CellData tmp_buf[XCELLS * YCELLS];
};


#endif
