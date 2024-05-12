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

struct ConfigStruct {
    unsigned int XCELLS = XCELLS;
    unsigned int YCELLS = YCELLS;
    unsigned int dataSize = sizeof(CellData) / sizeof(float);
	unsigned int vxOffset = offsetof(CellData, vx) / sizeof(float);
	unsigned int vyOffset = offsetof(CellData, vy) / sizeof(float);
	unsigned int pvOffset = offsetof(CellData, pv) / sizeof(float);
	unsigned int hvOffset = offsetof(CellData, hv) / sizeof(float);
	unsigned int wallOffset = offsetof(CellData, wall) / sizeof(float);
};

inline constexpr std::size_t BUFFER_SIZE = XCELLS * YCELLS * sizeof(CellData);

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
    ComputeShader shader;
    ConfigStruct config;
    unsigned int ssbo_out, ssbo_in, ssbo_config;
    CellData tmp_buf[XCELLS * YCELLS];
};


#endif
