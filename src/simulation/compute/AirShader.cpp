#include "AirShader.h"
#include "ComputeShader.h"
#include "../Simulation.h"
#include "../ElementCommon.h"
#include "../../SimulationConfig.h"
#include "../Air.h"

#include "Funcs.h"
#include <iostream>

using namespace Voxren::Gl;

AirShader::AirShader() {}

void AirShader::init() {
	if (initialized) return;

	// this shader computes the flux
	shader = std::move(ComputeShader(R"(
#version 430

// 16x16x1 has good performance on my laptop's GTX 1650
layout (local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

layout(std430, binding = 0) writeonly restrict buffer DataOutStruct {
	float dataOut[];
};

layout(std430, binding = 1) readonly restrict buffer DataInStruct {
	float dataIn[];
};

layout(std430, binding = 2) readonly restrict buffer FluxIn1Struct {
	vec4 fluxIn1[];
};

layout(std430, binding = 3) readonly restrict buffer FluxIn2Struct {
	vec4 fluxIn2[];
};

layout(std430, binding = 4) readonly restrict buffer FluxIn3Struct {
	vec4 fluxIn3[];
};

layout(std430, binding = 5) readonly restrict buffer FluxIn4Struct {
	vec4 fluxIn4[];
};

layout(std430, binding = 6) writeonly restrict buffer FluxOutStruct {
	vec4 fluxOut[];
};

layout(std430, binding = 7) readonly restrict buffer ConfigStruct {
	uint XCELLS;
	uint YCELLS;
	uint dataSize;
	uint vxOffset;
	uint vyOffset;
	uint pvOffset;
	uint hvOffset;
	uint wallOffset;
	float dt;
	float velocityCap;
	int degreesOfFreedom;
	float pressureScale;
	float MAX_TPT_PRESSURE;
	float MIN_TPT_PRESSURE;
};

layout(location = 1) uniform int function = 1; // varies from 1 to 4
layout(location = 2) uniform float dtMultiplier; // always either 0.5 or 1

layout(location = 3) uniform float ambientAirTemp;
layout(location = 4) uniform int aheat_enable;

struct CellData {
	float vx;
	float vy;
	float pv;
	float hv;
	bool wall;
};

// (momentum, mass, energy) -> (velocity, pressure, temperature)
vec4 fromConservedQuantities(vec4 a) {
	vec2 velocity = a.xy / a.z;
	float kineticEnergy = 0.5 * a.z * (velocity.x * velocity.x + velocity.y * velocity.y);
	float thermalEnergy = a.w - kineticEnergy;
	float temperature = thermalEnergy / a.z * (1.0 / float(degreesOfFreedom));

	if (aheat_enable == 0) {
		temperature = ambientAirTemp * (0.5 / 295.15);
	}

	float pressure = 2.0 * temperature * a.z;

	return vec4(velocity, pressure, temperature);
}

// (velocity, pressure, temperature) -> (momentum, mass, energy)
vec4 toConservedQuantities(vec4 a) {
	if (aheat_enable == 0) {
		a.w = ambientAirTemp * (0.5 / 295.15);
	}

	float mass = 0.5 * a.z / a.w;
	vec2 momentum = a.xy * mass;
	float kineticEnergy = 0.5 * mass * (a.x * a.x + a.y * a.y);
	float thermalEnergy = a.w * mass * float(degreesOfFreedom);
	float energy = kineticEnergy + thermalEnergy;

	return vec4(momentum, mass, energy);
}

vec4 toVec4(CellData data) {
	return vec4(data.vx, data.vy, data.pv, data.hv);
}

CellData toCellData(vec4 data, bool wall) {
	return CellData(data.x, data.y, data.z, data.w, wall);
}

float MAX_PRESSURE = exp(pressureScale * MAX_TPT_PRESSURE);
float MIN_PRESSURE = exp(pressureScale * MIN_TPT_PRESSURE);

const float DEFAULT_PRESSURE = 1.0;

CellData getCell(ivec2 pos, CellData data) {
	int x = pos.x;
	int y = pos.y;

	if (x >= 1 && x < XCELLS - 1 && y >= 1 && y < YCELLS - 1) {
		data.vx = dataIn[(y * XCELLS + x) * dataSize + vxOffset];
		data.vy = dataIn[(y * XCELLS + x) * dataSize + vyOffset];
		data.pv = dataIn[(y * XCELLS + x) * dataSize + pvOffset];
		data.hv = dataIn[(y * XCELLS + x) * dataSize + hvOffset];
		data.wall = dataIn[(y * XCELLS + x) * dataSize + wallOffset] != 0.0;
	} else {
		return data;
	}

	data.pv = clamp(data.pv, MIN_PRESSURE, MAX_PRESSURE);

	float velocity = length(vec2(data.vx, data.vy));

	if (velocity > velocityCap) {
		data.vx = data.vx / velocity * velocityCap;
		data.vy = data.vy / velocity * velocityCap;
	}

	if (aheat_enable == 0) {
		data.hv = ambientAirTemp;
	}

	data.hv = clamp(data.hv, 73.15, 9999.0);
	data.hv *= (0.5 / 295.15);

	return toCellData(toConservedQuantities(toVec4(data)), data.wall);
}

void setCell(ivec2 pos, CellData data) {
	int x = pos.x;
	int y = pos.y;

	data = toCellData(fromConservedQuantities(toVec4(data)), data.wall);

	data.pv = clamp(data.pv, MIN_PRESSURE, MAX_PRESSURE);

	float velocity = length(vec2(data.vx, data.vy));

	if (velocity > velocityCap) {
		data.vx = data.vx / velocity * velocityCap;
		data.vy = data.vy / velocity * velocityCap;
	}

	data.hv *= (295.15 / 0.5);
	data.hv = clamp(data.hv, 73.15, 9999.0);

	if (aheat_enable == 0) {
		data.hv = ambientAirTemp;
	}

	if (isnan(data.vx) || isnan(data.vy) || isnan(data.pv) || isnan(data.hv)) {
		data.vx = 0.0;
		data.vy = 0.0;
		data.pv = 0.0;
		data.hv = ambientAirTemp;
	}

	dataOut[(y * XCELLS + x) * dataSize + vxOffset] = data.vx;
	dataOut[(y * XCELLS + x) * dataSize + vyOffset] = data.vy;
	dataOut[(y * XCELLS + x) * dataSize + pvOffset] = data.pv;
	dataOut[(y * XCELLS + x) * dataSize + hvOffset] = data.hv;
	dataOut[(y * XCELLS + x) * dataSize + wallOffset] = data.wall ? 1.0 : 0.0;
}

const float PI  = 3.14159265359;
const float TAU = 2.0 * PI;

const float massConstant  = -0.5 * log(TAU);
const float massLinear    = sqrt(PI * 0.5);
const float massQuadratic = 0.5 - PI * 0.25;

const float momentumConstant  = -log(2.0);
const float momentumLinear    = 2.0 * sqrt(2.0 / PI);
const float momentumQuadratic = 1.0 - 4.0 / PI;

const float energyConstant  = -0.5 * log(TAU);
const float energyLinear    = 0.75 * sqrt(TAU);
const float energyQuadratic = 1.5 - 0.5625 * PI;

const vec3 constantFactor  = vec3(massConstant , momentumConstant , energyConstant );
const vec3 linearFactor    = vec3(massLinear   , momentumLinear   , energyLinear   );
const vec3 quadraticFactor = vec3(massQuadratic, momentumQuadratic, energyQuadratic);

vec3 approx_exp(vec3 x) {
	return 1.0 / (1.0 - x * (1.0 - x * (1.0 / 2.0 - x * (1.0 / 6.0 - x * (1.0 / 24.0 - x * (1.0 / 120.0))))));
}

vec4 getFlux(vec4 cell, float temperature) {
	float T = temperature * 2.0;
	float isqrtT = inversesqrt(T);
	float sqrtT = T * isqrtT;
	float velocity = cell.x / cell.z;
	float speed = abs(velocity);
	float x = speed * isqrtT;
	vec3 exponents = vec3(sqrtT, T, T * sqrtT) * approx_exp(constantFactor - x * (linearFactor - x * quadraticFactor));

	if (velocity > 0) {
		exponents = exponents * vec3(1, -1, 1) + vec3(velocity, velocity * velocity + T, (3.0 * T + velocity * velocity) * velocity * 0.5);
	}

	return exponents.yxxz * cell.zyzz + vec4(0, 0, 0, cell.z * exponents.x * (temperature * (degreesOfFreedom - 1) + 0.5 * cell.y * cell.y / (cell.z * cell.z)));
}

void computeFluxes() {
	ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
	int x = pos.x;
	int y = pos.y;

	if (x >= XCELLS || y >= YCELLS) {
		return;
	}

	vec4 upperFlux = vec4(0);
	vec4 lowerFlux = vec4(0);
	vec4 leftFlux  = vec4(0);
	vec4 rightFlux = vec4(0);

	CellData thisCellData = getCell(pos, toCellData(vec4(0), false));

	if (!thisCellData.wall) {
		vec4 thisCell = toVec4(thisCellData);

		CellData upperCellData = CellData(0, 0, 0, 0, false);
		CellData lowerCellData = CellData(0, 0, 0, 0, false);
		CellData leftCellData  = CellData(0, 0, 0, 0, false);
		CellData rightCellData = CellData(0, 0, 0, 0, false);

		vec4 upperEdge;
		vec4 lowerEdge;
		vec4 leftEdge;
		vec4 rightEdge;

		if (x != 0 && x != XCELLS - 1 && y != 0 && y != YCELLS - 1) {
			upperCellData = getCell(pos + ivec2( 0, -1), thisCellData);
			lowerCellData = getCell(pos + ivec2( 0,  1), thisCellData);
			leftCellData  = getCell(pos + ivec2(-1,  0), thisCellData);
			rightCellData = getCell(pos + ivec2( 1,  0), thisCellData);

			vec4 upperCell = fromConservedQuantities(upperCellData.wall ? thisCell * vec4(1, -1, 1, 1) : toVec4(upperCellData));
			vec4 lowerCell = fromConservedQuantities(lowerCellData.wall ? thisCell * vec4(1, -1, 1, 1) : toVec4(lowerCellData));
			vec4 leftCell  = fromConservedQuantities(leftCellData.wall  ? thisCell * vec4(-1, 1, 1, 1) : toVec4(leftCellData ));
			vec4 rightCell = fromConservedQuantities(rightCellData.wall ? thisCell * vec4(-1, 1, 1, 1) : toVec4(rightCellData));
			vec4 thisCell2 = fromConservedQuantities(thisCell);

			vec4 logUpperCell = vec4(upperCell.xy, log(upperCell.zw));
			vec4 logLowerCell = vec4(lowerCell.xy, log(lowerCell.zw));
			vec4 logLeftCell  = vec4(leftCell .xy, log(leftCell .zw));
			vec4 logRightCell = vec4(rightCell.xy, log(rightCell.zw));
			vec4 logThisCell  = vec4(thisCell2.xy, log(thisCell2.zw));

			vec4 logUpperEdge = logThisCell + (logUpperCell - logLowerCell) * 0.25;
			vec4 logLowerEdge = logThisCell + (logLowerCell - logUpperCell) * 0.25;
			vec4 logLeftEdge  = logThisCell + (logLeftCell  - logRightCell) * 0.25;
			vec4 logRightEdge = logThisCell + (logRightCell - logLeftCell ) * 0.25;

			upperEdge = vec4(logUpperEdge.xy, exp(logUpperEdge.zw));
			lowerEdge = vec4(logLowerEdge.xy, exp(logLowerEdge.zw));
			leftEdge  = vec4(logLeftEdge .xy, exp(logLeftEdge .zw));
			rightEdge = vec4(logRightEdge.xy, exp(logRightEdge.zw));
		} else {
			upperEdge = vec4(0, 0, DEFAULT_PRESSURE, ambientAirTemp * (0.5 / 295.15));
			lowerEdge = vec4(0, 0, DEFAULT_PRESSURE, ambientAirTemp * (0.5 / 295.15));
			leftEdge  = vec4(0, 0, DEFAULT_PRESSURE, ambientAirTemp * (0.5 / 295.15));
			rightEdge = vec4(0, 0, DEFAULT_PRESSURE, ambientAirTemp * (0.5 / 295.15));
		}

		vec4 temperature = vec4(upperEdge.w, lowerEdge.w, leftEdge.w, rightEdge.w);

		upperEdge = toConservedQuantities(upperEdge);
		lowerEdge = toConservedQuantities(lowerEdge);
		leftEdge  = toConservedQuantities(leftEdge );
		rightEdge = toConservedQuantities(rightEdge);

		upperFlux = getFlux(upperEdge.yxzw                    , temperature.x).yxzw                       ;
		lowerFlux = getFlux(lowerEdge.yxzw * vec4(-1, 1, 1, 1), temperature.y).yxzw * vec4(-1,  1, -1, -1);
		leftFlux  = getFlux(leftEdge       * vec4(-1, 1, 1, 1), temperature.z)      * vec4( 1, -1, -1, -1);
		rightFlux = getFlux(rightEdge                         , temperature.w)                            ;

		if (upperCellData.wall) {
			upperFlux *= vec4(0.0, 2.0, 0.0, 0.0);
		}

		if (lowerCellData.wall) {
			lowerFlux *= vec4(0.0, 2.0, 0.0, 0.0);
		}

		if (leftCellData.wall) {
			leftFlux *= vec4(2.0, 0.0, 0.0, 0.0);
		}

		if (rightCellData.wall) {
			rightFlux *= vec4(2.0, 0.0, 0.0, 0.0);
		}
	} else {
		upperFlux = vec4(0);
		lowerFlux = vec4(0);
		leftFlux  = vec4(0);
		rightFlux = vec4(0);
	}

	fluxOut[(y * XCELLS + x) * 4 + 0] = upperFlux;
	fluxOut[(y * XCELLS + x) * 4 + 1] = lowerFlux;
	fluxOut[(y * XCELLS + x) * 4 + 2] = leftFlux;
	fluxOut[(y * XCELLS + x) * 4 + 3] = rightFlux;
}

void eulerIntegrate() {
	ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
	int x = pos.x;
	int y = pos.y;

	if (x >= XCELLS || y >= YCELLS) {
		return;
	}

	CellData thisCellData = getCell(pos, CellData(0, 0, 0, 0, false));
	vec4 thisCell = toVec4(thisCellData);

	if (!thisCellData.wall && x != 0 && x != XCELLS - 1 && y != 0 && y != YCELLS - 1) {
		int upperY = y - 1;
		int lowerY = y + 1;
		int leftX  = x - 1;
		int rightX = x + 1;

		vec4 upperFlux = fluxIn1[(y * XCELLS + x) * 4 + 0] + fluxIn1[(upperY * XCELLS + x     ) * 4 + 1];
		vec4 lowerFlux = fluxIn1[(y * XCELLS + x) * 4 + 1] + fluxIn1[(lowerY * XCELLS + x     ) * 4 + 0];
		vec4 leftFlux  = fluxIn1[(y * XCELLS + x) * 4 + 2] + fluxIn1[(y      * XCELLS + leftX ) * 4 + 3];
		vec4 rightFlux = fluxIn1[(y * XCELLS + x) * 4 + 3] + fluxIn1[(y      * XCELLS + rightX) * 4 + 2];

		thisCell += (leftFlux + lowerFlux - rightFlux - upperFlux) * dt * dtMultiplier;
	} else {
		thisCell = toConservedQuantities(vec4(0, 0, DEFAULT_PRESSURE, ambientAirTemp * (0.5 / 295.15)));
	}

	setCell(pos, toCellData(thisCell, thisCellData.wall));
}

float third = 1.0 / 3.0;
float sixth = 1.0 / 6.0;

void RK4Integrate() {
	ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
	int x = pos.x;
	int y = pos.y;

	if (x >= XCELLS || y >= YCELLS) {
		return;
	}

	CellData thisCellData = getCell(pos, CellData(0, 0, 0, 0, false));
	vec4 thisCell = toVec4(thisCellData);

	if (!thisCellData.wall && x != 0 && x != XCELLS - 1 && y != 0 && y != YCELLS - 1) {
		int upperY = y - 1;
		int lowerY = y + 1;
		int leftX  = x - 1;
		int rightX = x + 1;

		// Runge-Kutta 4

		vec4 upperFlux = vec4(0);
		vec4 lowerFlux = vec4(0);
		vec4 leftFlux  = vec4(0);
		vec4 rightFlux = vec4(0);

		upperFlux += (fluxIn1[(y * XCELLS + x) * 4 + 0] + fluxIn1[(upperY * XCELLS + x     ) * 4 + 1]) * sixth;
		lowerFlux += (fluxIn1[(y * XCELLS + x) * 4 + 1] + fluxIn1[(lowerY * XCELLS + x     ) * 4 + 0]) * sixth;
		leftFlux  += (fluxIn1[(y * XCELLS + x) * 4 + 2] + fluxIn1[(y      * XCELLS + leftX ) * 4 + 3]) * sixth;
		rightFlux += (fluxIn1[(y * XCELLS + x) * 4 + 3] + fluxIn1[(y      * XCELLS + rightX) * 4 + 2]) * sixth;

		upperFlux += (fluxIn2[(y * XCELLS + x) * 4 + 0] + fluxIn2[(upperY * XCELLS + x     ) * 4 + 1]) * third;
		lowerFlux += (fluxIn2[(y * XCELLS + x) * 4 + 1] + fluxIn2[(lowerY * XCELLS + x     ) * 4 + 0]) * third;
		leftFlux  += (fluxIn2[(y * XCELLS + x) * 4 + 2] + fluxIn2[(y      * XCELLS + leftX ) * 4 + 3]) * third;
		rightFlux += (fluxIn2[(y * XCELLS + x) * 4 + 3] + fluxIn2[(y      * XCELLS + rightX) * 4 + 2]) * third;

		upperFlux += (fluxIn3[(y * XCELLS + x) * 4 + 0] + fluxIn3[(upperY * XCELLS + x     ) * 4 + 1]) * third;
		lowerFlux += (fluxIn3[(y * XCELLS + x) * 4 + 1] + fluxIn3[(lowerY * XCELLS + x     ) * 4 + 0]) * third;
		leftFlux  += (fluxIn3[(y * XCELLS + x) * 4 + 2] + fluxIn3[(y      * XCELLS + leftX ) * 4 + 3]) * third;
		rightFlux += (fluxIn3[(y * XCELLS + x) * 4 + 3] + fluxIn3[(y      * XCELLS + rightX) * 4 + 2]) * third;

		upperFlux += (fluxIn4[(y * XCELLS + x) * 4 + 0] + fluxIn4[(upperY * XCELLS + x     ) * 4 + 1]) * sixth;
		lowerFlux += (fluxIn4[(y * XCELLS + x) * 4 + 1] + fluxIn4[(lowerY * XCELLS + x     ) * 4 + 0]) * sixth;
		leftFlux  += (fluxIn4[(y * XCELLS + x) * 4 + 2] + fluxIn4[(y      * XCELLS + leftX ) * 4 + 3]) * sixth;
		rightFlux += (fluxIn4[(y * XCELLS + x) * 4 + 3] + fluxIn4[(y      * XCELLS + rightX) * 4 + 2]) * sixth;

		thisCell += (leftFlux + lowerFlux - rightFlux - upperFlux) * dt;
	} else {
		thisCell = toConservedQuantities(vec4(0, 0, DEFAULT_PRESSURE, ambientAirTemp * (0.5 / 295.15)));
	}

	setCell(pos, toCellData(thisCell, thisCellData.wall));
}

void main() {
	switch (function) {
	case 1:
		computeFluxes();
		break;

	case 2:
		eulerIntegrate();
		break;

	case 3:
		RK4Integrate();
		break;
	}
}
	)"));

	glGenBuffers(1, &ssbo_in);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_in);
	glBufferData(GL_SHADER_STORAGE_BUFFER, CELL_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &ssbo_flux1);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_flux1);
	glBufferData(GL_SHADER_STORAGE_BUFFER, FLUX_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &ssbo_flux2);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_flux2);
	glBufferData(GL_SHADER_STORAGE_BUFFER, FLUX_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &ssbo_flux3);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_flux3);
	glBufferData(GL_SHADER_STORAGE_BUFFER, FLUX_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &ssbo_flux4);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_flux4);
	glBufferData(GL_SHADER_STORAGE_BUFFER, FLUX_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &ssbo_flux);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_flux);
	glBufferData(GL_SHADER_STORAGE_BUFFER, FLUX_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &ssbo_out);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_out);
	glBufferData(GL_SHADER_STORAGE_BUFFER, CELL_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &ssbo_config);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_config);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(ConfigStruct), &config, GL_STATIC_DRAW);
	// Use glBufferSubData to update existing data it's faster

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	initialized = true;
}

AirShader::~AirShader() {
	glDeleteBuffers(1, &ssbo_in);
	glDeleteBuffers(1, &ssbo_flux1);
	glDeleteBuffers(1, &ssbo_flux2);
	glDeleteBuffers(1, &ssbo_flux3);
	glDeleteBuffers(1, &ssbo_flux4);
	glDeleteBuffers(1, &ssbo_flux);
	glDeleteBuffers(1, &ssbo_out);
	glDeleteBuffers(1, &ssbo_config);
}

// these structs should be enums but I don't want to pollute the namespace and "enum class" (or "enum struct") syntax requires I cast to unsigned int everywhere
struct BUFFER_BINDING final {
	static const unsigned int dataOut = 0;
	static const unsigned int dataIn  = 1;
	static const unsigned int fluxIn1 = 2;
	static const unsigned int fluxIn2 = 3;
	static const unsigned int fluxIn3 = 4;
	static const unsigned int fluxIn4 = 5;
	static const unsigned int fluxOut = 6;
	static const unsigned int config  = 7;
};

struct UNIFORM_BINDING final {
	static const unsigned int function       = 1;
	static const unsigned int dtMultiplier   = 2;
	static const unsigned int ambientAirTemp = 3;
	static const unsigned int aheat_enable   = 4;
};

struct SHADER_FUNCTION final {
	static const int computeFluxes  = 1; // uses dataIn, fluxOut
	static const int eulerIntegrate = 2; // uses dataOut, dataIn, fluxIn1, dtMultiplier
	static const int RK4Integrate   = 3; // uses dataOut, dataIn, fluxIn1, fluxIn2, fluxIn3, fluxIn4
};

void AirShader::run(int repetitions, Air *air) {
	shader.enable();
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::config, ssbo_config);
	glUniform1f(UNIFORM_BINDING::ambientAirTemp, std::max(air->ambientAirTemp, 73.15f));
	glUniform1i(UNIFORM_BINDING::aheat_enable, air->sim.aheat_enable);

	for (int i = 0; i < repetitions; i++) {
		// Runge-Kutta 4

		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataIn, ssbo_in);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxOut, ssbo_flux1);
		glUniform1i(UNIFORM_BINDING::function, SHADER_FUNCTION::computeFluxes);
		shader.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);

		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataOut, ssbo_out);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataIn, ssbo_in);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxIn1, ssbo_flux1);
		glUniform1i(UNIFORM_BINDING::function, SHADER_FUNCTION::eulerIntegrate);
		glUniform1f(UNIFORM_BINDING::dtMultiplier, 0.5f);
		shader.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);

		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataIn, ssbo_out);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxOut, ssbo_flux2);
		glUniform1i(UNIFORM_BINDING::function, SHADER_FUNCTION::computeFluxes);
		shader.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);

		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataOut, ssbo_out);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataIn, ssbo_in);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxIn1, ssbo_flux2);
		glUniform1i(UNIFORM_BINDING::function, SHADER_FUNCTION::eulerIntegrate);
		glUniform1f(UNIFORM_BINDING::dtMultiplier, 0.5f);
		shader.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);

		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataIn, ssbo_out);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxOut, ssbo_flux3);
		glUniform1i(UNIFORM_BINDING::function, SHADER_FUNCTION::computeFluxes);
		shader.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);

		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataOut, ssbo_out);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataIn, ssbo_in);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxIn1, ssbo_flux3);
		glUniform1i(UNIFORM_BINDING::function, SHADER_FUNCTION::eulerIntegrate);
		glUniform1f(UNIFORM_BINDING::dtMultiplier, 1.0f);
		shader.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);

		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataIn, ssbo_out);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxOut, ssbo_flux4);
		glUniform1i(UNIFORM_BINDING::function, SHADER_FUNCTION::computeFluxes);
		shader.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);

		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataOut, ssbo_out);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::dataIn, ssbo_in);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxIn1, ssbo_flux1);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxIn2, ssbo_flux2);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxIn3, ssbo_flux3);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BUFFER_BINDING::fluxIn4, ssbo_flux4);
		glUniform1i(UNIFORM_BINDING::function, SHADER_FUNCTION::RK4Integrate);
		shader.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);

		auto temp = ssbo_out;
		ssbo_out = ssbo_in;
		ssbo_in = temp;
	}

	shader.disable();
}

void AirShader::upload(Simulation &sim, Air *air) {
	for (int y = 0; y < YCELLS; y++) {
		for (int x = 0; x < XCELLS; x++) {
			tmp_buf[y * XCELLS + x].vx = sim.vx[y][x] / 1.6;
			tmp_buf[y * XCELLS + x].vy = -sim.vy[y][x] / 1.6;
			tmp_buf[y * XCELLS + x].pv = std::exp(sim.pv[y][x] * config.pressureScale);
			tmp_buf[y * XCELLS + x].hv = sim.hv[y][x];
			tmp_buf[y * XCELLS + x].wall = air->bmap_blockair[y][x];
		}
	}

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_in);
	glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, CELL_BUFFER_SIZE, &tmp_buf[0]);
}

void AirShader::download(Simulation &sim) {
	glMemoryBarrier(GL_ALL_BARRIER_BITS);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_in);

	// Read ssbo into tmp_buf
	glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, CELL_BUFFER_SIZE, &tmp_buf[0]);

	for (int y = 0; y < YCELLS; y++) {
		for (int x = 0; x < XCELLS; x++) {
			sim.vx[y][x] = tmp_buf[y * XCELLS + x].vx * 1.6;
			sim.vy[y][x] = -tmp_buf[y * XCELLS + x].vy * 1.6;
			sim.pv[y][x] = std::log(tmp_buf[y * XCELLS + x].pv) / config.pressureScale;
			sim.hv[y][x] = tmp_buf[y * XCELLS + x].hv;
		}
	}
}
