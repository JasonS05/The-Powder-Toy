#include "AirShader.h"
#include "ComputeShader.h"
#include "../Simulation.h"
#include "../ElementCommon.h"
#include "../../SimulationConfig.h"
#include "../Air.h"

#include <GL/glew.h>
#include <iostream>

AirShader::AirShader() {}

void AirShader::init() {
	if (initialized) return;

	shader1 = std::move(ComputeShader(R"(
#version 430

// 16x16x1 has good performance on my laptop's GTX 1650
layout (local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

layout(std430, binding = 0) writeonly restrict buffer DataFluxStruct {
	vec4 dataFlux[];
};

layout(std430, binding = 1) readonly restrict buffer DataInStruct {
	float dataIn[];
};

layout(std430, binding = 2) readonly restrict buffer ConfigStruct {
	uint XCELLS;
	uint YCELLS;
	uint dataSize;
	uint vxOffset;
	uint vyOffset;
	uint pvOffset;
	uint hvOffset;
	uint wallOffset;
};

struct CellData {
	float vx;
	float vy;
	float pv;
	float hv;
	bool wall;
};

const float dt = 0.1;
const vec4 diffusion = vec4(1.0) * 0.1;

CellData getCell(ivec2 pos) {
	CellData data = CellData(0.0, 0.0, 0.0, 0.0, false);
	int x = pos.x;
	int y = pos.y;

	if (x >= 0 && x < XCELLS && y >= 0 && y < YCELLS) {
		data.vx = dataIn[(y * XCELLS + x) * dataSize + vxOffset];
		data.vy = dataIn[(y * XCELLS + x) * dataSize + vyOffset];
		data.pv = dataIn[(y * XCELLS + x) * dataSize + pvOffset];
		data.hv = dataIn[(y * XCELLS + x) * dataSize + hvOffset];
		data.wall = dataIn[(y * XCELLS + x) * dataSize + wallOffset] != 0.0;
	}

	data.pv = clamp(data.pv + 10.1, 0.1, 266.1);

	return data;
}

vec4 toVec4(CellData data) {
	return vec4(data.vx, data.vy, data.pv, data.hv);
}

CellData toCellData(vec4 data, bool wall) {
	return CellData(data.x, data.y, data.z, data.w, wall);
}

vec4 getFluxH(vec4 cell) {
	vec4 momentumFlux = vec4(cell.z, 0.0, 0.0, 0.0);

	vec4 advectionFlux = cell * cell.x / cell.z;

	return momentumFlux + advectionFlux;
}

vec4 computeEdgeFluxH(vec4 leftCell, vec4 rightCell) {
	vec2 averageVelocity = (leftCell.xy / leftCell.z + rightCell.xy / rightCell.z) * 0.5;
	float averageDensity = sqrt(leftCell.z * rightCell.z);

	vec4 average = vec4(averageVelocity * averageDensity, averageDensity, 0.0);
	average += dt * (getFluxH(leftCell) - getFluxH(rightCell)) * 0.5;

	average.z = max(average.z, 0.1);

	vec4 diffusionFlux = (leftCell - rightCell) * diffusion;//(leftCell - rightCell) * (1.0 + length(average.xy / average.z)) * 0.5;

	return diffusionFlux + getFluxH(average);//momentumFlux + advectionFlux;
}

vec4 computeEdgeFluxV(vec4 lowerCell, vec4 upperCell) {
	return computeEdgeFluxH(lowerCell.yxzw, upperCell.yxzw).yxzw;
}

void main() {
	ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
	int x = pos.x;
	int y = pos.y;

	if (x >= XCELLS || y >= YCELLS) {
		return;
	}

	vec4 upperFlux = vec4(0);
	vec4 lowerFlux = vec4(0);
	vec4 leftFlux = vec4(0);
	vec4 rightFlux = vec4(0);

	CellData thisCellData = getCell(pos);

	if (!thisCellData.wall) {
		vec4 thisCell = toVec4(thisCellData);

		CellData upperCellData = getCell(pos + ivec2( 0, -1));
		CellData lowerCellData = getCell(pos + ivec2( 0,  1));
		CellData leftCellData  = getCell(pos + ivec2(-1,  0));
		CellData rightCellData = getCell(pos + ivec2( 1,  0));

		vec4 upperCell = upperCellData.wall ? thisCell : toVec4(upperCellData);
		vec4 lowerCell = lowerCellData.wall ? thisCell : toVec4(lowerCellData);
		vec4 leftCell  = leftCellData.wall  ? thisCell : toVec4(leftCellData);
		vec4 rightCell = rightCellData.wall ? thisCell : toVec4(rightCellData);

		upperFlux = computeEdgeFluxV(thisCell , upperCell);
		lowerFlux = computeEdgeFluxV(lowerCell, thisCell );
		leftFlux  = computeEdgeFluxH(leftCell , thisCell );
		rightFlux = computeEdgeFluxH(thisCell , rightCell);
	}

	dataFlux[(y * XCELLS + x) * 4 + 0] = upperFlux * 0.5;
	dataFlux[(y * XCELLS + x) * 4 + 1] = lowerFlux * 0.5;
	dataFlux[(y * XCELLS + x) * 4 + 2] = leftFlux * 0.5;
	dataFlux[(y * XCELLS + x) * 4 + 3] = rightFlux * 0.5;
}
	)"));

	shader2 = std::move(ComputeShader(R"(
#version 430

layout (local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

layout(std430, binding = 0) writeonly restrict buffer DataOutStruct {
	float dataOut[];
};

layout(std430, binding = 1) readonly restrict buffer DataInStruct {
	float dataIn[];
};

layout(std430, binding = 2) readonly restrict buffer DataFluxStruct {
	vec4 dataFlux[];
};

layout(std430, binding = 3) readonly restrict buffer ConfigStruct {
	uint XCELLS;
	uint YCELLS;
	uint dataSize;
	uint vxOffset;
	uint vyOffset;
	uint pvOffset;
	uint hvOffset;
	uint wallOffset;
};

struct CellData {
	float vx;
	float vy;
	float pv;
	float hv;
	bool wall;
};

const float dt = 0.1;

CellData getCell(ivec2 pos) {
	CellData data = CellData(0.0, 0.0, 0.0, 0.0, false);
	int x = pos.x;
	int y = pos.y;

	if (x >= 0 && x < XCELLS && y >= 0 && y < YCELLS) {
		data.vx = dataIn[(y * XCELLS + x) * dataSize + vxOffset];
		data.vy = dataIn[(y * XCELLS + x) * dataSize + vyOffset];
		data.pv = dataIn[(y * XCELLS + x) * dataSize + pvOffset];
		data.hv = dataIn[(y * XCELLS + x) * dataSize + hvOffset];
		data.wall = dataIn[(y * XCELLS + x) * dataSize + wallOffset] != 0.0;
	}

	data.pv = clamp(data.pv + 10.1, 0.1, 266.1);

	return data;
}

void setCell(ivec2 pos, CellData data) {
	int x = pos.x;
	int y = pos.y;

	float velocity = length(vec2(data.vx, data.vy)) / data.pv;
	float maxVelocity = (0.5 / dt - 1.0) / sqrt(2.0);

	if (velocity > maxVelocity) {
		data.vx = data.vx / velocity * maxVelocity;
		data.vy = data.vy / velocity * maxVelocity;
	}

	data.pv = clamp(data.pv - 10.1, -10.0, 256.0);

	if (isnan(data.vx)) {
		data.vx = 0.0;
	}

	if (isnan(data.vy)) {
		data.vy = 0.0;
	}

	if (isnan(data.pv)) {
		data.pv = 0.0;
	}

	if (isnan(data.hv)) {
		data.hv = 0.0;
	}

	dataOut[(y * XCELLS + x) * dataSize + vxOffset] = data.vx;
	dataOut[(y * XCELLS + x) * dataSize + vyOffset] = data.vy;
	dataOut[(y * XCELLS + x) * dataSize + pvOffset] = data.pv;
	dataOut[(y * XCELLS + x) * dataSize + hvOffset] = data.hv;
	dataOut[(y * XCELLS + x) * dataSize + wallOffset] = data.wall ? 1.0 : 0.0;
}

vec4 toVec4(CellData data) {
	return vec4(data.vx, data.vy, data.pv, data.hv);
}

CellData toCellData(vec4 data, bool wall) {
	return CellData(data.x, data.y, data.z, data.w, wall);
}

void main() {
	ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
	int x = pos.x;
	int y = pos.y;

	if (x >= XCELLS || y >= YCELLS) {
		return;
	}

	CellData thisCellData = getCell(pos);
	vec4 thisCell = toVec4(thisCellData);

	if (!thisCellData.wall) {
		int upperY = y - 1;
		int lowerY = y + 1;
		int leftX  = x - 1;
		int rightX = x + 1;

		if (upperY == -1) {
			upperY++;
		}

		if (lowerY == YCELLS) {
			lowerY--;
		}

		if (leftX == -1) {
			leftX++;
		}

		if (rightX == XCELLS) {
			rightX--;
		}

		if (x >= XCELLS || y >= YCELLS) {
			return;
		}

		CellData upperCellData = getCell(ivec2(x     , upperY));
		CellData lowerCellData = getCell(ivec2(x     , lowerY));
		CellData leftCellData  = getCell(ivec2(leftX , y     ));
		CellData rightCellData = getCell(ivec2(rightX, y     ));

		vec4 upperFlux = dataFlux[(y * XCELLS + x) * 4 + 0];
		vec4 lowerFlux = dataFlux[(y * XCELLS + x) * 4 + 1];
		vec4 leftFlux  = dataFlux[(y * XCELLS + x) * 4 + 2];
		vec4 rightFlux = dataFlux[(y * XCELLS + x) * 4 + 3];

		if (!upperCellData.wall) {
			upperFlux += dataFlux[(upperY * XCELLS + x) * 4 + 1];
		} else {
			upperFlux *= 2.0;
		}

		if (!lowerCellData.wall) {
			lowerFlux += dataFlux[(lowerY * XCELLS + x) * 4 + 0];
		} else {
			lowerFlux *= 2.0;
		}

		if (!leftCellData.wall) {
			leftFlux += dataFlux[(y * XCELLS + leftX) * 4 + 3];
		} else {
			leftFlux *= 2.0;
		}

		if (!rightCellData.wall) {
			rightFlux += dataFlux[(y * XCELLS + rightX) * 4 + 2];
		} else {
			rightFlux *= 2.0;
		}

		thisCell += (leftFlux + lowerFlux - rightFlux - upperFlux) * dt;
	}

	setCell(pos, toCellData(thisCell, thisCellData.wall));
}
	)"));

	glGenBuffers(1, &ssbo_in);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_in);
	glBufferData(GL_SHADER_STORAGE_BUFFER, CELL_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &ssbo_flux);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_flux);
	glBufferData(GL_SHADER_STORAGE_BUFFER, FLUX_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &ssbo_out);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_out);
	glBufferData(GL_SHADER_STORAGE_BUFFER, CELL_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);

	config.XCELLS = XCELLS;
	config.YCELLS = YCELLS;

	glGenBuffers(1, &ssbo_config);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_config);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(ConfigStruct), &config, GL_STATIC_DRAW);
	// Use glBufferSubData to update existing data it's faster

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	initialized = true;
}

AirShader::~AirShader() {
	glDeleteBuffers(1, &ssbo_in);
	glDeleteBuffers(1, &ssbo_out);
	glDeleteBuffers(1, &ssbo_config);
}

void AirShader::upload(Air * air) {
	for (int y = 0; y < YCELLS; y++) {
		for (int x = 0; x < XCELLS; x++) {
			tmp_buf[y * XCELLS + x].vx =  (air->vx)[y][x];
			tmp_buf[y * XCELLS + x].vy = -(air->vy)[y][x];
			tmp_buf[y * XCELLS + x].pv =  (air->pv)[y][x];
			tmp_buf[y * XCELLS + x].hv =  (air->hv)[y][x];
			tmp_buf[y * XCELLS + x].wall = (air->bmap_blockair)[y][x];
		}
	}

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_in);
	glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, CELL_BUFFER_SIZE, &tmp_buf[0]);
}

void AirShader::run(int repetitions) {
	for (int i = 0; i < repetitions; i++) {
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo_flux); // Index, id
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ssbo_in); // Index, id
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, ssbo_config); // Index, id

		shader1.enable();
		shader1.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);
		shader1.disable();

		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo_out); // Index, id
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ssbo_in); // Index, id
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, ssbo_flux); // Index, id
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssbo_config); // Index, id

		shader2.enable();
		shader2.dispatch((XCELLS - 1) / 16 + 1, (YCELLS - 1) / 16 + 1, 1);
		shader2.disable();

		auto temp = ssbo_out;
		ssbo_out = ssbo_in;
		ssbo_in = temp;
	}
}

void AirShader::download(Air * air) {
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_in);

	// Read ssbo into tmp_buf
	glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, CELL_BUFFER_SIZE, &tmp_buf[0]);

	for (int y = 0; y < YCELLS; y++) {
		for (int x = 0; x < XCELLS; x++) {
			(air->vx)[y][x] =  tmp_buf[y * XCELLS + x].vx;
			(air->vy)[y][x] = -tmp_buf[y * XCELLS + x].vy;
			(air->pv)[y][x] =  tmp_buf[y * XCELLS + x].pv;
			(air->hv)[y][x] =  tmp_buf[y * XCELLS + x].hv;
		}
	}
}
