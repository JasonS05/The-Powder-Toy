#include "ComputeShader.h"
#include "Funcs.h"
#include <string>
#include <stdexcept>
#include <iostream>

using namespace Voxren::Gl;

ComputeShader::ComputeShader(const char * programText) {
    unsigned int shader = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(shader, 1, &programText, NULL);

    GLint success = 0;
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

    if (success == GL_FALSE) {
        std::string err_msg = "Shader: [ID " + std::to_string(shader) + "] Failed to compile compute shader code\n";

        int maxLength = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);

        if (maxLength > 0) {
            int length = 0;
            char * log = new char[maxLength];
            glGetShaderInfoLog(shader, maxLength, &length, log);
            err_msg += log;
            delete[] log;
        }
        throw std::runtime_error(err_msg);
    }

    unsigned int program = 0;
    success = 0;
    program = glCreateProgram();
    glAttachShader(program, shader);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &success);

    if (success == GL_FALSE) {
        std::string err_msg = "Shader: [ID " + std::to_string(shader) + "] Failed to link compute shader program\n";

        int maxLength = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &maxLength);

        if (maxLength > 0) {
            int length = 0;
            char * log = new char[maxLength];
            glGetProgramInfoLog(program, maxLength, &length, log);
            err_msg += "[Program ID " + std::to_string(program) + "] Link error: " + log;
            delete[] log;
        }
        glDeleteProgram(program);
        program = 0;
        throw std::runtime_error(err_msg);
    }

    this->id = program;
    this->initialized = true;
}

ComputeShader& ComputeShader::operator=(ComputeShader &&other) {
    if (this != &other) {
        std::swap(initialized, other.initialized);
        std::swap(id, other.id);
    }
    return *this;
}

ComputeShader::~ComputeShader() {
    if (initialized)
        glDeleteProgram(id);
}

void ComputeShader::enable() {
    glUseProgram(id);
}

void ComputeShader::dispatch(unsigned int xGroups, unsigned int yGroups, unsigned int zGroups) {
    glDispatchCompute(xGroups, yGroups, zGroups);
}

void ComputeShader::disable() {
    glUseProgram(0);
}

