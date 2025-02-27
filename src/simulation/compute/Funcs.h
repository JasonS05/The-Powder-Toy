#pragma once
#include <SDL_opengl.h>

#define VOXREN_GLFUNC_LIST(X) \
	X(void, glAttachShader, GLuint program, GLuint shader) \
	X(void, glBindBufferBase, GLenum target, GLuint index, GLuint buffer) \
	X(void, glBindBuffer, GLenum target, GLuint buffer) \
	X(void, glBindVertexArray, GLuint array) \
	X(void, glBufferData, GLenum target, GLsizeiptr size, const void *data, GLenum usage) \
	X(void, glClearColor, GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha) \
	X(void, glClear, GLbitfield mask) \
	X(void, glCompileShader, GLuint shader) \
	X(GLuint, glCreateProgram, void) \
	X(GLuint, glCreateShader, GLenum shaderType) \
	X(void, glDebugMessageCallback, glDebugMessageCallbackFunc callback, const void *userParam) \
	X(void, glDebugMessageControl, GLenum source, GLenum type, GLenum severity, GLsizei count, const GLuint *ids, GLboolean enabled) \
	X(void, glDeleteBuffers, GLsizei n, const GLuint *buffers) \
	X(void, glDeleteProgram, GLuint program) \
	X(void, glDeleteShader, GLuint shader) \
	X(void, glDeleteVertexArrays, GLsizei n, const GLuint *arrays) \
	X(void, glDispatchCompute, GLuint num_groups_x, GLuint num_groups_y, GLuint num_groups_z) \
	X(void, glDrawArrays, GLenum mode, GLint first, GLsizei count) \
	X(void, glEnable, GLenum cap) \
	X(void, glEnableVertexAttribArray, GLuint index) \
	X(void, glGenBuffers, GLsizei n, GLuint *buffers) \
	X(void, glGenVertexArrays, GLsizei n, GLuint *arrays) \
	X(GLint, glGetAttribLocation, GLuint program, const GLchar *name) \
	X(void, glGetProgramInfoLog, GLuint program, GLsizei maxLength, GLsizei *length, GLchar *infoLog) \
	X(void, glGetProgramiv, GLuint program, GLenum pname, GLint *params) \
	X(GLuint, glGetProgramResourceIndex, GLuint program, GLenum programInterface, const char *name) \
	X(void, glGetShaderInfoLog, GLuint shader, GLsizei maxLength, GLsizei *length, GLchar *infoLog) \
	X(void, glGetShaderiv, GLuint shader, GLenum pname, GLint *params) \
	X(GLint, glGetUniformLocation, GLuint program, const GLchar *name) \
	X(void, glLinkProgram, GLuint program) \
	X(void, glMemoryBarrier, GLbitfield barriers) \
	X(void, glShaderSource, GLuint shader, GLsizei count, const GLchar **string, const GLint *length) \
	X(void, glUniform1f, GLint location, GLfloat v0) \
	X(void, glUniform1i, GLint location, GLint v0) \
	X(void, glUniform1ui, GLint location, GLuint v0) \
	X(void, glUniform3i, GLint location, GLint v0, GLint v1, GLint v2) \
	X(void, glUniformMatrix3fv, GLint location, GLsizei count, GLboolean transpose, const GLfloat *value) \
	X(void, glUseProgram, GLuint program) \
	X(void, glVertexAttribPointer, GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void *pointer) \
	X(void, glViewport, GLint x, GLint y, GLsizei width, GLsizei height) \
	X(void, glBufferSubData, GLenum target, GLintptr offset, GLsizeiptr size, const void *data) \
	X(void, glGetBufferSubData, GLenum target, GLintptr offset, GLsizeiptr size, void *data) \
	X(void, glDispatchComputeIndirect, GLintptr indirect) \
	// last line of the macro, don't remove

namespace Voxren::Gl
{
	using glDebugMessageCallbackFunc = void (*)(
		GLenum source,
		GLenum type,
		GLuint id,
		GLenum severity,
		GLsizei length,
		const GLchar *message,
		const void *userParam
	);

#define VOXREN_GLFUNC_DECLARE(ret, name, ...) extern ret (*name)(__VA_ARGS__);
	VOXREN_GLFUNC_LIST(VOXREN_GLFUNC_DECLARE)
#undef VOXREN_GLFUNC_DECLARE

	void LoadGlFuncs();
	void UnloadGlFuncs();
}

