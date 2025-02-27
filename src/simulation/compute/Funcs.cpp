#include "Funcs.h"
#include <cassert>
#include <SDL.h>

namespace Voxren::Gl
{
#define VOXREN_GLFUNC_DEFINE(ret, name, ...) ret (*name)(__VA_ARGS__) = nullptr;
	VOXREN_GLFUNC_LIST(VOXREN_GLFUNC_DEFINE)
#undef VOXREN_GLFUNC_DEFINE

	void LoadGlFuncs()
	{
#define VOXREN_GLFUNC_LOAD(ret, name, ...) assert(name = reinterpret_cast<ret (*)(__VA_ARGS__)>(SDL_GL_GetProcAddress(#name)));
		VOXREN_GLFUNC_LIST(VOXREN_GLFUNC_LOAD)
#undef VOXREN_GLFUNC_LOAD
	}

	void UnloadGlFuncs()
	{
#define VOXREN_GLFUNC_UNLOAD(ret, name, ...) name = nullptr;
		VOXREN_GLFUNC_LIST(VOXREN_GLFUNC_UNLOAD)
#undef VOXREN_GLFUNC_UNLOAD
	}
}

