// SPDX-License-Identifier: MIT
#pragma once

// Optional embedding configuration. Define BOX3D_USER_CONFIG to this header's path
// for the library AND its consumers. Standalone Box3D keeps upstream defaults.
#ifndef B3_ENABLE_ASSERT
#define B3_ENABLE_ASSERT
#endif

#if defined( EE_DLL ) && EE_DLL
#if defined( ESOTERICA_ENGINE_RUNTIME )
#ifndef box3d_EXPORTS
#define box3d_EXPORTS
#endif
#else
#ifndef BOX3D_DLL
#define BOX3D_DLL
#endif
#endif
#endif
