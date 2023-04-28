#ifndef GL15__POINTER_C_GENERATED_HEADER_OPENGL_H
#define GL15__POINTER_C_GENERATED_HEADER_OPENGL_H

#if defined(__glew_h__) || defined(__GLEW_H__)
#error Attempt to include auto-generated header after including glew.h
#endif
#if defined(__gl_h_) || defined(__GL_H__)
#error Attempt to include auto-generated header after including gl.h
#endif
#if defined(__glext_h_) || defined(__GLEXT_H_)
#error Attempt to include auto-generated header after including glext.h
#endif
#if defined(__gltypes_h_)
#error Attempt to include auto-generated header after gltypes.h
#endif
#if defined(__gl_ATI_h_)
#error Attempt to include auto-generated header after including glATI.h
#endif

#define __glew_h__
#define __GLEW_H__
#define __gl_h_
#define __GL_H__
#define __glext_h_
#define __GLEXT_H_
#define __gltypes_h_
#define __gl_ATI_h_

#ifndef APIENTRY
	#if defined(__MINGW32__)
		#ifndef WIN32_LEAN_AND_MEAN
			#define WIN32_LEAN_AND_MEAN 1
		#endif
		#ifndef NOMINMAX
			#define NOMINMAX
		#endif
		#include <windows.h>
	#elif (_MSC_VER >= 800) || defined(_STDCALL_SUPPORTED) || defined(__BORLANDC__)
		#ifndef WIN32_LEAN_AND_MEAN
			#define WIN32_LEAN_AND_MEAN 1
		#endif
		#ifndef NOMINMAX
			#define NOMINMAX
		#endif
		#include <windows.h>
	#else
		#define APIENTRY
	#endif
#endif /*APIENTRY*/

#ifndef CODEGEN_FUNCPTR
	#define CODEGEN_REMOVE_FUNCPTR
	#if defined(_WIN32)
		#define CODEGEN_FUNCPTR APIENTRY
	#else
		#define CODEGEN_FUNCPTR
	#endif
#endif /*CODEGEN_FUNCPTR*/

#ifndef GLAPI
	#define GLAPI extern
#endif


#ifndef GL_LOAD_GEN_BASIC_OPENGL_TYPEDEFS
#define GL_LOAD_GEN_BASIC_OPENGL_TYPEDEFS


#endif /*GL_LOAD_GEN_BASIC_OPENGL_TYPEDEFS*/


#include <stddef.h>
#ifndef GLEXT_64_TYPES_DEFINED
/* This code block is duplicated in glxext.h, so must be protected */
#define GLEXT_64_TYPES_DEFINED
/* Define int32_t, int64_t, and uint64_t types for UST/MSC */
/* (as used in the GL_EXT_timer_query extension). */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
#include <inttypes.h>
#elif defined(__sun__) || defined(__digital__)
#include <inttypes.h>
#if defined(__STDC__)
#if defined(__arch64__) || defined(_LP64)
typedef long int int64_t;
typedef unsigned long int uint64_t;
#else
typedef long long int int64_t;
typedef unsigned long long int uint64_t;
#endif /* __arch64__ */
#endif /* __STDC__ */
#elif defined( __VMS ) || defined(__sgi)
#include <inttypes.h>
#elif defined(__SCO__) || defined(__USLC__)
#include <stdint.h>
#elif defined(__UNIXOS2__) || defined(__SOL64__)
typedef long int int32_t;
typedef long long int int64_t;
typedef unsigned long long int uint64_t;
#elif defined(_WIN32) && defined(__GNUC__)
#include <stdint.h>
#elif defined(_WIN32)
typedef __int32 int32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
#else
/* Fallback if nothing above works */
#include <inttypes.h>
#endif
#endif
	typedef unsigned int GLenum;
	typedef unsigned char GLboolean;
	typedef unsigned int GLbitfield;
	typedef void GLvoid;
	typedef signed char GLbyte;
	typedef short GLshort;
	typedef int GLint;
	typedef unsigned char GLubyte;
	typedef unsigned short GLushort;
	typedef unsigned int GLuint;
	typedef int GLsizei;
	typedef float GLfloat;
	typedef float GLclampf;
	typedef double GLdouble;
	typedef double GLclampd;
	typedef char GLchar;
	typedef char GLcharARB;
	#ifdef __APPLE__
typedef void *GLhandleARB;
#else
typedef unsigned int GLhandleARB;
#endif
		typedef unsigned short GLhalfARB;
		typedef unsigned short GLhalf;
		typedef GLint GLfixed;
		typedef ptrdiff_t GLintptr;
		typedef ptrdiff_t GLsizeiptr;
		typedef int64_t GLint64;
		typedef uint64_t GLuint64;
		typedef ptrdiff_t GLintptrARB;
		typedef ptrdiff_t GLsizeiptrARB;
		typedef int64_t GLint64EXT;
		typedef uint64_t GLuint64EXT;
		typedef struct __GLsync *GLsync;
		struct _cl_context;
		struct _cl_event;
		typedef void (APIENTRY *GLDEBUGPROC)(GLenum source,GLenum type,GLuint id,GLenum severity,GLsizei length,const GLchar *message,const void *userParam);
		typedef void (APIENTRY *GLDEBUGPROCARB)(GLenum source,GLenum type,GLuint id,GLenum severity,GLsizei length,const GLchar *message,const void *userParam);
		typedef void (APIENTRY *GLDEBUGPROCAMD)(GLuint id,GLenum category,GLenum severity,GLsizei length,const GLchar *message,void *userParam);
		typedef unsigned short GLhalfNV;
		typedef GLintptr GLvdpauSurfaceNV;

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#define GL_2D 0x0600
#define GL_2_BYTES 0x1407
#define GL_3D 0x0601
#define GL_3D_COLOR 0x0602
#define GL_3D_COLOR_TEXTURE 0x0603
#define GL_3_BYTES 0x1408
#define GL_4D_COLOR_TEXTURE 0x0604
#define GL_4_BYTES 0x1409
#define GL_ACCUM 0x0100
#define GL_ACCUM_ALPHA_BITS 0x0D5B
#define GL_ACCUM_BLUE_BITS 0x0D5A
#define GL_ACCUM_BUFFER_BIT 0x00000200
#define GL_ACCUM_CLEAR_VALUE 0x0B80
#define GL_ACCUM_GREEN_BITS 0x0D59
#define GL_ACCUM_RED_BITS 0x0D58
#define GL_ADD 0x0104
#define GL_ALL_ATTRIB_BITS 0xFFFFFFFF
#define GL_ALPHA 0x1906
#define GL_ALPHA12 0x803D
#define GL_ALPHA16 0x803E
#define GL_ALPHA4 0x803B
#define GL_ALPHA8 0x803C
#define GL_ALPHA_BIAS 0x0D1D
#define GL_ALPHA_BITS 0x0D55
#define GL_ALPHA_SCALE 0x0D1C
#define GL_ALPHA_TEST 0x0BC0
#define GL_ALPHA_TEST_FUNC 0x0BC1
#define GL_ALPHA_TEST_REF 0x0BC2
#define GL_ALWAYS 0x0207
#define GL_AMBIENT 0x1200
#define GL_AMBIENT_AND_DIFFUSE 0x1602
#define GL_AND 0x1501
#define GL_AND_INVERTED 0x1504
#define GL_AND_REVERSE 0x1502
#define GL_ATTRIB_STACK_DEPTH 0x0BB0
#define GL_AUTO_NORMAL 0x0D80
#define GL_AUX0 0x0409
#define GL_AUX1 0x040A
#define GL_AUX2 0x040B
#define GL_AUX3 0x040C
#define GL_AUX_BUFFERS 0x0C00
#define GL_BACK 0x0405
#define GL_BACK_LEFT 0x0402
#define GL_BACK_RIGHT 0x0403
#define GL_BITMAP 0x1A00
#define GL_BITMAP_TOKEN 0x0704
#define GL_BLEND 0x0BE2
#define GL_BLEND_DST 0x0BE0
#define GL_BLEND_SRC 0x0BE1
#define GL_BLUE 0x1905
#define GL_BLUE_BIAS 0x0D1B
#define GL_BLUE_BITS 0x0D54
#define GL_BLUE_SCALE 0x0D1A
#define GL_BYTE 0x1400
#define GL_C3F_V3F 0x2A24
#define GL_C4F_N3F_V3F 0x2A26
#define GL_C4UB_V2F 0x2A22
#define GL_C4UB_V3F 0x2A23
#define GL_CCW 0x0901
#define GL_CLAMP 0x2900
#define GL_CLEAR 0x1500
#define GL_CLIENT_ALL_ATTRIB_BITS 0xFFFFFFFF
#define GL_CLIENT_ATTRIB_STACK_DEPTH 0x0BB1
#define GL_CLIENT_PIXEL_STORE_BIT 0x00000001
#define GL_CLIENT_VERTEX_ARRAY_BIT 0x00000002
#define GL_CLIP_PLANE0 0x3000
#define GL_CLIP_PLANE1 0x3001
#define GL_CLIP_PLANE2 0x3002
#define GL_CLIP_PLANE3 0x3003
#define GL_CLIP_PLANE4 0x3004
#define GL_CLIP_PLANE5 0x3005
#define GL_COEFF 0x0A00
#define GL_COLOR 0x1800
#define GL_COLOR_ARRAY 0x8076
#define GL_COLOR_ARRAY_POINTER 0x8090
#define GL_COLOR_ARRAY_SIZE 0x8081
#define GL_COLOR_ARRAY_STRIDE 0x8083
#define GL_COLOR_ARRAY_TYPE 0x8082
#define GL_COLOR_BUFFER_BIT 0x00004000
#define GL_COLOR_CLEAR_VALUE 0x0C22
#define GL_COLOR_INDEX 0x1900
#define GL_COLOR_INDEXES 0x1603
#define GL_COLOR_LOGIC_OP 0x0BF2
#define GL_COLOR_MATERIAL 0x0B57
#define GL_COLOR_MATERIAL_FACE 0x0B55
#define GL_COLOR_MATERIAL_PARAMETER 0x0B56
#define GL_COLOR_WRITEMASK 0x0C23
#define GL_COMPILE 0x1300
#define GL_COMPILE_AND_EXECUTE 0x1301
#define GL_CONSTANT_ATTENUATION 0x1207
#define GL_COPY 0x1503
#define GL_COPY_INVERTED 0x150C
#define GL_COPY_PIXEL_TOKEN 0x0706
#define GL_CULL_FACE 0x0B44
#define GL_CULL_FACE_MODE 0x0B45
#define GL_CURRENT_BIT 0x00000001
#define GL_CURRENT_COLOR 0x0B00
#define GL_CURRENT_INDEX 0x0B01
#define GL_CURRENT_NORMAL 0x0B02
#define GL_CURRENT_RASTER_COLOR 0x0B04
#define GL_CURRENT_RASTER_DISTANCE 0x0B09
#define GL_CURRENT_RASTER_INDEX 0x0B05
#define GL_CURRENT_RASTER_POSITION 0x0B07
#define GL_CURRENT_RASTER_POSITION_VALID 0x0B08
#define GL_CURRENT_RASTER_TEXTURE_COORDS 0x0B06
#define GL_CURRENT_TEXTURE_COORDS 0x0B03
#define GL_CW 0x0900
#define GL_DECAL 0x2101
#define GL_DECR 0x1E03
#define GL_DEPTH 0x1801
#define GL_DEPTH_BIAS 0x0D1F
#define GL_DEPTH_BITS 0x0D56
#define GL_DEPTH_BUFFER_BIT 0x00000100
#define GL_DEPTH_CLEAR_VALUE 0x0B73
#define GL_DEPTH_COMPONENT 0x1902
#define GL_DEPTH_FUNC 0x0B74
#define GL_DEPTH_RANGE 0x0B70
#define GL_DEPTH_SCALE 0x0D1E
#define GL_DEPTH_TEST 0x0B71
#define GL_DEPTH_WRITEMASK 0x0B72
#define GL_DIFFUSE 0x1201
#define GL_DITHER 0x0BD0
#define GL_DOMAIN 0x0A02
#define GL_DONT_CARE 0x1100
#define GL_DOUBLE 0x140A
#define GL_DOUBLEBUFFER 0x0C32
#define GL_DRAW_BUFFER 0x0C01
#define GL_DRAW_PIXEL_TOKEN 0x0705
#define GL_DST_ALPHA 0x0304
#define GL_DST_COLOR 0x0306
#define GL_EDGE_FLAG 0x0B43
#define GL_EDGE_FLAG_ARRAY 0x8079
#define GL_EDGE_FLAG_ARRAY_POINTER 0x8093
#define GL_EDGE_FLAG_ARRAY_STRIDE 0x808C
#define GL_EMISSION 0x1600
#define GL_ENABLE_BIT 0x00002000
#define GL_EQUAL 0x0202
#define GL_EQUIV 0x1509
#define GL_EVAL_BIT 0x00010000
#define GL_EXP 0x0800
#define GL_EXP2 0x0801
#define GL_EXTENSIONS 0x1F03
#define GL_EYE_LINEAR 0x2400
#define GL_EYE_PLANE 0x2502
#define GL_FALSE 0
#define GL_FASTEST 0x1101
#define GL_FEEDBACK 0x1C01
#define GL_FEEDBACK_BUFFER_POINTER 0x0DF0
#define GL_FEEDBACK_BUFFER_SIZE 0x0DF1
#define GL_FEEDBACK_BUFFER_TYPE 0x0DF2
#define GL_FILL 0x1B02
#define GL_FLAT 0x1D00
#define GL_FLOAT 0x1406
#define GL_FOG 0x0B60
#define GL_FOG_BIT 0x00000080
#define GL_FOG_COLOR 0x0B66
#define GL_FOG_DENSITY 0x0B62
#define GL_FOG_END 0x0B64
#define GL_FOG_HINT 0x0C54
#define GL_FOG_INDEX 0x0B61
#define GL_FOG_MODE 0x0B65
#define GL_FOG_START 0x0B63
#define GL_FRONT 0x0404
#define GL_FRONT_AND_BACK 0x0408
#define GL_FRONT_FACE 0x0B46
#define GL_FRONT_LEFT 0x0400
#define GL_FRONT_RIGHT 0x0401
#define GL_GEQUAL 0x0206
#define GL_GREATER 0x0204
#define GL_GREEN 0x1904
#define GL_GREEN_BIAS 0x0D19
#define GL_GREEN_BITS 0x0D53
#define GL_GREEN_SCALE 0x0D18
#define GL_HINT_BIT 0x00008000
#define GL_INCR 0x1E02
#define GL_INDEX_ARRAY 0x8077
#define GL_INDEX_ARRAY_POINTER 0x8091
#define GL_INDEX_ARRAY_STRIDE 0x8086
#define GL_INDEX_ARRAY_TYPE 0x8085
#define GL_INDEX_BITS 0x0D51
#define GL_INDEX_CLEAR_VALUE 0x0C20
#define GL_INDEX_LOGIC_OP 0x0BF1
#define GL_INDEX_MODE 0x0C30
#define GL_INDEX_OFFSET 0x0D13
#define GL_INDEX_SHIFT 0x0D12
#define GL_INDEX_WRITEMASK 0x0C21
#define GL_INT 0x1404
#define GL_INTENSITY 0x8049
#define GL_INTENSITY12 0x804C
#define GL_INTENSITY16 0x804D
#define GL_INTENSITY4 0x804A
#define GL_INTENSITY8 0x804B
#define GL_INVALID_ENUM 0x0500
#define GL_INVALID_OPERATION 0x0502
#define GL_INVALID_VALUE 0x0501
#define GL_INVERT 0x150A
#define GL_KEEP 0x1E00
#define GL_LEFT 0x0406
#define GL_LEQUAL 0x0203
#define GL_LESS 0x0201
#define GL_LIGHT0 0x4000
#define GL_LIGHT1 0x4001
#define GL_LIGHT2 0x4002
#define GL_LIGHT3 0x4003
#define GL_LIGHT4 0x4004
#define GL_LIGHT5 0x4005
#define GL_LIGHT6 0x4006
#define GL_LIGHT7 0x4007
#define GL_LIGHTING 0x0B50
#define GL_LIGHTING_BIT 0x00000040
#define GL_LIGHT_MODEL_AMBIENT 0x0B53
#define GL_LIGHT_MODEL_LOCAL_VIEWER 0x0B51
#define GL_LIGHT_MODEL_TWO_SIDE 0x0B52
#define GL_LINE 0x1B01
#define GL_LINEAR 0x2601
#define GL_LINEAR_ATTENUATION 0x1208
#define GL_LINEAR_MIPMAP_LINEAR 0x2703
#define GL_LINEAR_MIPMAP_NEAREST 0x2701
#define GL_LINES 0x0001
#define GL_LINE_BIT 0x00000004
#define GL_LINE_LOOP 0x0002
#define GL_LINE_RESET_TOKEN 0x0707
#define GL_LINE_SMOOTH 0x0B20
#define GL_LINE_SMOOTH_HINT 0x0C52
#define GL_LINE_STIPPLE 0x0B24
#define GL_LINE_STIPPLE_PATTERN 0x0B25
#define GL_LINE_STIPPLE_REPEAT 0x0B26
#define GL_LINE_STRIP 0x0003
#define GL_LINE_TOKEN 0x0702
#define GL_LINE_WIDTH 0x0B21
#define GL_LINE_WIDTH_GRANULARITY 0x0B23
#define GL_LINE_WIDTH_RANGE 0x0B22
#define GL_LIST_BASE 0x0B32
#define GL_LIST_BIT 0x00020000
#define GL_LIST_INDEX 0x0B33
#define GL_LIST_MODE 0x0B30
#define GL_LOAD 0x0101
#define GL_LOGIC_OP 0x0BF1
#define GL_LOGIC_OP_MODE 0x0BF0
#define GL_LUMINANCE 0x1909
#define GL_LUMINANCE12 0x8041
#define GL_LUMINANCE12_ALPHA12 0x8047
#define GL_LUMINANCE12_ALPHA4 0x8046
#define GL_LUMINANCE16 0x8042
#define GL_LUMINANCE16_ALPHA16 0x8048
#define GL_LUMINANCE4 0x803F
#define GL_LUMINANCE4_ALPHA4 0x8043
#define GL_LUMINANCE6_ALPHA2 0x8044
#define GL_LUMINANCE8 0x8040
#define GL_LUMINANCE8_ALPHA8 0x8045
#define GL_LUMINANCE_ALPHA 0x190A
#define GL_MAP1_COLOR_4 0x0D90
#define GL_MAP1_GRID_DOMAIN 0x0DD0
#define GL_MAP1_GRID_SEGMENTS 0x0DD1
#define GL_MAP1_INDEX 0x0D91
#define GL_MAP1_NORMAL 0x0D92
#define GL_MAP1_TEXTURE_COORD_1 0x0D93
#define GL_MAP1_TEXTURE_COORD_2 0x0D94
#define GL_MAP1_TEXTURE_COORD_3 0x0D95
#define GL_MAP1_TEXTURE_COORD_4 0x0D96
#define GL_MAP1_VERTEX_3 0x0D97
#define GL_MAP1_VERTEX_4 0x0D98
#define GL_MAP2_COLOR_4 0x0DB0
#define GL_MAP2_GRID_DOMAIN 0x0DD2
#define GL_MAP2_GRID_SEGMENTS 0x0DD3
#define GL_MAP2_INDEX 0x0DB1
#define GL_MAP2_NORMAL 0x0DB2
#define GL_MAP2_TEXTURE_COORD_1 0x0DB3
#define GL_MAP2_TEXTURE_COORD_2 0x0DB4
#define GL_MAP2_TEXTURE_COORD_3 0x0DB5
#define GL_MAP2_TEXTURE_COORD_4 0x0DB6
#define GL_MAP2_VERTEX_3 0x0DB7
#define GL_MAP2_VERTEX_4 0x0DB8
#define GL_MAP_COLOR 0x0D10
#define GL_MAP_STENCIL 0x0D11
#define GL_MATRIX_MODE 0x0BA0
#define GL_MAX_ATTRIB_STACK_DEPTH 0x0D35
#define GL_MAX_CLIENT_ATTRIB_STACK_DEPTH 0x0D3B
#define GL_MAX_CLIP_PLANES 0x0D32
#define GL_MAX_EVAL_ORDER 0x0D30
#define GL_MAX_LIGHTS 0x0D31
#define GL_MAX_LIST_NESTING 0x0B31
#define GL_MAX_MODELVIEW_STACK_DEPTH 0x0D36
#define GL_MAX_NAME_STACK_DEPTH 0x0D37
#define GL_MAX_PIXEL_MAP_TABLE 0x0D34
#define GL_MAX_PROJECTION_STACK_DEPTH 0x0D38
#define GL_MAX_TEXTURE_SIZE 0x0D33
#define GL_MAX_TEXTURE_STACK_DEPTH 0x0D39
#define GL_MAX_VIEWPORT_DIMS 0x0D3A
#define GL_MODELVIEW 0x1700
#define GL_MODELVIEW_MATRIX 0x0BA6
#define GL_MODELVIEW_STACK_DEPTH 0x0BA3
#define GL_MODULATE 0x2100
#define GL_MULT 0x0103
#define GL_N3F_V3F 0x2A25
#define GL_NAME_STACK_DEPTH 0x0D70
#define GL_NAND 0x150E
#define GL_NEAREST 0x2600
#define GL_NEAREST_MIPMAP_LINEAR 0x2702
#define GL_NEAREST_MIPMAP_NEAREST 0x2700
#define GL_NEVER 0x0200
#define GL_NICEST 0x1102
#define GL_NONE 0
#define GL_NOOP 0x1505
#define GL_NOR 0x1508
#define GL_NORMALIZE 0x0BA1
#define GL_NORMAL_ARRAY 0x8075
#define GL_NORMAL_ARRAY_POINTER 0x808F
#define GL_NORMAL_ARRAY_STRIDE 0x807F
#define GL_NORMAL_ARRAY_TYPE 0x807E
#define GL_NOTEQUAL 0x0205
#define GL_NO_ERROR 0
#define GL_OBJECT_LINEAR 0x2401
#define GL_OBJECT_PLANE 0x2501
#define GL_ONE 1
#define GL_ONE_MINUS_DST_ALPHA 0x0305
#define GL_ONE_MINUS_DST_COLOR 0x0307
#define GL_ONE_MINUS_SRC_ALPHA 0x0303
#define GL_ONE_MINUS_SRC_COLOR 0x0301
#define GL_OR 0x1507
#define GL_ORDER 0x0A01
#define GL_OR_INVERTED 0x150D
#define GL_OR_REVERSE 0x150B
#define GL_OUT_OF_MEMORY 0x0505
#define GL_PACK_ALIGNMENT 0x0D05
#define GL_PACK_LSB_FIRST 0x0D01
#define GL_PACK_ROW_LENGTH 0x0D02
#define GL_PACK_SKIP_PIXELS 0x0D04
#define GL_PACK_SKIP_ROWS 0x0D03
#define GL_PACK_SWAP_BYTES 0x0D00
#define GL_PASS_THROUGH_TOKEN 0x0700
#define GL_PERSPECTIVE_CORRECTION_HINT 0x0C50
#define GL_PIXEL_MAP_A_TO_A 0x0C79
#define GL_PIXEL_MAP_A_TO_A_SIZE 0x0CB9
#define GL_PIXEL_MAP_B_TO_B 0x0C78
#define GL_PIXEL_MAP_B_TO_B_SIZE 0x0CB8
#define GL_PIXEL_MAP_G_TO_G 0x0C77
#define GL_PIXEL_MAP_G_TO_G_SIZE 0x0CB7
#define GL_PIXEL_MAP_I_TO_A 0x0C75
#define GL_PIXEL_MAP_I_TO_A_SIZE 0x0CB5
#define GL_PIXEL_MAP_I_TO_B 0x0C74
#define GL_PIXEL_MAP_I_TO_B_SIZE 0x0CB4
#define GL_PIXEL_MAP_I_TO_G 0x0C73
#define GL_PIXEL_MAP_I_TO_G_SIZE 0x0CB3
#define GL_PIXEL_MAP_I_TO_I 0x0C70
#define GL_PIXEL_MAP_I_TO_I_SIZE 0x0CB0
#define GL_PIXEL_MAP_I_TO_R 0x0C72
#define GL_PIXEL_MAP_I_TO_R_SIZE 0x0CB2
#define GL_PIXEL_MAP_R_TO_R 0x0C76
#define GL_PIXEL_MAP_R_TO_R_SIZE 0x0CB6
#define GL_PIXEL_MAP_S_TO_S 0x0C71
#define GL_PIXEL_MAP_S_TO_S_SIZE 0x0CB1
#define GL_PIXEL_MODE_BIT 0x00000020
#define GL_POINT 0x1B00
#define GL_POINTS 0x0000
#define GL_POINT_BIT 0x00000002
#define GL_POINT_SIZE 0x0B11
#define GL_POINT_SIZE_GRANULARITY 0x0B13
#define GL_POINT_SIZE_RANGE 0x0B12
#define GL_POINT_SMOOTH 0x0B10
#define GL_POINT_SMOOTH_HINT 0x0C51
#define GL_POINT_TOKEN 0x0701
#define GL_POLYGON 0x0009
#define GL_POLYGON_BIT 0x00000008
#define GL_POLYGON_MODE 0x0B40
#define GL_POLYGON_OFFSET_FACTOR 0x8038
#define GL_POLYGON_OFFSET_FILL 0x8037
#define GL_POLYGON_OFFSET_LINE 0x2A02
#define GL_POLYGON_OFFSET_POINT 0x2A01
#define GL_POLYGON_OFFSET_UNITS 0x2A00
#define GL_POLYGON_SMOOTH 0x0B41
#define GL_POLYGON_SMOOTH_HINT 0x0C53
#define GL_POLYGON_STIPPLE 0x0B42
#define GL_POLYGON_STIPPLE_BIT 0x00000010
#define GL_POLYGON_TOKEN 0x0703
#define GL_POSITION 0x1203
#define GL_PROJECTION 0x1701
#define GL_PROJECTION_MATRIX 0x0BA7
#define GL_PROJECTION_STACK_DEPTH 0x0BA4
#define GL_PROXY_TEXTURE_1D 0x8063
#define GL_PROXY_TEXTURE_2D 0x8064
#define GL_Q 0x2003
#define GL_QUADRATIC_ATTENUATION 0x1209
#define GL_QUADS 0x0007
#define GL_QUAD_STRIP 0x0008
#define GL_R 0x2002
#define GL_R3_G3_B2 0x2A10
#define GL_READ_BUFFER 0x0C02
#define GL_RED 0x1903
#define GL_RED_BIAS 0x0D15
#define GL_RED_BITS 0x0D52
#define GL_RED_SCALE 0x0D14
#define GL_RENDER 0x1C00
#define GL_RENDERER 0x1F01
#define GL_RENDER_MODE 0x0C40
#define GL_REPEAT 0x2901
#define GL_REPLACE 0x1E01
#define GL_RETURN 0x0102
#define GL_RGB 0x1907
#define GL_RGB10 0x8052
#define GL_RGB10_A2 0x8059
#define GL_RGB12 0x8053
#define GL_RGB16 0x8054
#define GL_RGB4 0x804F
#define GL_RGB5 0x8050
#define GL_RGB5_A1 0x8057
#define GL_RGB8 0x8051
#define GL_RGBA 0x1908
#define GL_RGBA12 0x805A
#define GL_RGBA16 0x805B
#define GL_RGBA2 0x8055
#define GL_RGBA4 0x8056
#define GL_RGBA8 0x8058
#define GL_RGBA_MODE 0x0C31
#define GL_RIGHT 0x0407
#define GL_S 0x2000
#define GL_SCISSOR_BIT 0x00080000
#define GL_SCISSOR_BOX 0x0C10
#define GL_SCISSOR_TEST 0x0C11
#define GL_SELECT 0x1C02
#define GL_SELECTION_BUFFER_POINTER 0x0DF3
#define GL_SELECTION_BUFFER_SIZE 0x0DF4
#define GL_SET 0x150F
#define GL_SHADE_MODEL 0x0B54
#define GL_SHININESS 0x1601
#define GL_SHORT 0x1402
#define GL_SMOOTH 0x1D01
#define GL_SPECULAR 0x1202
#define GL_SPHERE_MAP 0x2402
#define GL_SPOT_CUTOFF 0x1206
#define GL_SPOT_DIRECTION 0x1204
#define GL_SPOT_EXPONENT 0x1205
#define GL_SRC_ALPHA 0x0302
#define GL_SRC_ALPHA_SATURATE 0x0308
#define GL_SRC_COLOR 0x0300
#define GL_STACK_OVERFLOW 0x0503
#define GL_STACK_UNDERFLOW 0x0504
#define GL_STENCIL 0x1802
#define GL_STENCIL_BITS 0x0D57
#define GL_STENCIL_BUFFER_BIT 0x00000400
#define GL_STENCIL_CLEAR_VALUE 0x0B91
#define GL_STENCIL_FAIL 0x0B94
#define GL_STENCIL_FUNC 0x0B92
#define GL_STENCIL_INDEX 0x1901
#define GL_STENCIL_PASS_DEPTH_FAIL 0x0B95
#define GL_STENCIL_PASS_DEPTH_PASS 0x0B96
#define GL_STENCIL_REF 0x0B97
#define GL_STENCIL_TEST 0x0B90
#define GL_STENCIL_VALUE_MASK 0x0B93
#define GL_STENCIL_WRITEMASK 0x0B98
#define GL_STEREO 0x0C33
#define GL_SUBPIXEL_BITS 0x0D50
#define GL_T 0x2001
#define GL_T2F_C3F_V3F 0x2A2A
#define GL_T2F_C4F_N3F_V3F 0x2A2C
#define GL_T2F_C4UB_V3F 0x2A29
#define GL_T2F_N3F_V3F 0x2A2B
#define GL_T2F_V3F 0x2A27
#define GL_T4F_C4F_N3F_V4F 0x2A2D
#define GL_T4F_V4F 0x2A28
#define GL_TEXTURE 0x1702
#define GL_TEXTURE_1D 0x0DE0
#define GL_TEXTURE_2D 0x0DE1
#define GL_TEXTURE_ALPHA_SIZE 0x805F
#define GL_TEXTURE_BINDING_1D 0x8068
#define GL_TEXTURE_BINDING_2D 0x8069
#define GL_TEXTURE_BIT 0x00040000
#define GL_TEXTURE_BLUE_SIZE 0x805E
#define GL_TEXTURE_BORDER 0x1005
#define GL_TEXTURE_BORDER_COLOR 0x1004
#define GL_TEXTURE_COMPONENTS 0x1003
#define GL_TEXTURE_COORD_ARRAY 0x8078
#define GL_TEXTURE_COORD_ARRAY_POINTER 0x8092
#define GL_TEXTURE_COORD_ARRAY_SIZE 0x8088
#define GL_TEXTURE_COORD_ARRAY_STRIDE 0x808A
#define GL_TEXTURE_COORD_ARRAY_TYPE 0x8089
#define GL_TEXTURE_ENV 0x2300
#define GL_TEXTURE_ENV_COLOR 0x2201
#define GL_TEXTURE_ENV_MODE 0x2200
#define GL_TEXTURE_GEN_MODE 0x2500
#define GL_TEXTURE_GEN_Q 0x0C63
#define GL_TEXTURE_GEN_R 0x0C62
#define GL_TEXTURE_GEN_S 0x0C60
#define GL_TEXTURE_GEN_T 0x0C61
#define GL_TEXTURE_GREEN_SIZE 0x805D
#define GL_TEXTURE_HEIGHT 0x1001
#define GL_TEXTURE_INTENSITY_SIZE 0x8061
#define GL_TEXTURE_INTERNAL_FORMAT 0x1003
#define GL_TEXTURE_LUMINANCE_SIZE 0x8060
#define GL_TEXTURE_MAG_FILTER 0x2800
#define GL_TEXTURE_MATRIX 0x0BA8
#define GL_TEXTURE_MIN_FILTER 0x2801
#define GL_TEXTURE_PRIORITY 0x8066
#define GL_TEXTURE_RED_SIZE 0x805C
#define GL_TEXTURE_RESIDENT 0x8067
#define GL_TEXTURE_STACK_DEPTH 0x0BA5
#define GL_TEXTURE_WIDTH 0x1000
#define GL_TEXTURE_WRAP_S 0x2802
#define GL_TEXTURE_WRAP_T 0x2803
#define GL_TRANSFORM_BIT 0x00001000
#define GL_TRIANGLES 0x0004
#define GL_TRIANGLE_FAN 0x0006
#define GL_TRIANGLE_STRIP 0x0005
#define GL_TRUE 1
#define GL_UNPACK_ALIGNMENT 0x0CF5
#define GL_UNPACK_LSB_FIRST 0x0CF1
#define GL_UNPACK_ROW_LENGTH 0x0CF2
#define GL_UNPACK_SKIP_PIXELS 0x0CF4
#define GL_UNPACK_SKIP_ROWS 0x0CF3
#define GL_UNPACK_SWAP_BYTES 0x0CF0
#define GL_UNSIGNED_BYTE 0x1401
#define GL_UNSIGNED_INT 0x1405
#define GL_UNSIGNED_SHORT 0x1403
#define GL_V2F 0x2A20
#define GL_V3F 0x2A21
#define GL_VENDOR 0x1F00
#define GL_VERSION 0x1F02
#define GL_VERTEX_ARRAY 0x8074
#define GL_VERTEX_ARRAY_POINTER 0x808E
#define GL_VERTEX_ARRAY_SIZE 0x807A
#define GL_VERTEX_ARRAY_STRIDE 0x807C
#define GL_VERTEX_ARRAY_TYPE 0x807B
#define GL_VIEWPORT 0x0BA2
#define GL_VIEWPORT_BIT 0x00000800
#define GL_XOR 0x1506
#define GL_ZERO 0
#define GL_ZOOM_X 0x0D16
#define GL_ZOOM_Y 0x0D17

#define GL_ALIASED_LINE_WIDTH_RANGE 0x846E
#define GL_ALIASED_POINT_SIZE_RANGE 0x846D
#define GL_BGR 0x80E0
#define GL_BGRA 0x80E1
#define GL_CLAMP_TO_EDGE 0x812F
#define GL_LIGHT_MODEL_COLOR_CONTROL 0x81F8
#define GL_MAX_3D_TEXTURE_SIZE 0x8073
#define GL_MAX_ELEMENTS_INDICES 0x80E9
#define GL_MAX_ELEMENTS_VERTICES 0x80E8
#define GL_PACK_IMAGE_HEIGHT 0x806C
#define GL_PACK_SKIP_IMAGES 0x806B
#define GL_PROXY_TEXTURE_3D 0x8070
#define GL_RESCALE_NORMAL 0x803A
#define GL_SEPARATE_SPECULAR_COLOR 0x81FA
#define GL_SINGLE_COLOR 0x81F9
#define GL_SMOOTH_LINE_WIDTH_GRANULARITY 0x0B23
#define GL_SMOOTH_LINE_WIDTH_RANGE 0x0B22
#define GL_SMOOTH_POINT_SIZE_GRANULARITY 0x0B13
#define GL_SMOOTH_POINT_SIZE_RANGE 0x0B12
#define GL_TEXTURE_3D 0x806F
#define GL_TEXTURE_BASE_LEVEL 0x813C
#define GL_TEXTURE_BINDING_3D 0x806A
#define GL_TEXTURE_DEPTH 0x8071
#define GL_TEXTURE_MAX_LEVEL 0x813D
#define GL_TEXTURE_MAX_LOD 0x813B
#define GL_TEXTURE_MIN_LOD 0x813A
#define GL_TEXTURE_WRAP_R 0x8072
#define GL_UNPACK_IMAGE_HEIGHT 0x806E
#define GL_UNPACK_SKIP_IMAGES 0x806D
#define GL_UNSIGNED_BYTE_2_3_3_REV 0x8362
#define GL_UNSIGNED_BYTE_3_3_2 0x8032
#define GL_UNSIGNED_INT_10_10_10_2 0x8036
#define GL_UNSIGNED_INT_2_10_10_10_REV 0x8368
#define GL_UNSIGNED_INT_8_8_8_8 0x8035
#define GL_UNSIGNED_INT_8_8_8_8_REV 0x8367
#define GL_UNSIGNED_SHORT_1_5_5_5_REV 0x8366
#define GL_UNSIGNED_SHORT_4_4_4_4 0x8033
#define GL_UNSIGNED_SHORT_4_4_4_4_REV 0x8365
#define GL_UNSIGNED_SHORT_5_5_5_1 0x8034
#define GL_UNSIGNED_SHORT_5_6_5 0x8363
#define GL_UNSIGNED_SHORT_5_6_5_REV 0x8364

#define GL_ACTIVE_TEXTURE 0x84E0
#define GL_ADD_SIGNED 0x8574
#define GL_CLAMP_TO_BORDER 0x812D
#define GL_CLIENT_ACTIVE_TEXTURE 0x84E1
#define GL_COMBINE 0x8570
#define GL_COMBINE_ALPHA 0x8572
#define GL_COMBINE_RGB 0x8571
#define GL_COMPRESSED_ALPHA 0x84E9
#define GL_COMPRESSED_INTENSITY 0x84EC
#define GL_COMPRESSED_LUMINANCE 0x84EA
#define GL_COMPRESSED_LUMINANCE_ALPHA 0x84EB
#define GL_COMPRESSED_RGB 0x84ED
#define GL_COMPRESSED_RGBA 0x84EE
#define GL_COMPRESSED_TEXTURE_FORMATS 0x86A3
#define GL_CONSTANT 0x8576
#define GL_DOT3_RGB 0x86AE
#define GL_DOT3_RGBA 0x86AF
#define GL_INTERPOLATE 0x8575
#define GL_MAX_CUBE_MAP_TEXTURE_SIZE 0x851C
#define GL_MAX_TEXTURE_UNITS 0x84E2
#define GL_MULTISAMPLE 0x809D
#define GL_MULTISAMPLE_BIT 0x20000000
#define GL_NORMAL_MAP 0x8511
#define GL_NUM_COMPRESSED_TEXTURE_FORMATS 0x86A2
#define GL_OPERAND0_ALPHA 0x8598
#define GL_OPERAND0_RGB 0x8590
#define GL_OPERAND1_ALPHA 0x8599
#define GL_OPERAND1_RGB 0x8591
#define GL_OPERAND2_ALPHA 0x859A
#define GL_OPERAND2_RGB 0x8592
#define GL_PREVIOUS 0x8578
#define GL_PRIMARY_COLOR 0x8577
#define GL_PROXY_TEXTURE_CUBE_MAP 0x851B
#define GL_REFLECTION_MAP 0x8512
#define GL_RGB_SCALE 0x8573
#define GL_SAMPLES 0x80A9
#define GL_SAMPLE_ALPHA_TO_COVERAGE 0x809E
#define GL_SAMPLE_ALPHA_TO_ONE 0x809F
#define GL_SAMPLE_BUFFERS 0x80A8
#define GL_SAMPLE_COVERAGE 0x80A0
#define GL_SAMPLE_COVERAGE_INVERT 0x80AB
#define GL_SAMPLE_COVERAGE_VALUE 0x80AA
#define GL_SOURCE0_ALPHA 0x8588
#define GL_SOURCE0_RGB 0x8580
#define GL_SOURCE1_ALPHA 0x8589
#define GL_SOURCE1_RGB 0x8581
#define GL_SOURCE2_ALPHA 0x858A
#define GL_SOURCE2_RGB 0x8582
#define GL_SUBTRACT 0x84E7
#define GL_TEXTURE0 0x84C0
#define GL_TEXTURE1 0x84C1
#define GL_TEXTURE10 0x84CA
#define GL_TEXTURE11 0x84CB
#define GL_TEXTURE12 0x84CC
#define GL_TEXTURE13 0x84CD
#define GL_TEXTURE14 0x84CE
#define GL_TEXTURE15 0x84CF
#define GL_TEXTURE16 0x84D0
#define GL_TEXTURE17 0x84D1
#define GL_TEXTURE18 0x84D2
#define GL_TEXTURE19 0x84D3
#define GL_TEXTURE2 0x84C2
#define GL_TEXTURE20 0x84D4
#define GL_TEXTURE21 0x84D5
#define GL_TEXTURE22 0x84D6
#define GL_TEXTURE23 0x84D7
#define GL_TEXTURE24 0x84D8
#define GL_TEXTURE25 0x84D9
#define GL_TEXTURE26 0x84DA
#define GL_TEXTURE27 0x84DB
#define GL_TEXTURE28 0x84DC
#define GL_TEXTURE29 0x84DD
#define GL_TEXTURE3 0x84C3
#define GL_TEXTURE30 0x84DE
#define GL_TEXTURE31 0x84DF
#define GL_TEXTURE4 0x84C4
#define GL_TEXTURE5 0x84C5
#define GL_TEXTURE6 0x84C6
#define GL_TEXTURE7 0x84C7
#define GL_TEXTURE8 0x84C8
#define GL_TEXTURE9 0x84C9
#define GL_TEXTURE_BINDING_CUBE_MAP 0x8514
#define GL_TEXTURE_COMPRESSED 0x86A1
#define GL_TEXTURE_COMPRESSED_IMAGE_SIZE 0x86A0
#define GL_TEXTURE_COMPRESSION_HINT 0x84EF
#define GL_TEXTURE_CUBE_MAP 0x8513
#define GL_TEXTURE_CUBE_MAP_NEGATIVE_X 0x8516
#define GL_TEXTURE_CUBE_MAP_NEGATIVE_Y 0x8518
#define GL_TEXTURE_CUBE_MAP_NEGATIVE_Z 0x851A
#define GL_TEXTURE_CUBE_MAP_POSITIVE_X 0x8515
#define GL_TEXTURE_CUBE_MAP_POSITIVE_Y 0x8517
#define GL_TEXTURE_CUBE_MAP_POSITIVE_Z 0x8519
#define GL_TRANSPOSE_COLOR_MATRIX 0x84E6
#define GL_TRANSPOSE_MODELVIEW_MATRIX 0x84E3
#define GL_TRANSPOSE_PROJECTION_MATRIX 0x84E4
#define GL_TRANSPOSE_TEXTURE_MATRIX 0x84E5

#define GL_BLEND_COLOR 0x8005
#define GL_BLEND_DST_ALPHA 0x80CA
#define GL_BLEND_DST_RGB 0x80C8
#define GL_BLEND_SRC_ALPHA 0x80CB
#define GL_BLEND_SRC_RGB 0x80C9
#define GL_COLOR_SUM 0x8458
#define GL_COMPARE_R_TO_TEXTURE 0x884E
#define GL_CONSTANT_ALPHA 0x8003
#define GL_CONSTANT_COLOR 0x8001
#define GL_CURRENT_FOG_COORDINATE 0x8453
#define GL_CURRENT_SECONDARY_COLOR 0x8459
#define GL_DECR_WRAP 0x8508
#define GL_DEPTH_COMPONENT16 0x81A5
#define GL_DEPTH_COMPONENT24 0x81A6
#define GL_DEPTH_COMPONENT32 0x81A7
#define GL_DEPTH_TEXTURE_MODE 0x884B
#define GL_FOG_COORDINATE 0x8451
#define GL_FOG_COORDINATE_ARRAY 0x8457
#define GL_FOG_COORDINATE_ARRAY_POINTER 0x8456
#define GL_FOG_COORDINATE_ARRAY_STRIDE 0x8455
#define GL_FOG_COORDINATE_ARRAY_TYPE 0x8454
#define GL_FOG_COORDINATE_SOURCE 0x8450
#define GL_FRAGMENT_DEPTH 0x8452
#define GL_FUNC_ADD 0x8006
#define GL_FUNC_REVERSE_SUBTRACT 0x800B
#define GL_FUNC_SUBTRACT 0x800A
#define GL_GENERATE_MIPMAP 0x8191
#define GL_GENERATE_MIPMAP_HINT 0x8192
#define GL_INCR_WRAP 0x8507
#define GL_MAX 0x8008
#define GL_MAX_TEXTURE_LOD_BIAS 0x84FD
#define GL_MIN 0x8007
#define GL_MIRRORED_REPEAT 0x8370
#define GL_ONE_MINUS_CONSTANT_ALPHA 0x8004
#define GL_ONE_MINUS_CONSTANT_COLOR 0x8002
#define GL_POINT_DISTANCE_ATTENUATION 0x8129
#define GL_POINT_FADE_THRESHOLD_SIZE 0x8128
#define GL_POINT_SIZE_MAX 0x8127
#define GL_POINT_SIZE_MIN 0x8126
#define GL_SECONDARY_COLOR_ARRAY 0x845E
#define GL_SECONDARY_COLOR_ARRAY_POINTER 0x845D
#define GL_SECONDARY_COLOR_ARRAY_SIZE 0x845A
#define GL_SECONDARY_COLOR_ARRAY_STRIDE 0x845C
#define GL_SECONDARY_COLOR_ARRAY_TYPE 0x845B
#define GL_TEXTURE_COMPARE_FUNC 0x884D
#define GL_TEXTURE_COMPARE_MODE 0x884C
#define GL_TEXTURE_DEPTH_SIZE 0x884A
#define GL_TEXTURE_FILTER_CONTROL 0x8500
#define GL_TEXTURE_LOD_BIAS 0x8501

#define GL_ARRAY_BUFFER 0x8892
#define GL_ARRAY_BUFFER_BINDING 0x8894
#define GL_BUFFER_ACCESS 0x88BB
#define GL_BUFFER_MAPPED 0x88BC
#define GL_BUFFER_MAP_POINTER 0x88BD
#define GL_BUFFER_SIZE 0x8764
#define GL_BUFFER_USAGE 0x8765
#define GL_COLOR_ARRAY_BUFFER_BINDING 0x8898
#define GL_CURRENT_FOG_COORD 0x8453
#define GL_CURRENT_QUERY 0x8865
#define GL_DYNAMIC_COPY 0x88EA
#define GL_DYNAMIC_DRAW 0x88E8
#define GL_DYNAMIC_READ 0x88E9
#define GL_EDGE_FLAG_ARRAY_BUFFER_BINDING 0x889B
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#define GL_ELEMENT_ARRAY_BUFFER_BINDING 0x8895
#define GL_FOG_COORD 0x8451
#define GL_FOG_COORDINATE_ARRAY_BUFFER_BINDING 0x889D
#define GL_FOG_COORD_ARRAY 0x8457
#define GL_FOG_COORD_ARRAY_BUFFER_BINDING 0x889D
#define GL_FOG_COORD_ARRAY_POINTER 0x8456
#define GL_FOG_COORD_ARRAY_STRIDE 0x8455
#define GL_FOG_COORD_ARRAY_TYPE 0x8454
#define GL_FOG_COORD_SRC 0x8450
#define GL_INDEX_ARRAY_BUFFER_BINDING 0x8899
#define GL_NORMAL_ARRAY_BUFFER_BINDING 0x8897
#define GL_QUERY_COUNTER_BITS 0x8864
#define GL_QUERY_RESULT 0x8866
#define GL_QUERY_RESULT_AVAILABLE 0x8867
#define GL_READ_ONLY 0x88B8
#define GL_READ_WRITE 0x88BA
#define GL_SAMPLES_PASSED 0x8914
#define GL_SECONDARY_COLOR_ARRAY_BUFFER_BINDING 0x889C
#define GL_SRC0_ALPHA 0x8588
#define GL_SRC0_RGB 0x8580
#define GL_SRC1_ALPHA 0x8589
#define GL_SRC1_RGB 0x8581
#define GL_SRC2_ALPHA 0x858A
#define GL_SRC2_RGB 0x8582
#define GL_STATIC_COPY 0x88E6
#define GL_STATIC_DRAW 0x88E4
#define GL_STATIC_READ 0x88E5
#define GL_STREAM_COPY 0x88E2
#define GL_STREAM_DRAW 0x88E0
#define GL_STREAM_READ 0x88E1
#define GL_TEXTURE_COORD_ARRAY_BUFFER_BINDING 0x889A
#define GL_VERTEX_ARRAY_BUFFER_BINDING 0x8896
#define GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING 0x889F
#define GL_WEIGHT_ARRAY_BUFFER_BINDING 0x889E
#define GL_WRITE_ONLY 0x88B9

extern void (CODEGEN_FUNCPTR *GL15__ptrc_glAccum)(GLenum op, GLfloat value);
#define glAccum GL15__ptrc_glAccum
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glAlphaFunc)(GLenum func, GLfloat ref);
#define glAlphaFunc GL15__ptrc_glAlphaFunc
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBegin)(GLenum mode);
#define glBegin GL15__ptrc_glBegin
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBitmap)(GLsizei width, GLsizei height, GLfloat xorig, GLfloat yorig, GLfloat xmove, GLfloat ymove, const GLubyte * bitmap);
#define glBitmap GL15__ptrc_glBitmap
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBlendFunc)(GLenum sfactor, GLenum dfactor);
#define glBlendFunc GL15__ptrc_glBlendFunc
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCallList)(GLuint list);
#define glCallList GL15__ptrc_glCallList
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCallLists)(GLsizei n, GLenum type, const void * lists);
#define glCallLists GL15__ptrc_glCallLists
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glClear)(GLbitfield mask);
#define glClear GL15__ptrc_glClear
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glClearAccum)(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha);
#define glClearAccum GL15__ptrc_glClearAccum
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glClearColor)(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha);
#define glClearColor GL15__ptrc_glClearColor
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glClearDepth)(GLdouble depth);
#define glClearDepth GL15__ptrc_glClearDepth
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glClearIndex)(GLfloat c);
#define glClearIndex GL15__ptrc_glClearIndex
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glClearStencil)(GLint s);
#define glClearStencil GL15__ptrc_glClearStencil
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glClipPlane)(GLenum plane, const GLdouble * equation);
#define glClipPlane GL15__ptrc_glClipPlane
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3b)(GLbyte red, GLbyte green, GLbyte blue);
#define glColor3b GL15__ptrc_glColor3b
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3bv)(const GLbyte * v);
#define glColor3bv GL15__ptrc_glColor3bv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3d)(GLdouble red, GLdouble green, GLdouble blue);
#define glColor3d GL15__ptrc_glColor3d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3dv)(const GLdouble * v);
#define glColor3dv GL15__ptrc_glColor3dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3f)(GLfloat red, GLfloat green, GLfloat blue);
#define glColor3f GL15__ptrc_glColor3f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3fv)(const GLfloat * v);
#define glColor3fv GL15__ptrc_glColor3fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3i)(GLint red, GLint green, GLint blue);
#define glColor3i GL15__ptrc_glColor3i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3iv)(const GLint * v);
#define glColor3iv GL15__ptrc_glColor3iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3s)(GLshort red, GLshort green, GLshort blue);
#define glColor3s GL15__ptrc_glColor3s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3sv)(const GLshort * v);
#define glColor3sv GL15__ptrc_glColor3sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3ub)(GLubyte red, GLubyte green, GLubyte blue);
#define glColor3ub GL15__ptrc_glColor3ub
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3ubv)(const GLubyte * v);
#define glColor3ubv GL15__ptrc_glColor3ubv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3ui)(GLuint red, GLuint green, GLuint blue);
#define glColor3ui GL15__ptrc_glColor3ui
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3uiv)(const GLuint * v);
#define glColor3uiv GL15__ptrc_glColor3uiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3us)(GLushort red, GLushort green, GLushort blue);
#define glColor3us GL15__ptrc_glColor3us
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3usv)(const GLushort * v);
#define glColor3usv GL15__ptrc_glColor3usv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4b)(GLbyte red, GLbyte green, GLbyte blue, GLbyte alpha);
#define glColor4b GL15__ptrc_glColor4b
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4bv)(const GLbyte * v);
#define glColor4bv GL15__ptrc_glColor4bv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4d)(GLdouble red, GLdouble green, GLdouble blue, GLdouble alpha);
#define glColor4d GL15__ptrc_glColor4d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4dv)(const GLdouble * v);
#define glColor4dv GL15__ptrc_glColor4dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4f)(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha);
#define glColor4f GL15__ptrc_glColor4f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4fv)(const GLfloat * v);
#define glColor4fv GL15__ptrc_glColor4fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4i)(GLint red, GLint green, GLint blue, GLint alpha);
#define glColor4i GL15__ptrc_glColor4i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4iv)(const GLint * v);
#define glColor4iv GL15__ptrc_glColor4iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4s)(GLshort red, GLshort green, GLshort blue, GLshort alpha);
#define glColor4s GL15__ptrc_glColor4s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4sv)(const GLshort * v);
#define glColor4sv GL15__ptrc_glColor4sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4ub)(GLubyte red, GLubyte green, GLubyte blue, GLubyte alpha);
#define glColor4ub GL15__ptrc_glColor4ub
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4ubv)(const GLubyte * v);
#define glColor4ubv GL15__ptrc_glColor4ubv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4ui)(GLuint red, GLuint green, GLuint blue, GLuint alpha);
#define glColor4ui GL15__ptrc_glColor4ui
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4uiv)(const GLuint * v);
#define glColor4uiv GL15__ptrc_glColor4uiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4us)(GLushort red, GLushort green, GLushort blue, GLushort alpha);
#define glColor4us GL15__ptrc_glColor4us
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4usv)(const GLushort * v);
#define glColor4usv GL15__ptrc_glColor4usv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColorMask)(GLboolean red, GLboolean green, GLboolean blue, GLboolean alpha);
#define glColorMask GL15__ptrc_glColorMask
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColorMaterial)(GLenum face, GLenum mode);
#define glColorMaterial GL15__ptrc_glColorMaterial
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyPixels)(GLint x, GLint y, GLsizei width, GLsizei height, GLenum type);
#define glCopyPixels GL15__ptrc_glCopyPixels
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCullFace)(GLenum mode);
#define glCullFace GL15__ptrc_glCullFace
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDeleteLists)(GLuint list, GLsizei range);
#define glDeleteLists GL15__ptrc_glDeleteLists
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDepthFunc)(GLenum func);
#define glDepthFunc GL15__ptrc_glDepthFunc
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDepthMask)(GLboolean flag);
#define glDepthMask GL15__ptrc_glDepthMask
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDepthRange)(GLdouble ren_near, GLdouble ren_far);
#define glDepthRange GL15__ptrc_glDepthRange
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDisable)(GLenum cap);
#define glDisable GL15__ptrc_glDisable
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawBuffer)(GLenum buf);
#define glDrawBuffer GL15__ptrc_glDrawBuffer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawPixels)(GLsizei width, GLsizei height, GLenum format, GLenum type, const void * pixels);
#define glDrawPixels GL15__ptrc_glDrawPixels
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEdgeFlag)(GLboolean flag);
#define glEdgeFlag GL15__ptrc_glEdgeFlag
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEdgeFlagv)(const GLboolean * flag);
#define glEdgeFlagv GL15__ptrc_glEdgeFlagv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEnable)(GLenum cap);
#define glEnable GL15__ptrc_glEnable
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEnd)(void);
#define glEnd GL15__ptrc_glEnd
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEndList)(void);
#define glEndList GL15__ptrc_glEndList
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord1d)(GLdouble u);
#define glEvalCoord1d GL15__ptrc_glEvalCoord1d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord1dv)(const GLdouble * u);
#define glEvalCoord1dv GL15__ptrc_glEvalCoord1dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord1f)(GLfloat u);
#define glEvalCoord1f GL15__ptrc_glEvalCoord1f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord1fv)(const GLfloat * u);
#define glEvalCoord1fv GL15__ptrc_glEvalCoord1fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord2d)(GLdouble u, GLdouble v);
#define glEvalCoord2d GL15__ptrc_glEvalCoord2d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord2dv)(const GLdouble * u);
#define glEvalCoord2dv GL15__ptrc_glEvalCoord2dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord2f)(GLfloat u, GLfloat v);
#define glEvalCoord2f GL15__ptrc_glEvalCoord2f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord2fv)(const GLfloat * u);
#define glEvalCoord2fv GL15__ptrc_glEvalCoord2fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalMesh1)(GLenum mode, GLint i1, GLint i2);
#define glEvalMesh1 GL15__ptrc_glEvalMesh1
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalMesh2)(GLenum mode, GLint i1, GLint i2, GLint j1, GLint j2);
#define glEvalMesh2 GL15__ptrc_glEvalMesh2
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalPoint1)(GLint i);
#define glEvalPoint1 GL15__ptrc_glEvalPoint1
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalPoint2)(GLint i, GLint j);
#define glEvalPoint2 GL15__ptrc_glEvalPoint2
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFeedbackBuffer)(GLsizei size, GLenum type, GLfloat * buffer);
#define glFeedbackBuffer GL15__ptrc_glFeedbackBuffer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFinish)(void);
#define glFinish GL15__ptrc_glFinish
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFlush)(void);
#define glFlush GL15__ptrc_glFlush
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFogf)(GLenum pname, GLfloat param);
#define glFogf GL15__ptrc_glFogf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFogfv)(GLenum pname, const GLfloat * params);
#define glFogfv GL15__ptrc_glFogfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFogi)(GLenum pname, GLint param);
#define glFogi GL15__ptrc_glFogi
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFogiv)(GLenum pname, const GLint * params);
#define glFogiv GL15__ptrc_glFogiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFrontFace)(GLenum mode);
#define glFrontFace GL15__ptrc_glFrontFace
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFrustum)(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top, GLdouble zNear, GLdouble zFar);
#define glFrustum GL15__ptrc_glFrustum
extern GLuint (CODEGEN_FUNCPTR *GL15__ptrc_glGenLists)(GLsizei range);
#define glGenLists GL15__ptrc_glGenLists
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetBooleanv)(GLenum pname, GLboolean * data);
#define glGetBooleanv GL15__ptrc_glGetBooleanv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetClipPlane)(GLenum plane, GLdouble * equation);
#define glGetClipPlane GL15__ptrc_glGetClipPlane
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetDoublev)(GLenum pname, GLdouble * data);
#define glGetDoublev GL15__ptrc_glGetDoublev
extern GLenum (CODEGEN_FUNCPTR *GL15__ptrc_glGetError)(void);
#define glGetError GL15__ptrc_glGetError
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetFloatv)(GLenum pname, GLfloat * data);
#define glGetFloatv GL15__ptrc_glGetFloatv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetIntegerv)(GLenum pname, GLint * data);
#define glGetIntegerv GL15__ptrc_glGetIntegerv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetLightfv)(GLenum light, GLenum pname, GLfloat * params);
#define glGetLightfv GL15__ptrc_glGetLightfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetLightiv)(GLenum light, GLenum pname, GLint * params);
#define glGetLightiv GL15__ptrc_glGetLightiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMapdv)(GLenum target, GLenum query, GLdouble * v);
#define glGetMapdv GL15__ptrc_glGetMapdv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMapfv)(GLenum target, GLenum query, GLfloat * v);
#define glGetMapfv GL15__ptrc_glGetMapfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMapiv)(GLenum target, GLenum query, GLint * v);
#define glGetMapiv GL15__ptrc_glGetMapiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMaterialfv)(GLenum face, GLenum pname, GLfloat * params);
#define glGetMaterialfv GL15__ptrc_glGetMaterialfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMaterialiv)(GLenum face, GLenum pname, GLint * params);
#define glGetMaterialiv GL15__ptrc_glGetMaterialiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPixelMapfv)(GLenum map, GLfloat * values);
#define glGetPixelMapfv GL15__ptrc_glGetPixelMapfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPixelMapuiv)(GLenum map, GLuint * values);
#define glGetPixelMapuiv GL15__ptrc_glGetPixelMapuiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPixelMapusv)(GLenum map, GLushort * values);
#define glGetPixelMapusv GL15__ptrc_glGetPixelMapusv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPolygonStipple)(GLubyte * mask);
#define glGetPolygonStipple GL15__ptrc_glGetPolygonStipple
extern const GLubyte * (CODEGEN_FUNCPTR *GL15__ptrc_glGetString)(GLenum name);
#define glGetString GL15__ptrc_glGetString
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexEnvfv)(GLenum target, GLenum pname, GLfloat * params);
#define glGetTexEnvfv GL15__ptrc_glGetTexEnvfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexEnviv)(GLenum target, GLenum pname, GLint * params);
#define glGetTexEnviv GL15__ptrc_glGetTexEnviv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexGendv)(GLenum coord, GLenum pname, GLdouble * params);
#define glGetTexGendv GL15__ptrc_glGetTexGendv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexGenfv)(GLenum coord, GLenum pname, GLfloat * params);
#define glGetTexGenfv GL15__ptrc_glGetTexGenfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexGeniv)(GLenum coord, GLenum pname, GLint * params);
#define glGetTexGeniv GL15__ptrc_glGetTexGeniv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexImage)(GLenum target, GLint level, GLenum format, GLenum type, void * pixels);
#define glGetTexImage GL15__ptrc_glGetTexImage
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexLevelParameterfv)(GLenum target, GLint level, GLenum pname, GLfloat * params);
#define glGetTexLevelParameterfv GL15__ptrc_glGetTexLevelParameterfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexLevelParameteriv)(GLenum target, GLint level, GLenum pname, GLint * params);
#define glGetTexLevelParameteriv GL15__ptrc_glGetTexLevelParameteriv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexParameterfv)(GLenum target, GLenum pname, GLfloat * params);
#define glGetTexParameterfv GL15__ptrc_glGetTexParameterfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexParameteriv)(GLenum target, GLenum pname, GLint * params);
#define glGetTexParameteriv GL15__ptrc_glGetTexParameteriv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glHint)(GLenum target, GLenum mode);
#define glHint GL15__ptrc_glHint
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexMask)(GLuint mask);
#define glIndexMask GL15__ptrc_glIndexMask
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexd)(GLdouble c);
#define glIndexd GL15__ptrc_glIndexd
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexdv)(const GLdouble * c);
#define glIndexdv GL15__ptrc_glIndexdv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexf)(GLfloat c);
#define glIndexf GL15__ptrc_glIndexf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexfv)(const GLfloat * c);
#define glIndexfv GL15__ptrc_glIndexfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexi)(GLint c);
#define glIndexi GL15__ptrc_glIndexi
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexiv)(const GLint * c);
#define glIndexiv GL15__ptrc_glIndexiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexs)(GLshort c);
#define glIndexs GL15__ptrc_glIndexs
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexsv)(const GLshort * c);
#define glIndexsv GL15__ptrc_glIndexsv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glInitNames)(void);
#define glInitNames GL15__ptrc_glInitNames
extern GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsEnabled)(GLenum cap);
#define glIsEnabled GL15__ptrc_glIsEnabled
extern GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsList)(GLuint list);
#define glIsList GL15__ptrc_glIsList
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLightModelf)(GLenum pname, GLfloat param);
#define glLightModelf GL15__ptrc_glLightModelf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLightModelfv)(GLenum pname, const GLfloat * params);
#define glLightModelfv GL15__ptrc_glLightModelfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLightModeli)(GLenum pname, GLint param);
#define glLightModeli GL15__ptrc_glLightModeli
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLightModeliv)(GLenum pname, const GLint * params);
#define glLightModeliv GL15__ptrc_glLightModeliv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLightf)(GLenum light, GLenum pname, GLfloat param);
#define glLightf GL15__ptrc_glLightf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLightfv)(GLenum light, GLenum pname, const GLfloat * params);
#define glLightfv GL15__ptrc_glLightfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLighti)(GLenum light, GLenum pname, GLint param);
#define glLighti GL15__ptrc_glLighti
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLightiv)(GLenum light, GLenum pname, const GLint * params);
#define glLightiv GL15__ptrc_glLightiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLineStipple)(GLint factor, GLushort pattern);
#define glLineStipple GL15__ptrc_glLineStipple
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLineWidth)(GLfloat width);
#define glLineWidth GL15__ptrc_glLineWidth
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glListBase)(GLuint base);
#define glListBase GL15__ptrc_glListBase
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadIdentity)(void);
#define glLoadIdentity GL15__ptrc_glLoadIdentity
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadMatrixd)(const GLdouble * m);
#define glLoadMatrixd GL15__ptrc_glLoadMatrixd
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadMatrixf)(const GLfloat * m);
#define glLoadMatrixf GL15__ptrc_glLoadMatrixf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadName)(GLuint name);
#define glLoadName GL15__ptrc_glLoadName
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLogicOp)(GLenum opcode);
#define glLogicOp GL15__ptrc_glLogicOp
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMap1d)(GLenum target, GLdouble u1, GLdouble u2, GLint stride, GLint order, const GLdouble * points);
#define glMap1d GL15__ptrc_glMap1d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMap1f)(GLenum target, GLfloat u1, GLfloat u2, GLint stride, GLint order, const GLfloat * points);
#define glMap1f GL15__ptrc_glMap1f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMap2d)(GLenum target, GLdouble u1, GLdouble u2, GLint ustride, GLint uorder, GLdouble v1, GLdouble v2, GLint vstride, GLint vorder, const GLdouble * points);
#define glMap2d GL15__ptrc_glMap2d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMap2f)(GLenum target, GLfloat u1, GLfloat u2, GLint ustride, GLint uorder, GLfloat v1, GLfloat v2, GLint vstride, GLint vorder, const GLfloat * points);
#define glMap2f GL15__ptrc_glMap2f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMapGrid1d)(GLint un, GLdouble u1, GLdouble u2);
#define glMapGrid1d GL15__ptrc_glMapGrid1d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMapGrid1f)(GLint un, GLfloat u1, GLfloat u2);
#define glMapGrid1f GL15__ptrc_glMapGrid1f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMapGrid2d)(GLint un, GLdouble u1, GLdouble u2, GLint vn, GLdouble v1, GLdouble v2);
#define glMapGrid2d GL15__ptrc_glMapGrid2d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMapGrid2f)(GLint un, GLfloat u1, GLfloat u2, GLint vn, GLfloat v1, GLfloat v2);
#define glMapGrid2f GL15__ptrc_glMapGrid2f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMaterialf)(GLenum face, GLenum pname, GLfloat param);
#define glMaterialf GL15__ptrc_glMaterialf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMaterialfv)(GLenum face, GLenum pname, const GLfloat * params);
#define glMaterialfv GL15__ptrc_glMaterialfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMateriali)(GLenum face, GLenum pname, GLint param);
#define glMateriali GL15__ptrc_glMateriali
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMaterialiv)(GLenum face, GLenum pname, const GLint * params);
#define glMaterialiv GL15__ptrc_glMaterialiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMatrixMode)(GLenum mode);
#define glMatrixMode GL15__ptrc_glMatrixMode
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultMatrixd)(const GLdouble * m);
#define glMultMatrixd GL15__ptrc_glMultMatrixd
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultMatrixf)(const GLfloat * m);
#define glMultMatrixf GL15__ptrc_glMultMatrixf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNewList)(GLuint list, GLenum mode);
#define glNewList GL15__ptrc_glNewList
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3b)(GLbyte nx, GLbyte ny, GLbyte nz);
#define glNormal3b GL15__ptrc_glNormal3b
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3bv)(const GLbyte * v);
#define glNormal3bv GL15__ptrc_glNormal3bv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3d)(GLdouble nx, GLdouble ny, GLdouble nz);
#define glNormal3d GL15__ptrc_glNormal3d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3dv)(const GLdouble * v);
#define glNormal3dv GL15__ptrc_glNormal3dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3f)(GLfloat nx, GLfloat ny, GLfloat nz);
#define glNormal3f GL15__ptrc_glNormal3f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3fv)(const GLfloat * v);
#define glNormal3fv GL15__ptrc_glNormal3fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3i)(GLint nx, GLint ny, GLint nz);
#define glNormal3i GL15__ptrc_glNormal3i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3iv)(const GLint * v);
#define glNormal3iv GL15__ptrc_glNormal3iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3s)(GLshort nx, GLshort ny, GLshort nz);
#define glNormal3s GL15__ptrc_glNormal3s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3sv)(const GLshort * v);
#define glNormal3sv GL15__ptrc_glNormal3sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glOrtho)(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top, GLdouble zNear, GLdouble zFar);
#define glOrtho GL15__ptrc_glOrtho
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPassThrough)(GLfloat token);
#define glPassThrough GL15__ptrc_glPassThrough
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelMapfv)(GLenum map, GLsizei mapsize, const GLfloat * values);
#define glPixelMapfv GL15__ptrc_glPixelMapfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelMapuiv)(GLenum map, GLsizei mapsize, const GLuint * values);
#define glPixelMapuiv GL15__ptrc_glPixelMapuiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelMapusv)(GLenum map, GLsizei mapsize, const GLushort * values);
#define glPixelMapusv GL15__ptrc_glPixelMapusv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelStoref)(GLenum pname, GLfloat param);
#define glPixelStoref GL15__ptrc_glPixelStoref
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelStorei)(GLenum pname, GLint param);
#define glPixelStorei GL15__ptrc_glPixelStorei
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelTransferf)(GLenum pname, GLfloat param);
#define glPixelTransferf GL15__ptrc_glPixelTransferf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelTransferi)(GLenum pname, GLint param);
#define glPixelTransferi GL15__ptrc_glPixelTransferi
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelZoom)(GLfloat xfactor, GLfloat yfactor);
#define glPixelZoom GL15__ptrc_glPixelZoom
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPointSize)(GLfloat size);
#define glPointSize GL15__ptrc_glPointSize
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPolygonMode)(GLenum face, GLenum mode);
#define glPolygonMode GL15__ptrc_glPolygonMode
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPolygonStipple)(const GLubyte * mask);
#define glPolygonStipple GL15__ptrc_glPolygonStipple
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPopAttrib)(void);
#define glPopAttrib GL15__ptrc_glPopAttrib
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPopMatrix)(void);
#define glPopMatrix GL15__ptrc_glPopMatrix
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPopName)(void);
#define glPopName GL15__ptrc_glPopName
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPushAttrib)(GLbitfield mask);
#define glPushAttrib GL15__ptrc_glPushAttrib
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPushMatrix)(void);
#define glPushMatrix GL15__ptrc_glPushMatrix
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPushName)(GLuint name);
#define glPushName GL15__ptrc_glPushName
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2d)(GLdouble x, GLdouble y);
#define glRasterPos2d GL15__ptrc_glRasterPos2d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2dv)(const GLdouble * v);
#define glRasterPos2dv GL15__ptrc_glRasterPos2dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2f)(GLfloat x, GLfloat y);
#define glRasterPos2f GL15__ptrc_glRasterPos2f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2fv)(const GLfloat * v);
#define glRasterPos2fv GL15__ptrc_glRasterPos2fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2i)(GLint x, GLint y);
#define glRasterPos2i GL15__ptrc_glRasterPos2i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2iv)(const GLint * v);
#define glRasterPos2iv GL15__ptrc_glRasterPos2iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2s)(GLshort x, GLshort y);
#define glRasterPos2s GL15__ptrc_glRasterPos2s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2sv)(const GLshort * v);
#define glRasterPos2sv GL15__ptrc_glRasterPos2sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3d)(GLdouble x, GLdouble y, GLdouble z);
#define glRasterPos3d GL15__ptrc_glRasterPos3d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3dv)(const GLdouble * v);
#define glRasterPos3dv GL15__ptrc_glRasterPos3dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3f)(GLfloat x, GLfloat y, GLfloat z);
#define glRasterPos3f GL15__ptrc_glRasterPos3f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3fv)(const GLfloat * v);
#define glRasterPos3fv GL15__ptrc_glRasterPos3fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3i)(GLint x, GLint y, GLint z);
#define glRasterPos3i GL15__ptrc_glRasterPos3i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3iv)(const GLint * v);
#define glRasterPos3iv GL15__ptrc_glRasterPos3iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3s)(GLshort x, GLshort y, GLshort z);
#define glRasterPos3s GL15__ptrc_glRasterPos3s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3sv)(const GLshort * v);
#define glRasterPos3sv GL15__ptrc_glRasterPos3sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4d)(GLdouble x, GLdouble y, GLdouble z, GLdouble w);
#define glRasterPos4d GL15__ptrc_glRasterPos4d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4dv)(const GLdouble * v);
#define glRasterPos4dv GL15__ptrc_glRasterPos4dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4f)(GLfloat x, GLfloat y, GLfloat z, GLfloat w);
#define glRasterPos4f GL15__ptrc_glRasterPos4f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4fv)(const GLfloat * v);
#define glRasterPos4fv GL15__ptrc_glRasterPos4fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4i)(GLint x, GLint y, GLint z, GLint w);
#define glRasterPos4i GL15__ptrc_glRasterPos4i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4iv)(const GLint * v);
#define glRasterPos4iv GL15__ptrc_glRasterPos4iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4s)(GLshort x, GLshort y, GLshort z, GLshort w);
#define glRasterPos4s GL15__ptrc_glRasterPos4s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4sv)(const GLshort * v);
#define glRasterPos4sv GL15__ptrc_glRasterPos4sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glReadBuffer)(GLenum src);
#define glReadBuffer GL15__ptrc_glReadBuffer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glReadPixels)(GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, void * pixels);
#define glReadPixels GL15__ptrc_glReadPixels
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRectd)(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2);
#define glRectd GL15__ptrc_glRectd
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRectdv)(const GLdouble * v1, const GLdouble * v2);
#define glRectdv GL15__ptrc_glRectdv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRectf)(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2);
#define glRectf GL15__ptrc_glRectf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRectfv)(const GLfloat * v1, const GLfloat * v2);
#define glRectfv GL15__ptrc_glRectfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRecti)(GLint x1, GLint y1, GLint x2, GLint y2);
#define glRecti GL15__ptrc_glRecti
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRectiv)(const GLint * v1, const GLint * v2);
#define glRectiv GL15__ptrc_glRectiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRects)(GLshort x1, GLshort y1, GLshort x2, GLshort y2);
#define glRects GL15__ptrc_glRects
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRectsv)(const GLshort * v1, const GLshort * v2);
#define glRectsv GL15__ptrc_glRectsv
extern GLint (CODEGEN_FUNCPTR *GL15__ptrc_glRenderMode)(GLenum mode);
#define glRenderMode GL15__ptrc_glRenderMode
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRotated)(GLdouble angle, GLdouble x, GLdouble y, GLdouble z);
#define glRotated GL15__ptrc_glRotated
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glRotatef)(GLfloat angle, GLfloat x, GLfloat y, GLfloat z);
#define glRotatef GL15__ptrc_glRotatef
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glScaled)(GLdouble x, GLdouble y, GLdouble z);
#define glScaled GL15__ptrc_glScaled
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glScalef)(GLfloat x, GLfloat y, GLfloat z);
#define glScalef GL15__ptrc_glScalef
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glScissor)(GLint x, GLint y, GLsizei width, GLsizei height);
#define glScissor GL15__ptrc_glScissor
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSelectBuffer)(GLsizei size, GLuint * buffer);
#define glSelectBuffer GL15__ptrc_glSelectBuffer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glShadeModel)(GLenum mode);
#define glShadeModel GL15__ptrc_glShadeModel
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glStencilFunc)(GLenum func, GLint ref, GLuint mask);
#define glStencilFunc GL15__ptrc_glStencilFunc
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glStencilMask)(GLuint mask);
#define glStencilMask GL15__ptrc_glStencilMask
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glStencilOp)(GLenum fail, GLenum zfail, GLenum zpass);
#define glStencilOp GL15__ptrc_glStencilOp
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1d)(GLdouble s);
#define glTexCoord1d GL15__ptrc_glTexCoord1d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1dv)(const GLdouble * v);
#define glTexCoord1dv GL15__ptrc_glTexCoord1dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1f)(GLfloat s);
#define glTexCoord1f GL15__ptrc_glTexCoord1f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1fv)(const GLfloat * v);
#define glTexCoord1fv GL15__ptrc_glTexCoord1fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1i)(GLint s);
#define glTexCoord1i GL15__ptrc_glTexCoord1i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1iv)(const GLint * v);
#define glTexCoord1iv GL15__ptrc_glTexCoord1iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1s)(GLshort s);
#define glTexCoord1s GL15__ptrc_glTexCoord1s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1sv)(const GLshort * v);
#define glTexCoord1sv GL15__ptrc_glTexCoord1sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2d)(GLdouble s, GLdouble t);
#define glTexCoord2d GL15__ptrc_glTexCoord2d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2dv)(const GLdouble * v);
#define glTexCoord2dv GL15__ptrc_glTexCoord2dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2f)(GLfloat s, GLfloat t);
#define glTexCoord2f GL15__ptrc_glTexCoord2f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2fv)(const GLfloat * v);
#define glTexCoord2fv GL15__ptrc_glTexCoord2fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2i)(GLint s, GLint t);
#define glTexCoord2i GL15__ptrc_glTexCoord2i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2iv)(const GLint * v);
#define glTexCoord2iv GL15__ptrc_glTexCoord2iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2s)(GLshort s, GLshort t);
#define glTexCoord2s GL15__ptrc_glTexCoord2s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2sv)(const GLshort * v);
#define glTexCoord2sv GL15__ptrc_glTexCoord2sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3d)(GLdouble s, GLdouble t, GLdouble r);
#define glTexCoord3d GL15__ptrc_glTexCoord3d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3dv)(const GLdouble * v);
#define glTexCoord3dv GL15__ptrc_glTexCoord3dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3f)(GLfloat s, GLfloat t, GLfloat r);
#define glTexCoord3f GL15__ptrc_glTexCoord3f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3fv)(const GLfloat * v);
#define glTexCoord3fv GL15__ptrc_glTexCoord3fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3i)(GLint s, GLint t, GLint r);
#define glTexCoord3i GL15__ptrc_glTexCoord3i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3iv)(const GLint * v);
#define glTexCoord3iv GL15__ptrc_glTexCoord3iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3s)(GLshort s, GLshort t, GLshort r);
#define glTexCoord3s GL15__ptrc_glTexCoord3s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3sv)(const GLshort * v);
#define glTexCoord3sv GL15__ptrc_glTexCoord3sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4d)(GLdouble s, GLdouble t, GLdouble r, GLdouble q);
#define glTexCoord4d GL15__ptrc_glTexCoord4d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4dv)(const GLdouble * v);
#define glTexCoord4dv GL15__ptrc_glTexCoord4dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4f)(GLfloat s, GLfloat t, GLfloat r, GLfloat q);
#define glTexCoord4f GL15__ptrc_glTexCoord4f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4fv)(const GLfloat * v);
#define glTexCoord4fv GL15__ptrc_glTexCoord4fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4i)(GLint s, GLint t, GLint r, GLint q);
#define glTexCoord4i GL15__ptrc_glTexCoord4i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4iv)(const GLint * v);
#define glTexCoord4iv GL15__ptrc_glTexCoord4iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4s)(GLshort s, GLshort t, GLshort r, GLshort q);
#define glTexCoord4s GL15__ptrc_glTexCoord4s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4sv)(const GLshort * v);
#define glTexCoord4sv GL15__ptrc_glTexCoord4sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexEnvf)(GLenum target, GLenum pname, GLfloat param);
#define glTexEnvf GL15__ptrc_glTexEnvf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexEnvfv)(GLenum target, GLenum pname, const GLfloat * params);
#define glTexEnvfv GL15__ptrc_glTexEnvfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexEnvi)(GLenum target, GLenum pname, GLint param);
#define glTexEnvi GL15__ptrc_glTexEnvi
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexEnviv)(GLenum target, GLenum pname, const GLint * params);
#define glTexEnviv GL15__ptrc_glTexEnviv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGend)(GLenum coord, GLenum pname, GLdouble param);
#define glTexGend GL15__ptrc_glTexGend
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGendv)(GLenum coord, GLenum pname, const GLdouble * params);
#define glTexGendv GL15__ptrc_glTexGendv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGenf)(GLenum coord, GLenum pname, GLfloat param);
#define glTexGenf GL15__ptrc_glTexGenf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGenfv)(GLenum coord, GLenum pname, const GLfloat * params);
#define glTexGenfv GL15__ptrc_glTexGenfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGeni)(GLenum coord, GLenum pname, GLint param);
#define glTexGeni GL15__ptrc_glTexGeni
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGeniv)(GLenum coord, GLenum pname, const GLint * params);
#define glTexGeniv GL15__ptrc_glTexGeniv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexImage1D)(GLenum target, GLint level, GLint internalformat, GLsizei width, GLint border, GLenum format, GLenum type, const void * pixels);
#define glTexImage1D GL15__ptrc_glTexImage1D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexImage2D)(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const void * pixels);
#define glTexImage2D GL15__ptrc_glTexImage2D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexParameterf)(GLenum target, GLenum pname, GLfloat param);
#define glTexParameterf GL15__ptrc_glTexParameterf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexParameterfv)(GLenum target, GLenum pname, const GLfloat * params);
#define glTexParameterfv GL15__ptrc_glTexParameterfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexParameteri)(GLenum target, GLenum pname, GLint param);
#define glTexParameteri GL15__ptrc_glTexParameteri
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexParameteriv)(GLenum target, GLenum pname, const GLint * params);
#define glTexParameteriv GL15__ptrc_glTexParameteriv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTranslated)(GLdouble x, GLdouble y, GLdouble z);
#define glTranslated GL15__ptrc_glTranslated
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTranslatef)(GLfloat x, GLfloat y, GLfloat z);
#define glTranslatef GL15__ptrc_glTranslatef
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2d)(GLdouble x, GLdouble y);
#define glVertex2d GL15__ptrc_glVertex2d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2dv)(const GLdouble * v);
#define glVertex2dv GL15__ptrc_glVertex2dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2f)(GLfloat x, GLfloat y);
#define glVertex2f GL15__ptrc_glVertex2f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2fv)(const GLfloat * v);
#define glVertex2fv GL15__ptrc_glVertex2fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2i)(GLint x, GLint y);
#define glVertex2i GL15__ptrc_glVertex2i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2iv)(const GLint * v);
#define glVertex2iv GL15__ptrc_glVertex2iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2s)(GLshort x, GLshort y);
#define glVertex2s GL15__ptrc_glVertex2s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2sv)(const GLshort * v);
#define glVertex2sv GL15__ptrc_glVertex2sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3d)(GLdouble x, GLdouble y, GLdouble z);
#define glVertex3d GL15__ptrc_glVertex3d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3dv)(const GLdouble * v);
#define glVertex3dv GL15__ptrc_glVertex3dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3f)(GLfloat x, GLfloat y, GLfloat z);
#define glVertex3f GL15__ptrc_glVertex3f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3fv)(const GLfloat * v);
#define glVertex3fv GL15__ptrc_glVertex3fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3i)(GLint x, GLint y, GLint z);
#define glVertex3i GL15__ptrc_glVertex3i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3iv)(const GLint * v);
#define glVertex3iv GL15__ptrc_glVertex3iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3s)(GLshort x, GLshort y, GLshort z);
#define glVertex3s GL15__ptrc_glVertex3s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3sv)(const GLshort * v);
#define glVertex3sv GL15__ptrc_glVertex3sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4d)(GLdouble x, GLdouble y, GLdouble z, GLdouble w);
#define glVertex4d GL15__ptrc_glVertex4d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4dv)(const GLdouble * v);
#define glVertex4dv GL15__ptrc_glVertex4dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4f)(GLfloat x, GLfloat y, GLfloat z, GLfloat w);
#define glVertex4f GL15__ptrc_glVertex4f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4fv)(const GLfloat * v);
#define glVertex4fv GL15__ptrc_glVertex4fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4i)(GLint x, GLint y, GLint z, GLint w);
#define glVertex4i GL15__ptrc_glVertex4i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4iv)(const GLint * v);
#define glVertex4iv GL15__ptrc_glVertex4iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4s)(GLshort x, GLshort y, GLshort z, GLshort w);
#define glVertex4s GL15__ptrc_glVertex4s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4sv)(const GLshort * v);
#define glVertex4sv GL15__ptrc_glVertex4sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glViewport)(GLint x, GLint y, GLsizei width, GLsizei height);
#define glViewport GL15__ptrc_glViewport

extern GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glAreTexturesResident)(GLsizei n, const GLuint * textures, GLboolean * residences);
#define glAreTexturesResident GL15__ptrc_glAreTexturesResident
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glArrayElement)(GLint i);
#define glArrayElement GL15__ptrc_glArrayElement
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBindTexture)(GLenum target, GLuint texture);
#define glBindTexture GL15__ptrc_glBindTexture
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glColorPointer)(GLint size, GLenum type, GLsizei stride, const void * pointer);
#define glColorPointer GL15__ptrc_glColorPointer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexImage1D)(GLenum target, GLint level, GLenum internalformat, GLint x, GLint y, GLsizei width, GLint border);
#define glCopyTexImage1D GL15__ptrc_glCopyTexImage1D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexImage2D)(GLenum target, GLint level, GLenum internalformat, GLint x, GLint y, GLsizei width, GLsizei height, GLint border);
#define glCopyTexImage2D GL15__ptrc_glCopyTexImage2D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexSubImage1D)(GLenum target, GLint level, GLint xoffset, GLint x, GLint y, GLsizei width);
#define glCopyTexSubImage1D GL15__ptrc_glCopyTexSubImage1D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexSubImage2D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint x, GLint y, GLsizei width, GLsizei height);
#define glCopyTexSubImage2D GL15__ptrc_glCopyTexSubImage2D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDeleteTextures)(GLsizei n, const GLuint * textures);
#define glDeleteTextures GL15__ptrc_glDeleteTextures
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDisableClientState)(GLenum ren_array);
#define glDisableClientState GL15__ptrc_glDisableClientState
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawArrays)(GLenum mode, GLint first, GLsizei count);
#define glDrawArrays GL15__ptrc_glDrawArrays
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawElements)(GLenum mode, GLsizei count, GLenum type, const void * indices);
#define glDrawElements GL15__ptrc_glDrawElements
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEdgeFlagPointer)(GLsizei stride, const void * pointer);
#define glEdgeFlagPointer GL15__ptrc_glEdgeFlagPointer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEnableClientState)(GLenum ren_array);
#define glEnableClientState GL15__ptrc_glEnableClientState
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGenTextures)(GLsizei n, GLuint * textures);
#define glGenTextures GL15__ptrc_glGenTextures
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPointerv)(GLenum pname, void ** params);
#define glGetPointerv GL15__ptrc_glGetPointerv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexPointer)(GLenum type, GLsizei stride, const void * pointer);
#define glIndexPointer GL15__ptrc_glIndexPointer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexub)(GLubyte c);
#define glIndexub GL15__ptrc_glIndexub
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexubv)(const GLubyte * c);
#define glIndexubv GL15__ptrc_glIndexubv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glInterleavedArrays)(GLenum format, GLsizei stride, const void * pointer);
#define glInterleavedArrays GL15__ptrc_glInterleavedArrays
extern GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsTexture)(GLuint texture);
#define glIsTexture GL15__ptrc_glIsTexture
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glNormalPointer)(GLenum type, GLsizei stride, const void * pointer);
#define glNormalPointer GL15__ptrc_glNormalPointer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPolygonOffset)(GLfloat factor, GLfloat units);
#define glPolygonOffset GL15__ptrc_glPolygonOffset
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPopClientAttrib)(void);
#define glPopClientAttrib GL15__ptrc_glPopClientAttrib
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPrioritizeTextures)(GLsizei n, const GLuint * textures, const GLfloat * priorities);
#define glPrioritizeTextures GL15__ptrc_glPrioritizeTextures
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPushClientAttrib)(GLbitfield mask);
#define glPushClientAttrib GL15__ptrc_glPushClientAttrib
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoordPointer)(GLint size, GLenum type, GLsizei stride, const void * pointer);
#define glTexCoordPointer GL15__ptrc_glTexCoordPointer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexSubImage1D)(GLenum target, GLint level, GLint xoffset, GLsizei width, GLenum format, GLenum type, const void * pixels);
#define glTexSubImage1D GL15__ptrc_glTexSubImage1D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexSubImage2D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum format, GLenum type, const void * pixels);
#define glTexSubImage2D GL15__ptrc_glTexSubImage2D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glVertexPointer)(GLint size, GLenum type, GLsizei stride, const void * pointer);
#define glVertexPointer GL15__ptrc_glVertexPointer

extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexSubImage3D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLint x, GLint y, GLsizei width, GLsizei height);
#define glCopyTexSubImage3D GL15__ptrc_glCopyTexSubImage3D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawRangeElements)(GLenum mode, GLuint start, GLuint end, GLsizei count, GLenum type, const void * indices);
#define glDrawRangeElements GL15__ptrc_glDrawRangeElements
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexImage3D)(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, const void * pixels);
#define glTexImage3D GL15__ptrc_glTexImage3D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glTexSubImage3D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLenum type, const void * pixels);
#define glTexSubImage3D GL15__ptrc_glTexSubImage3D

extern void (CODEGEN_FUNCPTR *GL15__ptrc_glActiveTexture)(GLenum texture);
#define glActiveTexture GL15__ptrc_glActiveTexture
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glClientActiveTexture)(GLenum texture);
#define glClientActiveTexture GL15__ptrc_glClientActiveTexture
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexImage1D)(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLint border, GLsizei imageSize, const void * data);
#define glCompressedTexImage1D GL15__ptrc_glCompressedTexImage1D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexImage2D)(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLint border, GLsizei imageSize, const void * data);
#define glCompressedTexImage2D GL15__ptrc_glCompressedTexImage2D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexImage3D)(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLsizei imageSize, const void * data);
#define glCompressedTexImage3D GL15__ptrc_glCompressedTexImage3D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexSubImage1D)(GLenum target, GLint level, GLint xoffset, GLsizei width, GLenum format, GLsizei imageSize, const void * data);
#define glCompressedTexSubImage1D GL15__ptrc_glCompressedTexSubImage1D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexSubImage2D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum format, GLsizei imageSize, const void * data);
#define glCompressedTexSubImage2D GL15__ptrc_glCompressedTexSubImage2D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexSubImage3D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLsizei imageSize, const void * data);
#define glCompressedTexSubImage3D GL15__ptrc_glCompressedTexSubImage3D
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetCompressedTexImage)(GLenum target, GLint level, void * img);
#define glGetCompressedTexImage GL15__ptrc_glGetCompressedTexImage
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadTransposeMatrixd)(const GLdouble * m);
#define glLoadTransposeMatrixd GL15__ptrc_glLoadTransposeMatrixd
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadTransposeMatrixf)(const GLfloat * m);
#define glLoadTransposeMatrixf GL15__ptrc_glLoadTransposeMatrixf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultTransposeMatrixd)(const GLdouble * m);
#define glMultTransposeMatrixd GL15__ptrc_glMultTransposeMatrixd
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultTransposeMatrixf)(const GLfloat * m);
#define glMultTransposeMatrixf GL15__ptrc_glMultTransposeMatrixf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1d)(GLenum target, GLdouble s);
#define glMultiTexCoord1d GL15__ptrc_glMultiTexCoord1d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1dv)(GLenum target, const GLdouble * v);
#define glMultiTexCoord1dv GL15__ptrc_glMultiTexCoord1dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1f)(GLenum target, GLfloat s);
#define glMultiTexCoord1f GL15__ptrc_glMultiTexCoord1f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1fv)(GLenum target, const GLfloat * v);
#define glMultiTexCoord1fv GL15__ptrc_glMultiTexCoord1fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1i)(GLenum target, GLint s);
#define glMultiTexCoord1i GL15__ptrc_glMultiTexCoord1i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1iv)(GLenum target, const GLint * v);
#define glMultiTexCoord1iv GL15__ptrc_glMultiTexCoord1iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1s)(GLenum target, GLshort s);
#define glMultiTexCoord1s GL15__ptrc_glMultiTexCoord1s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1sv)(GLenum target, const GLshort * v);
#define glMultiTexCoord1sv GL15__ptrc_glMultiTexCoord1sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2d)(GLenum target, GLdouble s, GLdouble t);
#define glMultiTexCoord2d GL15__ptrc_glMultiTexCoord2d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2dv)(GLenum target, const GLdouble * v);
#define glMultiTexCoord2dv GL15__ptrc_glMultiTexCoord2dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2f)(GLenum target, GLfloat s, GLfloat t);
#define glMultiTexCoord2f GL15__ptrc_glMultiTexCoord2f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2fv)(GLenum target, const GLfloat * v);
#define glMultiTexCoord2fv GL15__ptrc_glMultiTexCoord2fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2i)(GLenum target, GLint s, GLint t);
#define glMultiTexCoord2i GL15__ptrc_glMultiTexCoord2i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2iv)(GLenum target, const GLint * v);
#define glMultiTexCoord2iv GL15__ptrc_glMultiTexCoord2iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2s)(GLenum target, GLshort s, GLshort t);
#define glMultiTexCoord2s GL15__ptrc_glMultiTexCoord2s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2sv)(GLenum target, const GLshort * v);
#define glMultiTexCoord2sv GL15__ptrc_glMultiTexCoord2sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3d)(GLenum target, GLdouble s, GLdouble t, GLdouble r);
#define glMultiTexCoord3d GL15__ptrc_glMultiTexCoord3d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3dv)(GLenum target, const GLdouble * v);
#define glMultiTexCoord3dv GL15__ptrc_glMultiTexCoord3dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3f)(GLenum target, GLfloat s, GLfloat t, GLfloat r);
#define glMultiTexCoord3f GL15__ptrc_glMultiTexCoord3f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3fv)(GLenum target, const GLfloat * v);
#define glMultiTexCoord3fv GL15__ptrc_glMultiTexCoord3fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3i)(GLenum target, GLint s, GLint t, GLint r);
#define glMultiTexCoord3i GL15__ptrc_glMultiTexCoord3i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3iv)(GLenum target, const GLint * v);
#define glMultiTexCoord3iv GL15__ptrc_glMultiTexCoord3iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3s)(GLenum target, GLshort s, GLshort t, GLshort r);
#define glMultiTexCoord3s GL15__ptrc_glMultiTexCoord3s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3sv)(GLenum target, const GLshort * v);
#define glMultiTexCoord3sv GL15__ptrc_glMultiTexCoord3sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4d)(GLenum target, GLdouble s, GLdouble t, GLdouble r, GLdouble q);
#define glMultiTexCoord4d GL15__ptrc_glMultiTexCoord4d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4dv)(GLenum target, const GLdouble * v);
#define glMultiTexCoord4dv GL15__ptrc_glMultiTexCoord4dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4f)(GLenum target, GLfloat s, GLfloat t, GLfloat r, GLfloat q);
#define glMultiTexCoord4f GL15__ptrc_glMultiTexCoord4f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4fv)(GLenum target, const GLfloat * v);
#define glMultiTexCoord4fv GL15__ptrc_glMultiTexCoord4fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4i)(GLenum target, GLint s, GLint t, GLint r, GLint q);
#define glMultiTexCoord4i GL15__ptrc_glMultiTexCoord4i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4iv)(GLenum target, const GLint * v);
#define glMultiTexCoord4iv GL15__ptrc_glMultiTexCoord4iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4s)(GLenum target, GLshort s, GLshort t, GLshort r, GLshort q);
#define glMultiTexCoord4s GL15__ptrc_glMultiTexCoord4s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4sv)(GLenum target, const GLshort * v);
#define glMultiTexCoord4sv GL15__ptrc_glMultiTexCoord4sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSampleCoverage)(GLfloat value, GLboolean invert);
#define glSampleCoverage GL15__ptrc_glSampleCoverage

extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBlendColor)(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha);
#define glBlendColor GL15__ptrc_glBlendColor
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBlendEquation)(GLenum mode);
#define glBlendEquation GL15__ptrc_glBlendEquation
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBlendFuncSeparate)(GLenum sfactorRGB, GLenum dfactorRGB, GLenum sfactorAlpha, GLenum dfactorAlpha);
#define glBlendFuncSeparate GL15__ptrc_glBlendFuncSeparate
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoordPointer)(GLenum type, GLsizei stride, const void * pointer);
#define glFogCoordPointer GL15__ptrc_glFogCoordPointer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoordd)(GLdouble coord);
#define glFogCoordd GL15__ptrc_glFogCoordd
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoorddv)(const GLdouble * coord);
#define glFogCoorddv GL15__ptrc_glFogCoorddv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoordf)(GLfloat coord);
#define glFogCoordf GL15__ptrc_glFogCoordf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoordfv)(const GLfloat * coord);
#define glFogCoordfv GL15__ptrc_glFogCoordfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiDrawArrays)(GLenum mode, const GLint * first, const GLsizei * count, GLsizei drawcount);
#define glMultiDrawArrays GL15__ptrc_glMultiDrawArrays
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiDrawElements)(GLenum mode, const GLsizei * count, GLenum type, const void *const* indices, GLsizei drawcount);
#define glMultiDrawElements GL15__ptrc_glMultiDrawElements
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPointParameterf)(GLenum pname, GLfloat param);
#define glPointParameterf GL15__ptrc_glPointParameterf
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPointParameterfv)(GLenum pname, const GLfloat * params);
#define glPointParameterfv GL15__ptrc_glPointParameterfv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPointParameteri)(GLenum pname, GLint param);
#define glPointParameteri GL15__ptrc_glPointParameteri
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glPointParameteriv)(GLenum pname, const GLint * params);
#define glPointParameteriv GL15__ptrc_glPointParameteriv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3b)(GLbyte red, GLbyte green, GLbyte blue);
#define glSecondaryColor3b GL15__ptrc_glSecondaryColor3b
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3bv)(const GLbyte * v);
#define glSecondaryColor3bv GL15__ptrc_glSecondaryColor3bv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3d)(GLdouble red, GLdouble green, GLdouble blue);
#define glSecondaryColor3d GL15__ptrc_glSecondaryColor3d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3dv)(const GLdouble * v);
#define glSecondaryColor3dv GL15__ptrc_glSecondaryColor3dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3f)(GLfloat red, GLfloat green, GLfloat blue);
#define glSecondaryColor3f GL15__ptrc_glSecondaryColor3f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3fv)(const GLfloat * v);
#define glSecondaryColor3fv GL15__ptrc_glSecondaryColor3fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3i)(GLint red, GLint green, GLint blue);
#define glSecondaryColor3i GL15__ptrc_glSecondaryColor3i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3iv)(const GLint * v);
#define glSecondaryColor3iv GL15__ptrc_glSecondaryColor3iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3s)(GLshort red, GLshort green, GLshort blue);
#define glSecondaryColor3s GL15__ptrc_glSecondaryColor3s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3sv)(const GLshort * v);
#define glSecondaryColor3sv GL15__ptrc_glSecondaryColor3sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3ub)(GLubyte red, GLubyte green, GLubyte blue);
#define glSecondaryColor3ub GL15__ptrc_glSecondaryColor3ub
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3ubv)(const GLubyte * v);
#define glSecondaryColor3ubv GL15__ptrc_glSecondaryColor3ubv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3ui)(GLuint red, GLuint green, GLuint blue);
#define glSecondaryColor3ui GL15__ptrc_glSecondaryColor3ui
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3uiv)(const GLuint * v);
#define glSecondaryColor3uiv GL15__ptrc_glSecondaryColor3uiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3us)(GLushort red, GLushort green, GLushort blue);
#define glSecondaryColor3us GL15__ptrc_glSecondaryColor3us
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3usv)(const GLushort * v);
#define glSecondaryColor3usv GL15__ptrc_glSecondaryColor3usv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColorPointer)(GLint size, GLenum type, GLsizei stride, const void * pointer);
#define glSecondaryColorPointer GL15__ptrc_glSecondaryColorPointer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2d)(GLdouble x, GLdouble y);
#define glWindowPos2d GL15__ptrc_glWindowPos2d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2dv)(const GLdouble * v);
#define glWindowPos2dv GL15__ptrc_glWindowPos2dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2f)(GLfloat x, GLfloat y);
#define glWindowPos2f GL15__ptrc_glWindowPos2f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2fv)(const GLfloat * v);
#define glWindowPos2fv GL15__ptrc_glWindowPos2fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2i)(GLint x, GLint y);
#define glWindowPos2i GL15__ptrc_glWindowPos2i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2iv)(const GLint * v);
#define glWindowPos2iv GL15__ptrc_glWindowPos2iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2s)(GLshort x, GLshort y);
#define glWindowPos2s GL15__ptrc_glWindowPos2s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2sv)(const GLshort * v);
#define glWindowPos2sv GL15__ptrc_glWindowPos2sv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3d)(GLdouble x, GLdouble y, GLdouble z);
#define glWindowPos3d GL15__ptrc_glWindowPos3d
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3dv)(const GLdouble * v);
#define glWindowPos3dv GL15__ptrc_glWindowPos3dv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3f)(GLfloat x, GLfloat y, GLfloat z);
#define glWindowPos3f GL15__ptrc_glWindowPos3f
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3fv)(const GLfloat * v);
#define glWindowPos3fv GL15__ptrc_glWindowPos3fv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3i)(GLint x, GLint y, GLint z);
#define glWindowPos3i GL15__ptrc_glWindowPos3i
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3iv)(const GLint * v);
#define glWindowPos3iv GL15__ptrc_glWindowPos3iv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3s)(GLshort x, GLshort y, GLshort z);
#define glWindowPos3s GL15__ptrc_glWindowPos3s
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3sv)(const GLshort * v);
#define glWindowPos3sv GL15__ptrc_glWindowPos3sv

extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBeginQuery)(GLenum target, GLuint id);
#define glBeginQuery GL15__ptrc_glBeginQuery
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBindBuffer)(GLenum target, GLuint buffer);
#define glBindBuffer GL15__ptrc_glBindBuffer
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBufferData)(GLenum target, GLsizeiptr size, const void * data, GLenum usage);
#define glBufferData GL15__ptrc_glBufferData
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glBufferSubData)(GLenum target, GLintptr offset, GLsizeiptr size, const void * data);
#define glBufferSubData GL15__ptrc_glBufferSubData
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDeleteBuffers)(GLsizei n, const GLuint * buffers);
#define glDeleteBuffers GL15__ptrc_glDeleteBuffers
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glDeleteQueries)(GLsizei n, const GLuint * ids);
#define glDeleteQueries GL15__ptrc_glDeleteQueries
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glEndQuery)(GLenum target);
#define glEndQuery GL15__ptrc_glEndQuery
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGenBuffers)(GLsizei n, GLuint * buffers);
#define glGenBuffers GL15__ptrc_glGenBuffers
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGenQueries)(GLsizei n, GLuint * ids);
#define glGenQueries GL15__ptrc_glGenQueries
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetBufferParameteriv)(GLenum target, GLenum pname, GLint * params);
#define glGetBufferParameteriv GL15__ptrc_glGetBufferParameteriv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetBufferPointerv)(GLenum target, GLenum pname, void ** params);
#define glGetBufferPointerv GL15__ptrc_glGetBufferPointerv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetBufferSubData)(GLenum target, GLintptr offset, GLsizeiptr size, void * data);
#define glGetBufferSubData GL15__ptrc_glGetBufferSubData
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetQueryObjectiv)(GLuint id, GLenum pname, GLint * params);
#define glGetQueryObjectiv GL15__ptrc_glGetQueryObjectiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetQueryObjectuiv)(GLuint id, GLenum pname, GLuint * params);
#define glGetQueryObjectuiv GL15__ptrc_glGetQueryObjectuiv
extern void (CODEGEN_FUNCPTR *GL15__ptrc_glGetQueryiv)(GLenum target, GLenum pname, GLint * params);
#define glGetQueryiv GL15__ptrc_glGetQueryiv
extern GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsBuffer)(GLuint buffer);
#define glIsBuffer GL15__ptrc_glIsBuffer
extern GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsQuery)(GLuint id);
#define glIsQuery GL15__ptrc_glIsQuery
extern void * (CODEGEN_FUNCPTR *GL15__ptrc_glMapBuffer)(GLenum target, GLenum access);
#define glMapBuffer GL15__ptrc_glMapBuffer
extern GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glUnmapBuffer)(GLenum target);
#define glUnmapBuffer GL15__ptrc_glUnmapBuffer

enum GL15_ogl_LoadStatus
{
	GL15_ogl_LOAD_FAILED = 0,
	GL15_ogl_LOAD_SUCCEEDED = 1,
};

int GL15_ogl_LoadFunctions();

int GL15_ogl_GetMinorVersion(void);
int GL15_ogl_GetMajorVersion(void);
int GL15_ogl_IsVersionGEQ(int majorVersion, int minorVersion);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*GL15__POINTER_C_GENERATED_HEADER_OPENGL_H*/
