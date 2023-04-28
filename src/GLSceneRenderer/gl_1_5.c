#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "gl_1_5.h"

#if defined(__APPLE__)
#include <dlfcn.h>

static void* AppleGLGetProcAddress (const char *name)
{
	static void* image = NULL;
	
	if (NULL == image)
		image = dlopen("/System/Library/Frameworks/OpenGL.framework/Versions/Current/OpenGL", RTLD_LAZY);

	return (image ? dlsym(image, name) : NULL);
}
#endif /* __APPLE__ */

#if defined(__sgi) || defined (__sun)
#include <dlfcn.h>
#include <stdio.h>

static void* SunGetProcAddress (const GLubyte* name)
{
  static void* h = NULL;
  static void* gpa;

  if (h == NULL)
  {
    if ((h = dlopen(NULL, RTLD_LAZY | RTLD_LOCAL)) == NULL) return NULL;
    gpa = dlsym(h, "glXGetProcAddress");
  }

  if (gpa != NULL)
    return ((void*(*)(const GLubyte*))gpa)(name);
  else
    return dlsym(h, (const char*)name);
}
#endif /* __sgi || __sun */

#if defined(_WIN32)

#ifdef _MSC_VER
#pragma warning(disable: 4055)
#pragma warning(disable: 4054)
#pragma warning(disable: 4996)
#endif

static int TestPointer(const PROC pTest)
{
	ptrdiff_t iTest;
	if(!pTest) return 0;
	iTest = (ptrdiff_t)pTest;
	
	if(iTest == 1 || iTest == 2 || iTest == 3 || iTest == -1) return 0;
	
	return 1;
}

static PROC WinGetProcAddress(const char *name)
{
	HMODULE glMod = NULL;
	PROC pFunc = wglGetProcAddress((LPCSTR)name);
	if(TestPointer(pFunc))
	{
		return pFunc;
	}
	glMod = GetModuleHandleA("OpenGL32.dll");
	return (PROC)GetProcAddress(glMod, (LPCSTR)name);
}
	
#define IntGetProcAddress(name) WinGetProcAddress(name)
#else
	#if defined(__APPLE__)
		#define IntGetProcAddress(name) AppleGLGetProcAddress(name)
	#else
		#if defined(__sgi) || defined(__sun)
			#define IntGetProcAddress(name) SunGetProcAddress(name)
		#else /* GLX */
		    #include <GL/glx.h>

			#define IntGetProcAddress(name) (*glXGetProcAddressARB)((const GLubyte*)name)
		#endif
	#endif
#endif

void (CODEGEN_FUNCPTR *GL15__ptrc_glAccum)(GLenum op, GLfloat value) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glAlphaFunc)(GLenum func, GLfloat ref) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glBegin)(GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glBitmap)(GLsizei width, GLsizei height, GLfloat xorig, GLfloat yorig, GLfloat xmove, GLfloat ymove, const GLubyte * bitmap) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glBlendFunc)(GLenum sfactor, GLenum dfactor) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCallList)(GLuint list) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCallLists)(GLsizei n, GLenum type, const void * lists) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glClear)(GLbitfield mask) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glClearAccum)(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glClearColor)(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glClearDepth)(GLdouble depth) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glClearIndex)(GLfloat c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glClearStencil)(GLint s) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glClipPlane)(GLenum plane, const GLdouble * equation) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3b)(GLbyte red, GLbyte green, GLbyte blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3bv)(const GLbyte * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3d)(GLdouble red, GLdouble green, GLdouble blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3f)(GLfloat red, GLfloat green, GLfloat blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3i)(GLint red, GLint green, GLint blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3s)(GLshort red, GLshort green, GLshort blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3ub)(GLubyte red, GLubyte green, GLubyte blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3ubv)(const GLubyte * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3ui)(GLuint red, GLuint green, GLuint blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3uiv)(const GLuint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3us)(GLushort red, GLushort green, GLushort blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor3usv)(const GLushort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4b)(GLbyte red, GLbyte green, GLbyte blue, GLbyte alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4bv)(const GLbyte * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4d)(GLdouble red, GLdouble green, GLdouble blue, GLdouble alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4f)(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4i)(GLint red, GLint green, GLint blue, GLint alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4s)(GLshort red, GLshort green, GLshort blue, GLshort alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4ub)(GLubyte red, GLubyte green, GLubyte blue, GLubyte alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4ubv)(const GLubyte * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4ui)(GLuint red, GLuint green, GLuint blue, GLuint alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4uiv)(const GLuint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4us)(GLushort red, GLushort green, GLushort blue, GLushort alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColor4usv)(const GLushort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColorMask)(GLboolean red, GLboolean green, GLboolean blue, GLboolean alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColorMaterial)(GLenum face, GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyPixels)(GLint x, GLint y, GLsizei width, GLsizei height, GLenum type) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCullFace)(GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDeleteLists)(GLuint list, GLsizei range) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDepthFunc)(GLenum func) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDepthMask)(GLboolean flag) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDepthRange)(GLdouble ren_near, GLdouble ren_far) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDisable)(GLenum cap) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawBuffer)(GLenum buf) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawPixels)(GLsizei width, GLsizei height, GLenum format, GLenum type, const void * pixels) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEdgeFlag)(GLboolean flag) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEdgeFlagv)(const GLboolean * flag) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEnable)(GLenum cap) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEnd)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEndList)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord1d)(GLdouble u) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord1dv)(const GLdouble * u) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord1f)(GLfloat u) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord1fv)(const GLfloat * u) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord2d)(GLdouble u, GLdouble v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord2dv)(const GLdouble * u) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord2f)(GLfloat u, GLfloat v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalCoord2fv)(const GLfloat * u) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalMesh1)(GLenum mode, GLint i1, GLint i2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalMesh2)(GLenum mode, GLint i1, GLint i2, GLint j1, GLint j2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalPoint1)(GLint i) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEvalPoint2)(GLint i, GLint j) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFeedbackBuffer)(GLsizei size, GLenum type, GLfloat * buffer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFinish)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFlush)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFogf)(GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFogfv)(GLenum pname, const GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFogi)(GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFogiv)(GLenum pname, const GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFrontFace)(GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFrustum)(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top, GLdouble zNear, GLdouble zFar) = NULL;
GLuint (CODEGEN_FUNCPTR *GL15__ptrc_glGenLists)(GLsizei range) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetBooleanv)(GLenum pname, GLboolean * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetClipPlane)(GLenum plane, GLdouble * equation) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetDoublev)(GLenum pname, GLdouble * data) = NULL;
GLenum (CODEGEN_FUNCPTR *GL15__ptrc_glGetError)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetFloatv)(GLenum pname, GLfloat * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetIntegerv)(GLenum pname, GLint * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetLightfv)(GLenum light, GLenum pname, GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetLightiv)(GLenum light, GLenum pname, GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMapdv)(GLenum target, GLenum query, GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMapfv)(GLenum target, GLenum query, GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMapiv)(GLenum target, GLenum query, GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMaterialfv)(GLenum face, GLenum pname, GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetMaterialiv)(GLenum face, GLenum pname, GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPixelMapfv)(GLenum map, GLfloat * values) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPixelMapuiv)(GLenum map, GLuint * values) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPixelMapusv)(GLenum map, GLushort * values) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPolygonStipple)(GLubyte * mask) = NULL;
const GLubyte * (CODEGEN_FUNCPTR *GL15__ptrc_glGetString)(GLenum name) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexEnvfv)(GLenum target, GLenum pname, GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexEnviv)(GLenum target, GLenum pname, GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexGendv)(GLenum coord, GLenum pname, GLdouble * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexGenfv)(GLenum coord, GLenum pname, GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexGeniv)(GLenum coord, GLenum pname, GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexImage)(GLenum target, GLint level, GLenum format, GLenum type, void * pixels) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexLevelParameterfv)(GLenum target, GLint level, GLenum pname, GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexLevelParameteriv)(GLenum target, GLint level, GLenum pname, GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexParameterfv)(GLenum target, GLenum pname, GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetTexParameteriv)(GLenum target, GLenum pname, GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glHint)(GLenum target, GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexMask)(GLuint mask) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexd)(GLdouble c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexdv)(const GLdouble * c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexf)(GLfloat c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexfv)(const GLfloat * c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexi)(GLint c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexiv)(const GLint * c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexs)(GLshort c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexsv)(const GLshort * c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glInitNames)(void) = NULL;
GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsEnabled)(GLenum cap) = NULL;
GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsList)(GLuint list) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLightModelf)(GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLightModelfv)(GLenum pname, const GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLightModeli)(GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLightModeliv)(GLenum pname, const GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLightf)(GLenum light, GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLightfv)(GLenum light, GLenum pname, const GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLighti)(GLenum light, GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLightiv)(GLenum light, GLenum pname, const GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLineStipple)(GLint factor, GLushort pattern) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLineWidth)(GLfloat width) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glListBase)(GLuint base) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadIdentity)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadMatrixd)(const GLdouble * m) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadMatrixf)(const GLfloat * m) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadName)(GLuint name) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLogicOp)(GLenum opcode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMap1d)(GLenum target, GLdouble u1, GLdouble u2, GLint stride, GLint order, const GLdouble * points) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMap1f)(GLenum target, GLfloat u1, GLfloat u2, GLint stride, GLint order, const GLfloat * points) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMap2d)(GLenum target, GLdouble u1, GLdouble u2, GLint ustride, GLint uorder, GLdouble v1, GLdouble v2, GLint vstride, GLint vorder, const GLdouble * points) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMap2f)(GLenum target, GLfloat u1, GLfloat u2, GLint ustride, GLint uorder, GLfloat v1, GLfloat v2, GLint vstride, GLint vorder, const GLfloat * points) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMapGrid1d)(GLint un, GLdouble u1, GLdouble u2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMapGrid1f)(GLint un, GLfloat u1, GLfloat u2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMapGrid2d)(GLint un, GLdouble u1, GLdouble u2, GLint vn, GLdouble v1, GLdouble v2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMapGrid2f)(GLint un, GLfloat u1, GLfloat u2, GLint vn, GLfloat v1, GLfloat v2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMaterialf)(GLenum face, GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMaterialfv)(GLenum face, GLenum pname, const GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMateriali)(GLenum face, GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMaterialiv)(GLenum face, GLenum pname, const GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMatrixMode)(GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultMatrixd)(const GLdouble * m) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultMatrixf)(const GLfloat * m) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNewList)(GLuint list, GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3b)(GLbyte nx, GLbyte ny, GLbyte nz) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3bv)(const GLbyte * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3d)(GLdouble nx, GLdouble ny, GLdouble nz) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3f)(GLfloat nx, GLfloat ny, GLfloat nz) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3i)(GLint nx, GLint ny, GLint nz) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3s)(GLshort nx, GLshort ny, GLshort nz) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormal3sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glOrtho)(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top, GLdouble zNear, GLdouble zFar) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPassThrough)(GLfloat token) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelMapfv)(GLenum map, GLsizei mapsize, const GLfloat * values) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelMapuiv)(GLenum map, GLsizei mapsize, const GLuint * values) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelMapusv)(GLenum map, GLsizei mapsize, const GLushort * values) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelStoref)(GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelStorei)(GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelTransferf)(GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelTransferi)(GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPixelZoom)(GLfloat xfactor, GLfloat yfactor) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPointSize)(GLfloat size) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPolygonMode)(GLenum face, GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPolygonStipple)(const GLubyte * mask) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPopAttrib)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPopMatrix)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPopName)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPushAttrib)(GLbitfield mask) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPushMatrix)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPushName)(GLuint name) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2d)(GLdouble x, GLdouble y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2f)(GLfloat x, GLfloat y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2i)(GLint x, GLint y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2s)(GLshort x, GLshort y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos2sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3d)(GLdouble x, GLdouble y, GLdouble z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3f)(GLfloat x, GLfloat y, GLfloat z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3i)(GLint x, GLint y, GLint z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3s)(GLshort x, GLshort y, GLshort z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos3sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4d)(GLdouble x, GLdouble y, GLdouble z, GLdouble w) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4f)(GLfloat x, GLfloat y, GLfloat z, GLfloat w) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4i)(GLint x, GLint y, GLint z, GLint w) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4s)(GLshort x, GLshort y, GLshort z, GLshort w) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRasterPos4sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glReadBuffer)(GLenum src) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glReadPixels)(GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, void * pixels) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRectd)(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRectdv)(const GLdouble * v1, const GLdouble * v2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRectf)(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRectfv)(const GLfloat * v1, const GLfloat * v2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRecti)(GLint x1, GLint y1, GLint x2, GLint y2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRectiv)(const GLint * v1, const GLint * v2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRects)(GLshort x1, GLshort y1, GLshort x2, GLshort y2) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRectsv)(const GLshort * v1, const GLshort * v2) = NULL;
GLint (CODEGEN_FUNCPTR *GL15__ptrc_glRenderMode)(GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRotated)(GLdouble angle, GLdouble x, GLdouble y, GLdouble z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glRotatef)(GLfloat angle, GLfloat x, GLfloat y, GLfloat z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glScaled)(GLdouble x, GLdouble y, GLdouble z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glScalef)(GLfloat x, GLfloat y, GLfloat z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glScissor)(GLint x, GLint y, GLsizei width, GLsizei height) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSelectBuffer)(GLsizei size, GLuint * buffer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glShadeModel)(GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glStencilFunc)(GLenum func, GLint ref, GLuint mask) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glStencilMask)(GLuint mask) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glStencilOp)(GLenum fail, GLenum zfail, GLenum zpass) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1d)(GLdouble s) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1f)(GLfloat s) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1i)(GLint s) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1s)(GLshort s) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord1sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2d)(GLdouble s, GLdouble t) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2f)(GLfloat s, GLfloat t) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2i)(GLint s, GLint t) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2s)(GLshort s, GLshort t) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord2sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3d)(GLdouble s, GLdouble t, GLdouble r) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3f)(GLfloat s, GLfloat t, GLfloat r) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3i)(GLint s, GLint t, GLint r) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3s)(GLshort s, GLshort t, GLshort r) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord3sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4d)(GLdouble s, GLdouble t, GLdouble r, GLdouble q) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4f)(GLfloat s, GLfloat t, GLfloat r, GLfloat q) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4i)(GLint s, GLint t, GLint r, GLint q) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4s)(GLshort s, GLshort t, GLshort r, GLshort q) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoord4sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexEnvf)(GLenum target, GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexEnvfv)(GLenum target, GLenum pname, const GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexEnvi)(GLenum target, GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexEnviv)(GLenum target, GLenum pname, const GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGend)(GLenum coord, GLenum pname, GLdouble param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGendv)(GLenum coord, GLenum pname, const GLdouble * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGenf)(GLenum coord, GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGenfv)(GLenum coord, GLenum pname, const GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGeni)(GLenum coord, GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexGeniv)(GLenum coord, GLenum pname, const GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexImage1D)(GLenum target, GLint level, GLint internalformat, GLsizei width, GLint border, GLenum format, GLenum type, const void * pixels) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexImage2D)(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const void * pixels) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexParameterf)(GLenum target, GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexParameterfv)(GLenum target, GLenum pname, const GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexParameteri)(GLenum target, GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexParameteriv)(GLenum target, GLenum pname, const GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTranslated)(GLdouble x, GLdouble y, GLdouble z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTranslatef)(GLfloat x, GLfloat y, GLfloat z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2d)(GLdouble x, GLdouble y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2f)(GLfloat x, GLfloat y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2i)(GLint x, GLint y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2s)(GLshort x, GLshort y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex2sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3d)(GLdouble x, GLdouble y, GLdouble z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3f)(GLfloat x, GLfloat y, GLfloat z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3i)(GLint x, GLint y, GLint z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3s)(GLshort x, GLshort y, GLshort z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex3sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4d)(GLdouble x, GLdouble y, GLdouble z, GLdouble w) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4f)(GLfloat x, GLfloat y, GLfloat z, GLfloat w) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4i)(GLint x, GLint y, GLint z, GLint w) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4s)(GLshort x, GLshort y, GLshort z, GLshort w) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertex4sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glViewport)(GLint x, GLint y, GLsizei width, GLsizei height) = NULL;

GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glAreTexturesResident)(GLsizei n, const GLuint * textures, GLboolean * residences) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glArrayElement)(GLint i) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glBindTexture)(GLenum target, GLuint texture) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glColorPointer)(GLint size, GLenum type, GLsizei stride, const void * pointer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexImage1D)(GLenum target, GLint level, GLenum internalformat, GLint x, GLint y, GLsizei width, GLint border) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexImage2D)(GLenum target, GLint level, GLenum internalformat, GLint x, GLint y, GLsizei width, GLsizei height, GLint border) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexSubImage1D)(GLenum target, GLint level, GLint xoffset, GLint x, GLint y, GLsizei width) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexSubImage2D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint x, GLint y, GLsizei width, GLsizei height) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDeleteTextures)(GLsizei n, const GLuint * textures) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDisableClientState)(GLenum ren_array) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawArrays)(GLenum mode, GLint first, GLsizei count) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawElements)(GLenum mode, GLsizei count, GLenum type, const void * indices) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEdgeFlagPointer)(GLsizei stride, const void * pointer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEnableClientState)(GLenum ren_array) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGenTextures)(GLsizei n, GLuint * textures) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetPointerv)(GLenum pname, void ** params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexPointer)(GLenum type, GLsizei stride, const void * pointer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexub)(GLubyte c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glIndexubv)(const GLubyte * c) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glInterleavedArrays)(GLenum format, GLsizei stride, const void * pointer) = NULL;
GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsTexture)(GLuint texture) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glNormalPointer)(GLenum type, GLsizei stride, const void * pointer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPolygonOffset)(GLfloat factor, GLfloat units) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPopClientAttrib)(void) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPrioritizeTextures)(GLsizei n, const GLuint * textures, const GLfloat * priorities) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPushClientAttrib)(GLbitfield mask) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexCoordPointer)(GLint size, GLenum type, GLsizei stride, const void * pointer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexSubImage1D)(GLenum target, GLint level, GLint xoffset, GLsizei width, GLenum format, GLenum type, const void * pixels) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexSubImage2D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum format, GLenum type, const void * pixels) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glVertexPointer)(GLint size, GLenum type, GLsizei stride, const void * pointer) = NULL;

void (CODEGEN_FUNCPTR *GL15__ptrc_glCopyTexSubImage3D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLint x, GLint y, GLsizei width, GLsizei height) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDrawRangeElements)(GLenum mode, GLuint start, GLuint end, GLsizei count, GLenum type, const void * indices) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexImage3D)(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, const void * pixels) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glTexSubImage3D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLenum type, const void * pixels) = NULL;

void (CODEGEN_FUNCPTR *GL15__ptrc_glActiveTexture)(GLenum texture) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glClientActiveTexture)(GLenum texture) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexImage1D)(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLint border, GLsizei imageSize, const void * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexImage2D)(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLint border, GLsizei imageSize, const void * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexImage3D)(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLsizei imageSize, const void * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexSubImage1D)(GLenum target, GLint level, GLint xoffset, GLsizei width, GLenum format, GLsizei imageSize, const void * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexSubImage2D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum format, GLsizei imageSize, const void * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glCompressedTexSubImage3D)(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLsizei imageSize, const void * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetCompressedTexImage)(GLenum target, GLint level, void * img) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadTransposeMatrixd)(const GLdouble * m) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glLoadTransposeMatrixf)(const GLfloat * m) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultTransposeMatrixd)(const GLdouble * m) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultTransposeMatrixf)(const GLfloat * m) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1d)(GLenum target, GLdouble s) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1dv)(GLenum target, const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1f)(GLenum target, GLfloat s) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1fv)(GLenum target, const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1i)(GLenum target, GLint s) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1iv)(GLenum target, const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1s)(GLenum target, GLshort s) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord1sv)(GLenum target, const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2d)(GLenum target, GLdouble s, GLdouble t) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2dv)(GLenum target, const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2f)(GLenum target, GLfloat s, GLfloat t) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2fv)(GLenum target, const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2i)(GLenum target, GLint s, GLint t) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2iv)(GLenum target, const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2s)(GLenum target, GLshort s, GLshort t) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord2sv)(GLenum target, const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3d)(GLenum target, GLdouble s, GLdouble t, GLdouble r) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3dv)(GLenum target, const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3f)(GLenum target, GLfloat s, GLfloat t, GLfloat r) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3fv)(GLenum target, const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3i)(GLenum target, GLint s, GLint t, GLint r) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3iv)(GLenum target, const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3s)(GLenum target, GLshort s, GLshort t, GLshort r) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord3sv)(GLenum target, const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4d)(GLenum target, GLdouble s, GLdouble t, GLdouble r, GLdouble q) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4dv)(GLenum target, const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4f)(GLenum target, GLfloat s, GLfloat t, GLfloat r, GLfloat q) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4fv)(GLenum target, const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4i)(GLenum target, GLint s, GLint t, GLint r, GLint q) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4iv)(GLenum target, const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4s)(GLenum target, GLshort s, GLshort t, GLshort r, GLshort q) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiTexCoord4sv)(GLenum target, const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSampleCoverage)(GLfloat value, GLboolean invert) = NULL;

void (CODEGEN_FUNCPTR *GL15__ptrc_glBlendColor)(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glBlendEquation)(GLenum mode) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glBlendFuncSeparate)(GLenum sfactorRGB, GLenum dfactorRGB, GLenum sfactorAlpha, GLenum dfactorAlpha) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoordPointer)(GLenum type, GLsizei stride, const void * pointer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoordd)(GLdouble coord) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoorddv)(const GLdouble * coord) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoordf)(GLfloat coord) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glFogCoordfv)(const GLfloat * coord) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiDrawArrays)(GLenum mode, const GLint * first, const GLsizei * count, GLsizei drawcount) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glMultiDrawElements)(GLenum mode, const GLsizei * count, GLenum type, const void *const* indices, GLsizei drawcount) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPointParameterf)(GLenum pname, GLfloat param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPointParameterfv)(GLenum pname, const GLfloat * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPointParameteri)(GLenum pname, GLint param) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glPointParameteriv)(GLenum pname, const GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3b)(GLbyte red, GLbyte green, GLbyte blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3bv)(const GLbyte * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3d)(GLdouble red, GLdouble green, GLdouble blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3f)(GLfloat red, GLfloat green, GLfloat blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3i)(GLint red, GLint green, GLint blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3s)(GLshort red, GLshort green, GLshort blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3ub)(GLubyte red, GLubyte green, GLubyte blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3ubv)(const GLubyte * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3ui)(GLuint red, GLuint green, GLuint blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3uiv)(const GLuint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3us)(GLushort red, GLushort green, GLushort blue) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColor3usv)(const GLushort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glSecondaryColorPointer)(GLint size, GLenum type, GLsizei stride, const void * pointer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2d)(GLdouble x, GLdouble y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2f)(GLfloat x, GLfloat y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2i)(GLint x, GLint y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2s)(GLshort x, GLshort y) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos2sv)(const GLshort * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3d)(GLdouble x, GLdouble y, GLdouble z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3dv)(const GLdouble * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3f)(GLfloat x, GLfloat y, GLfloat z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3fv)(const GLfloat * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3i)(GLint x, GLint y, GLint z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3iv)(const GLint * v) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3s)(GLshort x, GLshort y, GLshort z) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glWindowPos3sv)(const GLshort * v) = NULL;

void (CODEGEN_FUNCPTR *GL15__ptrc_glBeginQuery)(GLenum target, GLuint id) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glBindBuffer)(GLenum target, GLuint buffer) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glBufferData)(GLenum target, GLsizeiptr size, const void * data, GLenum usage) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glBufferSubData)(GLenum target, GLintptr offset, GLsizeiptr size, const void * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDeleteBuffers)(GLsizei n, const GLuint * buffers) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glDeleteQueries)(GLsizei n, const GLuint * ids) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glEndQuery)(GLenum target) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGenBuffers)(GLsizei n, GLuint * buffers) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGenQueries)(GLsizei n, GLuint * ids) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetBufferParameteriv)(GLenum target, GLenum pname, GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetBufferPointerv)(GLenum target, GLenum pname, void ** params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetBufferSubData)(GLenum target, GLintptr offset, GLsizeiptr size, void * data) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetQueryObjectiv)(GLuint id, GLenum pname, GLint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetQueryObjectuiv)(GLuint id, GLenum pname, GLuint * params) = NULL;
void (CODEGEN_FUNCPTR *GL15__ptrc_glGetQueryiv)(GLenum target, GLenum pname, GLint * params) = NULL;
GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsBuffer)(GLuint buffer) = NULL;
GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glIsQuery)(GLuint id) = NULL;
void * (CODEGEN_FUNCPTR *GL15__ptrc_glMapBuffer)(GLenum target, GLenum access) = NULL;
GLboolean (CODEGEN_FUNCPTR *GL15__ptrc_glUnmapBuffer)(GLenum target) = NULL;

static int Load_Version_1_5(void)
{
	int numFailed = 0;
	GL15__ptrc_glAccum = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat))IntGetProcAddress("glAccum");
	if(!GL15__ptrc_glAccum) numFailed++;
	GL15__ptrc_glAlphaFunc = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat))IntGetProcAddress("glAlphaFunc");
	if(!GL15__ptrc_glAlphaFunc) numFailed++;
	GL15__ptrc_glBegin = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glBegin");
	if(!GL15__ptrc_glBegin) numFailed++;
	GL15__ptrc_glBitmap = (void (CODEGEN_FUNCPTR *)(GLsizei, GLsizei, GLfloat, GLfloat, GLfloat, GLfloat, const GLubyte *))IntGetProcAddress("glBitmap");
	if(!GL15__ptrc_glBitmap) numFailed++;
	GL15__ptrc_glBlendFunc = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum))IntGetProcAddress("glBlendFunc");
	if(!GL15__ptrc_glBlendFunc) numFailed++;
	GL15__ptrc_glCallList = (void (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glCallList");
	if(!GL15__ptrc_glCallList) numFailed++;
	GL15__ptrc_glCallLists = (void (CODEGEN_FUNCPTR *)(GLsizei, GLenum, const void *))IntGetProcAddress("glCallLists");
	if(!GL15__ptrc_glCallLists) numFailed++;
	GL15__ptrc_glClear = (void (CODEGEN_FUNCPTR *)(GLbitfield))IntGetProcAddress("glClear");
	if(!GL15__ptrc_glClear) numFailed++;
	GL15__ptrc_glClearAccum = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glClearAccum");
	if(!GL15__ptrc_glClearAccum) numFailed++;
	GL15__ptrc_glClearColor = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glClearColor");
	if(!GL15__ptrc_glClearColor) numFailed++;
	GL15__ptrc_glClearDepth = (void (CODEGEN_FUNCPTR *)(GLdouble))IntGetProcAddress("glClearDepth");
	if(!GL15__ptrc_glClearDepth) numFailed++;
	GL15__ptrc_glClearIndex = (void (CODEGEN_FUNCPTR *)(GLfloat))IntGetProcAddress("glClearIndex");
	if(!GL15__ptrc_glClearIndex) numFailed++;
	GL15__ptrc_glClearStencil = (void (CODEGEN_FUNCPTR *)(GLint))IntGetProcAddress("glClearStencil");
	if(!GL15__ptrc_glClearStencil) numFailed++;
	GL15__ptrc_glClipPlane = (void (CODEGEN_FUNCPTR *)(GLenum, const GLdouble *))IntGetProcAddress("glClipPlane");
	if(!GL15__ptrc_glClipPlane) numFailed++;
	GL15__ptrc_glColor3b = (void (CODEGEN_FUNCPTR *)(GLbyte, GLbyte, GLbyte))IntGetProcAddress("glColor3b");
	if(!GL15__ptrc_glColor3b) numFailed++;
	GL15__ptrc_glColor3bv = (void (CODEGEN_FUNCPTR *)(const GLbyte *))IntGetProcAddress("glColor3bv");
	if(!GL15__ptrc_glColor3bv) numFailed++;
	GL15__ptrc_glColor3d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble))IntGetProcAddress("glColor3d");
	if(!GL15__ptrc_glColor3d) numFailed++;
	GL15__ptrc_glColor3dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glColor3dv");
	if(!GL15__ptrc_glColor3dv) numFailed++;
	GL15__ptrc_glColor3f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat))IntGetProcAddress("glColor3f");
	if(!GL15__ptrc_glColor3f) numFailed++;
	GL15__ptrc_glColor3fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glColor3fv");
	if(!GL15__ptrc_glColor3fv) numFailed++;
	GL15__ptrc_glColor3i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint))IntGetProcAddress("glColor3i");
	if(!GL15__ptrc_glColor3i) numFailed++;
	GL15__ptrc_glColor3iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glColor3iv");
	if(!GL15__ptrc_glColor3iv) numFailed++;
	GL15__ptrc_glColor3s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort))IntGetProcAddress("glColor3s");
	if(!GL15__ptrc_glColor3s) numFailed++;
	GL15__ptrc_glColor3sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glColor3sv");
	if(!GL15__ptrc_glColor3sv) numFailed++;
	GL15__ptrc_glColor3ub = (void (CODEGEN_FUNCPTR *)(GLubyte, GLubyte, GLubyte))IntGetProcAddress("glColor3ub");
	if(!GL15__ptrc_glColor3ub) numFailed++;
	GL15__ptrc_glColor3ubv = (void (CODEGEN_FUNCPTR *)(const GLubyte *))IntGetProcAddress("glColor3ubv");
	if(!GL15__ptrc_glColor3ubv) numFailed++;
	GL15__ptrc_glColor3ui = (void (CODEGEN_FUNCPTR *)(GLuint, GLuint, GLuint))IntGetProcAddress("glColor3ui");
	if(!GL15__ptrc_glColor3ui) numFailed++;
	GL15__ptrc_glColor3uiv = (void (CODEGEN_FUNCPTR *)(const GLuint *))IntGetProcAddress("glColor3uiv");
	if(!GL15__ptrc_glColor3uiv) numFailed++;
	GL15__ptrc_glColor3us = (void (CODEGEN_FUNCPTR *)(GLushort, GLushort, GLushort))IntGetProcAddress("glColor3us");
	if(!GL15__ptrc_glColor3us) numFailed++;
	GL15__ptrc_glColor3usv = (void (CODEGEN_FUNCPTR *)(const GLushort *))IntGetProcAddress("glColor3usv");
	if(!GL15__ptrc_glColor3usv) numFailed++;
	GL15__ptrc_glColor4b = (void (CODEGEN_FUNCPTR *)(GLbyte, GLbyte, GLbyte, GLbyte))IntGetProcAddress("glColor4b");
	if(!GL15__ptrc_glColor4b) numFailed++;
	GL15__ptrc_glColor4bv = (void (CODEGEN_FUNCPTR *)(const GLbyte *))IntGetProcAddress("glColor4bv");
	if(!GL15__ptrc_glColor4bv) numFailed++;
	GL15__ptrc_glColor4d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glColor4d");
	if(!GL15__ptrc_glColor4d) numFailed++;
	GL15__ptrc_glColor4dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glColor4dv");
	if(!GL15__ptrc_glColor4dv) numFailed++;
	GL15__ptrc_glColor4f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glColor4f");
	if(!GL15__ptrc_glColor4f) numFailed++;
	GL15__ptrc_glColor4fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glColor4fv");
	if(!GL15__ptrc_glColor4fv) numFailed++;
	GL15__ptrc_glColor4i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint, GLint))IntGetProcAddress("glColor4i");
	if(!GL15__ptrc_glColor4i) numFailed++;
	GL15__ptrc_glColor4iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glColor4iv");
	if(!GL15__ptrc_glColor4iv) numFailed++;
	GL15__ptrc_glColor4s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort, GLshort))IntGetProcAddress("glColor4s");
	if(!GL15__ptrc_glColor4s) numFailed++;
	GL15__ptrc_glColor4sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glColor4sv");
	if(!GL15__ptrc_glColor4sv) numFailed++;
	GL15__ptrc_glColor4ub = (void (CODEGEN_FUNCPTR *)(GLubyte, GLubyte, GLubyte, GLubyte))IntGetProcAddress("glColor4ub");
	if(!GL15__ptrc_glColor4ub) numFailed++;
	GL15__ptrc_glColor4ubv = (void (CODEGEN_FUNCPTR *)(const GLubyte *))IntGetProcAddress("glColor4ubv");
	if(!GL15__ptrc_glColor4ubv) numFailed++;
	GL15__ptrc_glColor4ui = (void (CODEGEN_FUNCPTR *)(GLuint, GLuint, GLuint, GLuint))IntGetProcAddress("glColor4ui");
	if(!GL15__ptrc_glColor4ui) numFailed++;
	GL15__ptrc_glColor4uiv = (void (CODEGEN_FUNCPTR *)(const GLuint *))IntGetProcAddress("glColor4uiv");
	if(!GL15__ptrc_glColor4uiv) numFailed++;
	GL15__ptrc_glColor4us = (void (CODEGEN_FUNCPTR *)(GLushort, GLushort, GLushort, GLushort))IntGetProcAddress("glColor4us");
	if(!GL15__ptrc_glColor4us) numFailed++;
	GL15__ptrc_glColor4usv = (void (CODEGEN_FUNCPTR *)(const GLushort *))IntGetProcAddress("glColor4usv");
	if(!GL15__ptrc_glColor4usv) numFailed++;
	GL15__ptrc_glColorMask = (void (CODEGEN_FUNCPTR *)(GLboolean, GLboolean, GLboolean, GLboolean))IntGetProcAddress("glColorMask");
	if(!GL15__ptrc_glColorMask) numFailed++;
	GL15__ptrc_glColorMaterial = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum))IntGetProcAddress("glColorMaterial");
	if(!GL15__ptrc_glColorMaterial) numFailed++;
	GL15__ptrc_glCopyPixels = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLsizei, GLsizei, GLenum))IntGetProcAddress("glCopyPixels");
	if(!GL15__ptrc_glCopyPixels) numFailed++;
	GL15__ptrc_glCullFace = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glCullFace");
	if(!GL15__ptrc_glCullFace) numFailed++;
	GL15__ptrc_glDeleteLists = (void (CODEGEN_FUNCPTR *)(GLuint, GLsizei))IntGetProcAddress("glDeleteLists");
	if(!GL15__ptrc_glDeleteLists) numFailed++;
	GL15__ptrc_glDepthFunc = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glDepthFunc");
	if(!GL15__ptrc_glDepthFunc) numFailed++;
	GL15__ptrc_glDepthMask = (void (CODEGEN_FUNCPTR *)(GLboolean))IntGetProcAddress("glDepthMask");
	if(!GL15__ptrc_glDepthMask) numFailed++;
	GL15__ptrc_glDepthRange = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble))IntGetProcAddress("glDepthRange");
	if(!GL15__ptrc_glDepthRange) numFailed++;
	GL15__ptrc_glDisable = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glDisable");
	if(!GL15__ptrc_glDisable) numFailed++;
	GL15__ptrc_glDrawBuffer = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glDrawBuffer");
	if(!GL15__ptrc_glDrawBuffer) numFailed++;
	GL15__ptrc_glDrawPixels = (void (CODEGEN_FUNCPTR *)(GLsizei, GLsizei, GLenum, GLenum, const void *))IntGetProcAddress("glDrawPixels");
	if(!GL15__ptrc_glDrawPixels) numFailed++;
	GL15__ptrc_glEdgeFlag = (void (CODEGEN_FUNCPTR *)(GLboolean))IntGetProcAddress("glEdgeFlag");
	if(!GL15__ptrc_glEdgeFlag) numFailed++;
	GL15__ptrc_glEdgeFlagv = (void (CODEGEN_FUNCPTR *)(const GLboolean *))IntGetProcAddress("glEdgeFlagv");
	if(!GL15__ptrc_glEdgeFlagv) numFailed++;
	GL15__ptrc_glEnable = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glEnable");
	if(!GL15__ptrc_glEnable) numFailed++;
	GL15__ptrc_glEnd = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glEnd");
	if(!GL15__ptrc_glEnd) numFailed++;
	GL15__ptrc_glEndList = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glEndList");
	if(!GL15__ptrc_glEndList) numFailed++;
	GL15__ptrc_glEvalCoord1d = (void (CODEGEN_FUNCPTR *)(GLdouble))IntGetProcAddress("glEvalCoord1d");
	if(!GL15__ptrc_glEvalCoord1d) numFailed++;
	GL15__ptrc_glEvalCoord1dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glEvalCoord1dv");
	if(!GL15__ptrc_glEvalCoord1dv) numFailed++;
	GL15__ptrc_glEvalCoord1f = (void (CODEGEN_FUNCPTR *)(GLfloat))IntGetProcAddress("glEvalCoord1f");
	if(!GL15__ptrc_glEvalCoord1f) numFailed++;
	GL15__ptrc_glEvalCoord1fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glEvalCoord1fv");
	if(!GL15__ptrc_glEvalCoord1fv) numFailed++;
	GL15__ptrc_glEvalCoord2d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble))IntGetProcAddress("glEvalCoord2d");
	if(!GL15__ptrc_glEvalCoord2d) numFailed++;
	GL15__ptrc_glEvalCoord2dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glEvalCoord2dv");
	if(!GL15__ptrc_glEvalCoord2dv) numFailed++;
	GL15__ptrc_glEvalCoord2f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat))IntGetProcAddress("glEvalCoord2f");
	if(!GL15__ptrc_glEvalCoord2f) numFailed++;
	GL15__ptrc_glEvalCoord2fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glEvalCoord2fv");
	if(!GL15__ptrc_glEvalCoord2fv) numFailed++;
	GL15__ptrc_glEvalMesh1 = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint))IntGetProcAddress("glEvalMesh1");
	if(!GL15__ptrc_glEvalMesh1) numFailed++;
	GL15__ptrc_glEvalMesh2 = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint, GLint))IntGetProcAddress("glEvalMesh2");
	if(!GL15__ptrc_glEvalMesh2) numFailed++;
	GL15__ptrc_glEvalPoint1 = (void (CODEGEN_FUNCPTR *)(GLint))IntGetProcAddress("glEvalPoint1");
	if(!GL15__ptrc_glEvalPoint1) numFailed++;
	GL15__ptrc_glEvalPoint2 = (void (CODEGEN_FUNCPTR *)(GLint, GLint))IntGetProcAddress("glEvalPoint2");
	if(!GL15__ptrc_glEvalPoint2) numFailed++;
	GL15__ptrc_glFeedbackBuffer = (void (CODEGEN_FUNCPTR *)(GLsizei, GLenum, GLfloat *))IntGetProcAddress("glFeedbackBuffer");
	if(!GL15__ptrc_glFeedbackBuffer) numFailed++;
	GL15__ptrc_glFinish = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glFinish");
	if(!GL15__ptrc_glFinish) numFailed++;
	GL15__ptrc_glFlush = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glFlush");
	if(!GL15__ptrc_glFlush) numFailed++;
	GL15__ptrc_glFogf = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat))IntGetProcAddress("glFogf");
	if(!GL15__ptrc_glFogf) numFailed++;
	GL15__ptrc_glFogfv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLfloat *))IntGetProcAddress("glFogfv");
	if(!GL15__ptrc_glFogfv) numFailed++;
	GL15__ptrc_glFogi = (void (CODEGEN_FUNCPTR *)(GLenum, GLint))IntGetProcAddress("glFogi");
	if(!GL15__ptrc_glFogi) numFailed++;
	GL15__ptrc_glFogiv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLint *))IntGetProcAddress("glFogiv");
	if(!GL15__ptrc_glFogiv) numFailed++;
	GL15__ptrc_glFrontFace = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glFrontFace");
	if(!GL15__ptrc_glFrontFace) numFailed++;
	GL15__ptrc_glFrustum = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glFrustum");
	if(!GL15__ptrc_glFrustum) numFailed++;
	GL15__ptrc_glGenLists = (GLuint (CODEGEN_FUNCPTR *)(GLsizei))IntGetProcAddress("glGenLists");
	if(!GL15__ptrc_glGenLists) numFailed++;
	GL15__ptrc_glGetBooleanv = (void (CODEGEN_FUNCPTR *)(GLenum, GLboolean *))IntGetProcAddress("glGetBooleanv");
	if(!GL15__ptrc_glGetBooleanv) numFailed++;
	GL15__ptrc_glGetClipPlane = (void (CODEGEN_FUNCPTR *)(GLenum, GLdouble *))IntGetProcAddress("glGetClipPlane");
	if(!GL15__ptrc_glGetClipPlane) numFailed++;
	GL15__ptrc_glGetDoublev = (void (CODEGEN_FUNCPTR *)(GLenum, GLdouble *))IntGetProcAddress("glGetDoublev");
	if(!GL15__ptrc_glGetDoublev) numFailed++;
	GL15__ptrc_glGetError = (GLenum (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glGetError");
	if(!GL15__ptrc_glGetError) numFailed++;
	GL15__ptrc_glGetFloatv = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat *))IntGetProcAddress("glGetFloatv");
	if(!GL15__ptrc_glGetFloatv) numFailed++;
	GL15__ptrc_glGetIntegerv = (void (CODEGEN_FUNCPTR *)(GLenum, GLint *))IntGetProcAddress("glGetIntegerv");
	if(!GL15__ptrc_glGetIntegerv) numFailed++;
	GL15__ptrc_glGetLightfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat *))IntGetProcAddress("glGetLightfv");
	if(!GL15__ptrc_glGetLightfv) numFailed++;
	GL15__ptrc_glGetLightiv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint *))IntGetProcAddress("glGetLightiv");
	if(!GL15__ptrc_glGetLightiv) numFailed++;
	GL15__ptrc_glGetMapdv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLdouble *))IntGetProcAddress("glGetMapdv");
	if(!GL15__ptrc_glGetMapdv) numFailed++;
	GL15__ptrc_glGetMapfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat *))IntGetProcAddress("glGetMapfv");
	if(!GL15__ptrc_glGetMapfv) numFailed++;
	GL15__ptrc_glGetMapiv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint *))IntGetProcAddress("glGetMapiv");
	if(!GL15__ptrc_glGetMapiv) numFailed++;
	GL15__ptrc_glGetMaterialfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat *))IntGetProcAddress("glGetMaterialfv");
	if(!GL15__ptrc_glGetMaterialfv) numFailed++;
	GL15__ptrc_glGetMaterialiv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint *))IntGetProcAddress("glGetMaterialiv");
	if(!GL15__ptrc_glGetMaterialiv) numFailed++;
	GL15__ptrc_glGetPixelMapfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat *))IntGetProcAddress("glGetPixelMapfv");
	if(!GL15__ptrc_glGetPixelMapfv) numFailed++;
	GL15__ptrc_glGetPixelMapuiv = (void (CODEGEN_FUNCPTR *)(GLenum, GLuint *))IntGetProcAddress("glGetPixelMapuiv");
	if(!GL15__ptrc_glGetPixelMapuiv) numFailed++;
	GL15__ptrc_glGetPixelMapusv = (void (CODEGEN_FUNCPTR *)(GLenum, GLushort *))IntGetProcAddress("glGetPixelMapusv");
	if(!GL15__ptrc_glGetPixelMapusv) numFailed++;
	GL15__ptrc_glGetPolygonStipple = (void (CODEGEN_FUNCPTR *)(GLubyte *))IntGetProcAddress("glGetPolygonStipple");
	if(!GL15__ptrc_glGetPolygonStipple) numFailed++;
	GL15__ptrc_glGetString = (const GLubyte * (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glGetString");
	if(!GL15__ptrc_glGetString) numFailed++;
	GL15__ptrc_glGetTexEnvfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat *))IntGetProcAddress("glGetTexEnvfv");
	if(!GL15__ptrc_glGetTexEnvfv) numFailed++;
	GL15__ptrc_glGetTexEnviv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint *))IntGetProcAddress("glGetTexEnviv");
	if(!GL15__ptrc_glGetTexEnviv) numFailed++;
	GL15__ptrc_glGetTexGendv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLdouble *))IntGetProcAddress("glGetTexGendv");
	if(!GL15__ptrc_glGetTexGendv) numFailed++;
	GL15__ptrc_glGetTexGenfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat *))IntGetProcAddress("glGetTexGenfv");
	if(!GL15__ptrc_glGetTexGenfv) numFailed++;
	GL15__ptrc_glGetTexGeniv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint *))IntGetProcAddress("glGetTexGeniv");
	if(!GL15__ptrc_glGetTexGeniv) numFailed++;
	GL15__ptrc_glGetTexImage = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLenum, GLenum, void *))IntGetProcAddress("glGetTexImage");
	if(!GL15__ptrc_glGetTexImage) numFailed++;
	GL15__ptrc_glGetTexLevelParameterfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLenum, GLfloat *))IntGetProcAddress("glGetTexLevelParameterfv");
	if(!GL15__ptrc_glGetTexLevelParameterfv) numFailed++;
	GL15__ptrc_glGetTexLevelParameteriv = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLenum, GLint *))IntGetProcAddress("glGetTexLevelParameteriv");
	if(!GL15__ptrc_glGetTexLevelParameteriv) numFailed++;
	GL15__ptrc_glGetTexParameterfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat *))IntGetProcAddress("glGetTexParameterfv");
	if(!GL15__ptrc_glGetTexParameterfv) numFailed++;
	GL15__ptrc_glGetTexParameteriv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint *))IntGetProcAddress("glGetTexParameteriv");
	if(!GL15__ptrc_glGetTexParameteriv) numFailed++;
	GL15__ptrc_glHint = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum))IntGetProcAddress("glHint");
	if(!GL15__ptrc_glHint) numFailed++;
	GL15__ptrc_glIndexMask = (void (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glIndexMask");
	if(!GL15__ptrc_glIndexMask) numFailed++;
	GL15__ptrc_glIndexd = (void (CODEGEN_FUNCPTR *)(GLdouble))IntGetProcAddress("glIndexd");
	if(!GL15__ptrc_glIndexd) numFailed++;
	GL15__ptrc_glIndexdv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glIndexdv");
	if(!GL15__ptrc_glIndexdv) numFailed++;
	GL15__ptrc_glIndexf = (void (CODEGEN_FUNCPTR *)(GLfloat))IntGetProcAddress("glIndexf");
	if(!GL15__ptrc_glIndexf) numFailed++;
	GL15__ptrc_glIndexfv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glIndexfv");
	if(!GL15__ptrc_glIndexfv) numFailed++;
	GL15__ptrc_glIndexi = (void (CODEGEN_FUNCPTR *)(GLint))IntGetProcAddress("glIndexi");
	if(!GL15__ptrc_glIndexi) numFailed++;
	GL15__ptrc_glIndexiv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glIndexiv");
	if(!GL15__ptrc_glIndexiv) numFailed++;
	GL15__ptrc_glIndexs = (void (CODEGEN_FUNCPTR *)(GLshort))IntGetProcAddress("glIndexs");
	if(!GL15__ptrc_glIndexs) numFailed++;
	GL15__ptrc_glIndexsv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glIndexsv");
	if(!GL15__ptrc_glIndexsv) numFailed++;
	GL15__ptrc_glInitNames = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glInitNames");
	if(!GL15__ptrc_glInitNames) numFailed++;
	GL15__ptrc_glIsEnabled = (GLboolean (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glIsEnabled");
	if(!GL15__ptrc_glIsEnabled) numFailed++;
	GL15__ptrc_glIsList = (GLboolean (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glIsList");
	if(!GL15__ptrc_glIsList) numFailed++;
	GL15__ptrc_glLightModelf = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat))IntGetProcAddress("glLightModelf");
	if(!GL15__ptrc_glLightModelf) numFailed++;
	GL15__ptrc_glLightModelfv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLfloat *))IntGetProcAddress("glLightModelfv");
	if(!GL15__ptrc_glLightModelfv) numFailed++;
	GL15__ptrc_glLightModeli = (void (CODEGEN_FUNCPTR *)(GLenum, GLint))IntGetProcAddress("glLightModeli");
	if(!GL15__ptrc_glLightModeli) numFailed++;
	GL15__ptrc_glLightModeliv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLint *))IntGetProcAddress("glLightModeliv");
	if(!GL15__ptrc_glLightModeliv) numFailed++;
	GL15__ptrc_glLightf = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat))IntGetProcAddress("glLightf");
	if(!GL15__ptrc_glLightf) numFailed++;
	GL15__ptrc_glLightfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLfloat *))IntGetProcAddress("glLightfv");
	if(!GL15__ptrc_glLightfv) numFailed++;
	GL15__ptrc_glLighti = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint))IntGetProcAddress("glLighti");
	if(!GL15__ptrc_glLighti) numFailed++;
	GL15__ptrc_glLightiv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLint *))IntGetProcAddress("glLightiv");
	if(!GL15__ptrc_glLightiv) numFailed++;
	GL15__ptrc_glLineStipple = (void (CODEGEN_FUNCPTR *)(GLint, GLushort))IntGetProcAddress("glLineStipple");
	if(!GL15__ptrc_glLineStipple) numFailed++;
	GL15__ptrc_glLineWidth = (void (CODEGEN_FUNCPTR *)(GLfloat))IntGetProcAddress("glLineWidth");
	if(!GL15__ptrc_glLineWidth) numFailed++;
	GL15__ptrc_glListBase = (void (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glListBase");
	if(!GL15__ptrc_glListBase) numFailed++;
	GL15__ptrc_glLoadIdentity = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glLoadIdentity");
	if(!GL15__ptrc_glLoadIdentity) numFailed++;
	GL15__ptrc_glLoadMatrixd = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glLoadMatrixd");
	if(!GL15__ptrc_glLoadMatrixd) numFailed++;
	GL15__ptrc_glLoadMatrixf = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glLoadMatrixf");
	if(!GL15__ptrc_glLoadMatrixf) numFailed++;
	GL15__ptrc_glLoadName = (void (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glLoadName");
	if(!GL15__ptrc_glLoadName) numFailed++;
	GL15__ptrc_glLogicOp = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glLogicOp");
	if(!GL15__ptrc_glLogicOp) numFailed++;
	GL15__ptrc_glMap1d = (void (CODEGEN_FUNCPTR *)(GLenum, GLdouble, GLdouble, GLint, GLint, const GLdouble *))IntGetProcAddress("glMap1d");
	if(!GL15__ptrc_glMap1d) numFailed++;
	GL15__ptrc_glMap1f = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat, GLfloat, GLint, GLint, const GLfloat *))IntGetProcAddress("glMap1f");
	if(!GL15__ptrc_glMap1f) numFailed++;
	GL15__ptrc_glMap2d = (void (CODEGEN_FUNCPTR *)(GLenum, GLdouble, GLdouble, GLint, GLint, GLdouble, GLdouble, GLint, GLint, const GLdouble *))IntGetProcAddress("glMap2d");
	if(!GL15__ptrc_glMap2d) numFailed++;
	GL15__ptrc_glMap2f = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat, GLfloat, GLint, GLint, GLfloat, GLfloat, GLint, GLint, const GLfloat *))IntGetProcAddress("glMap2f");
	if(!GL15__ptrc_glMap2f) numFailed++;
	GL15__ptrc_glMapGrid1d = (void (CODEGEN_FUNCPTR *)(GLint, GLdouble, GLdouble))IntGetProcAddress("glMapGrid1d");
	if(!GL15__ptrc_glMapGrid1d) numFailed++;
	GL15__ptrc_glMapGrid1f = (void (CODEGEN_FUNCPTR *)(GLint, GLfloat, GLfloat))IntGetProcAddress("glMapGrid1f");
	if(!GL15__ptrc_glMapGrid1f) numFailed++;
	GL15__ptrc_glMapGrid2d = (void (CODEGEN_FUNCPTR *)(GLint, GLdouble, GLdouble, GLint, GLdouble, GLdouble))IntGetProcAddress("glMapGrid2d");
	if(!GL15__ptrc_glMapGrid2d) numFailed++;
	GL15__ptrc_glMapGrid2f = (void (CODEGEN_FUNCPTR *)(GLint, GLfloat, GLfloat, GLint, GLfloat, GLfloat))IntGetProcAddress("glMapGrid2f");
	if(!GL15__ptrc_glMapGrid2f) numFailed++;
	GL15__ptrc_glMaterialf = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat))IntGetProcAddress("glMaterialf");
	if(!GL15__ptrc_glMaterialf) numFailed++;
	GL15__ptrc_glMaterialfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLfloat *))IntGetProcAddress("glMaterialfv");
	if(!GL15__ptrc_glMaterialfv) numFailed++;
	GL15__ptrc_glMateriali = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint))IntGetProcAddress("glMateriali");
	if(!GL15__ptrc_glMateriali) numFailed++;
	GL15__ptrc_glMaterialiv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLint *))IntGetProcAddress("glMaterialiv");
	if(!GL15__ptrc_glMaterialiv) numFailed++;
	GL15__ptrc_glMatrixMode = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glMatrixMode");
	if(!GL15__ptrc_glMatrixMode) numFailed++;
	GL15__ptrc_glMultMatrixd = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glMultMatrixd");
	if(!GL15__ptrc_glMultMatrixd) numFailed++;
	GL15__ptrc_glMultMatrixf = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glMultMatrixf");
	if(!GL15__ptrc_glMultMatrixf) numFailed++;
	GL15__ptrc_glNewList = (void (CODEGEN_FUNCPTR *)(GLuint, GLenum))IntGetProcAddress("glNewList");
	if(!GL15__ptrc_glNewList) numFailed++;
	GL15__ptrc_glNormal3b = (void (CODEGEN_FUNCPTR *)(GLbyte, GLbyte, GLbyte))IntGetProcAddress("glNormal3b");
	if(!GL15__ptrc_glNormal3b) numFailed++;
	GL15__ptrc_glNormal3bv = (void (CODEGEN_FUNCPTR *)(const GLbyte *))IntGetProcAddress("glNormal3bv");
	if(!GL15__ptrc_glNormal3bv) numFailed++;
	GL15__ptrc_glNormal3d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble))IntGetProcAddress("glNormal3d");
	if(!GL15__ptrc_glNormal3d) numFailed++;
	GL15__ptrc_glNormal3dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glNormal3dv");
	if(!GL15__ptrc_glNormal3dv) numFailed++;
	GL15__ptrc_glNormal3f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat))IntGetProcAddress("glNormal3f");
	if(!GL15__ptrc_glNormal3f) numFailed++;
	GL15__ptrc_glNormal3fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glNormal3fv");
	if(!GL15__ptrc_glNormal3fv) numFailed++;
	GL15__ptrc_glNormal3i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint))IntGetProcAddress("glNormal3i");
	if(!GL15__ptrc_glNormal3i) numFailed++;
	GL15__ptrc_glNormal3iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glNormal3iv");
	if(!GL15__ptrc_glNormal3iv) numFailed++;
	GL15__ptrc_glNormal3s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort))IntGetProcAddress("glNormal3s");
	if(!GL15__ptrc_glNormal3s) numFailed++;
	GL15__ptrc_glNormal3sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glNormal3sv");
	if(!GL15__ptrc_glNormal3sv) numFailed++;
	GL15__ptrc_glOrtho = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glOrtho");
	if(!GL15__ptrc_glOrtho) numFailed++;
	GL15__ptrc_glPassThrough = (void (CODEGEN_FUNCPTR *)(GLfloat))IntGetProcAddress("glPassThrough");
	if(!GL15__ptrc_glPassThrough) numFailed++;
	GL15__ptrc_glPixelMapfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLsizei, const GLfloat *))IntGetProcAddress("glPixelMapfv");
	if(!GL15__ptrc_glPixelMapfv) numFailed++;
	GL15__ptrc_glPixelMapuiv = (void (CODEGEN_FUNCPTR *)(GLenum, GLsizei, const GLuint *))IntGetProcAddress("glPixelMapuiv");
	if(!GL15__ptrc_glPixelMapuiv) numFailed++;
	GL15__ptrc_glPixelMapusv = (void (CODEGEN_FUNCPTR *)(GLenum, GLsizei, const GLushort *))IntGetProcAddress("glPixelMapusv");
	if(!GL15__ptrc_glPixelMapusv) numFailed++;
	GL15__ptrc_glPixelStoref = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat))IntGetProcAddress("glPixelStoref");
	if(!GL15__ptrc_glPixelStoref) numFailed++;
	GL15__ptrc_glPixelStorei = (void (CODEGEN_FUNCPTR *)(GLenum, GLint))IntGetProcAddress("glPixelStorei");
	if(!GL15__ptrc_glPixelStorei) numFailed++;
	GL15__ptrc_glPixelTransferf = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat))IntGetProcAddress("glPixelTransferf");
	if(!GL15__ptrc_glPixelTransferf) numFailed++;
	GL15__ptrc_glPixelTransferi = (void (CODEGEN_FUNCPTR *)(GLenum, GLint))IntGetProcAddress("glPixelTransferi");
	if(!GL15__ptrc_glPixelTransferi) numFailed++;
	GL15__ptrc_glPixelZoom = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat))IntGetProcAddress("glPixelZoom");
	if(!GL15__ptrc_glPixelZoom) numFailed++;
	GL15__ptrc_glPointSize = (void (CODEGEN_FUNCPTR *)(GLfloat))IntGetProcAddress("glPointSize");
	if(!GL15__ptrc_glPointSize) numFailed++;
	GL15__ptrc_glPolygonMode = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum))IntGetProcAddress("glPolygonMode");
	if(!GL15__ptrc_glPolygonMode) numFailed++;
	GL15__ptrc_glPolygonStipple = (void (CODEGEN_FUNCPTR *)(const GLubyte *))IntGetProcAddress("glPolygonStipple");
	if(!GL15__ptrc_glPolygonStipple) numFailed++;
	GL15__ptrc_glPopAttrib = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glPopAttrib");
	if(!GL15__ptrc_glPopAttrib) numFailed++;
	GL15__ptrc_glPopMatrix = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glPopMatrix");
	if(!GL15__ptrc_glPopMatrix) numFailed++;
	GL15__ptrc_glPopName = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glPopName");
	if(!GL15__ptrc_glPopName) numFailed++;
	GL15__ptrc_glPushAttrib = (void (CODEGEN_FUNCPTR *)(GLbitfield))IntGetProcAddress("glPushAttrib");
	if(!GL15__ptrc_glPushAttrib) numFailed++;
	GL15__ptrc_glPushMatrix = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glPushMatrix");
	if(!GL15__ptrc_glPushMatrix) numFailed++;
	GL15__ptrc_glPushName = (void (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glPushName");
	if(!GL15__ptrc_glPushName) numFailed++;
	GL15__ptrc_glRasterPos2d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble))IntGetProcAddress("glRasterPos2d");
	if(!GL15__ptrc_glRasterPos2d) numFailed++;
	GL15__ptrc_glRasterPos2dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glRasterPos2dv");
	if(!GL15__ptrc_glRasterPos2dv) numFailed++;
	GL15__ptrc_glRasterPos2f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat))IntGetProcAddress("glRasterPos2f");
	if(!GL15__ptrc_glRasterPos2f) numFailed++;
	GL15__ptrc_glRasterPos2fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glRasterPos2fv");
	if(!GL15__ptrc_glRasterPos2fv) numFailed++;
	GL15__ptrc_glRasterPos2i = (void (CODEGEN_FUNCPTR *)(GLint, GLint))IntGetProcAddress("glRasterPos2i");
	if(!GL15__ptrc_glRasterPos2i) numFailed++;
	GL15__ptrc_glRasterPos2iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glRasterPos2iv");
	if(!GL15__ptrc_glRasterPos2iv) numFailed++;
	GL15__ptrc_glRasterPos2s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort))IntGetProcAddress("glRasterPos2s");
	if(!GL15__ptrc_glRasterPos2s) numFailed++;
	GL15__ptrc_glRasterPos2sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glRasterPos2sv");
	if(!GL15__ptrc_glRasterPos2sv) numFailed++;
	GL15__ptrc_glRasterPos3d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble))IntGetProcAddress("glRasterPos3d");
	if(!GL15__ptrc_glRasterPos3d) numFailed++;
	GL15__ptrc_glRasterPos3dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glRasterPos3dv");
	if(!GL15__ptrc_glRasterPos3dv) numFailed++;
	GL15__ptrc_glRasterPos3f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat))IntGetProcAddress("glRasterPos3f");
	if(!GL15__ptrc_glRasterPos3f) numFailed++;
	GL15__ptrc_glRasterPos3fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glRasterPos3fv");
	if(!GL15__ptrc_glRasterPos3fv) numFailed++;
	GL15__ptrc_glRasterPos3i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint))IntGetProcAddress("glRasterPos3i");
	if(!GL15__ptrc_glRasterPos3i) numFailed++;
	GL15__ptrc_glRasterPos3iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glRasterPos3iv");
	if(!GL15__ptrc_glRasterPos3iv) numFailed++;
	GL15__ptrc_glRasterPos3s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort))IntGetProcAddress("glRasterPos3s");
	if(!GL15__ptrc_glRasterPos3s) numFailed++;
	GL15__ptrc_glRasterPos3sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glRasterPos3sv");
	if(!GL15__ptrc_glRasterPos3sv) numFailed++;
	GL15__ptrc_glRasterPos4d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glRasterPos4d");
	if(!GL15__ptrc_glRasterPos4d) numFailed++;
	GL15__ptrc_glRasterPos4dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glRasterPos4dv");
	if(!GL15__ptrc_glRasterPos4dv) numFailed++;
	GL15__ptrc_glRasterPos4f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glRasterPos4f");
	if(!GL15__ptrc_glRasterPos4f) numFailed++;
	GL15__ptrc_glRasterPos4fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glRasterPos4fv");
	if(!GL15__ptrc_glRasterPos4fv) numFailed++;
	GL15__ptrc_glRasterPos4i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint, GLint))IntGetProcAddress("glRasterPos4i");
	if(!GL15__ptrc_glRasterPos4i) numFailed++;
	GL15__ptrc_glRasterPos4iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glRasterPos4iv");
	if(!GL15__ptrc_glRasterPos4iv) numFailed++;
	GL15__ptrc_glRasterPos4s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort, GLshort))IntGetProcAddress("glRasterPos4s");
	if(!GL15__ptrc_glRasterPos4s) numFailed++;
	GL15__ptrc_glRasterPos4sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glRasterPos4sv");
	if(!GL15__ptrc_glRasterPos4sv) numFailed++;
	GL15__ptrc_glReadBuffer = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glReadBuffer");
	if(!GL15__ptrc_glReadBuffer) numFailed++;
	GL15__ptrc_glReadPixels = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, void *))IntGetProcAddress("glReadPixels");
	if(!GL15__ptrc_glReadPixels) numFailed++;
	GL15__ptrc_glRectd = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glRectd");
	if(!GL15__ptrc_glRectd) numFailed++;
	GL15__ptrc_glRectdv = (void (CODEGEN_FUNCPTR *)(const GLdouble *, const GLdouble *))IntGetProcAddress("glRectdv");
	if(!GL15__ptrc_glRectdv) numFailed++;
	GL15__ptrc_glRectf = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glRectf");
	if(!GL15__ptrc_glRectf) numFailed++;
	GL15__ptrc_glRectfv = (void (CODEGEN_FUNCPTR *)(const GLfloat *, const GLfloat *))IntGetProcAddress("glRectfv");
	if(!GL15__ptrc_glRectfv) numFailed++;
	GL15__ptrc_glRecti = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint, GLint))IntGetProcAddress("glRecti");
	if(!GL15__ptrc_glRecti) numFailed++;
	GL15__ptrc_glRectiv = (void (CODEGEN_FUNCPTR *)(const GLint *, const GLint *))IntGetProcAddress("glRectiv");
	if(!GL15__ptrc_glRectiv) numFailed++;
	GL15__ptrc_glRects = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort, GLshort))IntGetProcAddress("glRects");
	if(!GL15__ptrc_glRects) numFailed++;
	GL15__ptrc_glRectsv = (void (CODEGEN_FUNCPTR *)(const GLshort *, const GLshort *))IntGetProcAddress("glRectsv");
	if(!GL15__ptrc_glRectsv) numFailed++;
	GL15__ptrc_glRenderMode = (GLint (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glRenderMode");
	if(!GL15__ptrc_glRenderMode) numFailed++;
	GL15__ptrc_glRotated = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glRotated");
	if(!GL15__ptrc_glRotated) numFailed++;
	GL15__ptrc_glRotatef = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glRotatef");
	if(!GL15__ptrc_glRotatef) numFailed++;
	GL15__ptrc_glScaled = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble))IntGetProcAddress("glScaled");
	if(!GL15__ptrc_glScaled) numFailed++;
	GL15__ptrc_glScalef = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat))IntGetProcAddress("glScalef");
	if(!GL15__ptrc_glScalef) numFailed++;
	GL15__ptrc_glScissor = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLsizei, GLsizei))IntGetProcAddress("glScissor");
	if(!GL15__ptrc_glScissor) numFailed++;
	GL15__ptrc_glSelectBuffer = (void (CODEGEN_FUNCPTR *)(GLsizei, GLuint *))IntGetProcAddress("glSelectBuffer");
	if(!GL15__ptrc_glSelectBuffer) numFailed++;
	GL15__ptrc_glShadeModel = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glShadeModel");
	if(!GL15__ptrc_glShadeModel) numFailed++;
	GL15__ptrc_glStencilFunc = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLuint))IntGetProcAddress("glStencilFunc");
	if(!GL15__ptrc_glStencilFunc) numFailed++;
	GL15__ptrc_glStencilMask = (void (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glStencilMask");
	if(!GL15__ptrc_glStencilMask) numFailed++;
	GL15__ptrc_glStencilOp = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLenum))IntGetProcAddress("glStencilOp");
	if(!GL15__ptrc_glStencilOp) numFailed++;
	GL15__ptrc_glTexCoord1d = (void (CODEGEN_FUNCPTR *)(GLdouble))IntGetProcAddress("glTexCoord1d");
	if(!GL15__ptrc_glTexCoord1d) numFailed++;
	GL15__ptrc_glTexCoord1dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glTexCoord1dv");
	if(!GL15__ptrc_glTexCoord1dv) numFailed++;
	GL15__ptrc_glTexCoord1f = (void (CODEGEN_FUNCPTR *)(GLfloat))IntGetProcAddress("glTexCoord1f");
	if(!GL15__ptrc_glTexCoord1f) numFailed++;
	GL15__ptrc_glTexCoord1fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glTexCoord1fv");
	if(!GL15__ptrc_glTexCoord1fv) numFailed++;
	GL15__ptrc_glTexCoord1i = (void (CODEGEN_FUNCPTR *)(GLint))IntGetProcAddress("glTexCoord1i");
	if(!GL15__ptrc_glTexCoord1i) numFailed++;
	GL15__ptrc_glTexCoord1iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glTexCoord1iv");
	if(!GL15__ptrc_glTexCoord1iv) numFailed++;
	GL15__ptrc_glTexCoord1s = (void (CODEGEN_FUNCPTR *)(GLshort))IntGetProcAddress("glTexCoord1s");
	if(!GL15__ptrc_glTexCoord1s) numFailed++;
	GL15__ptrc_glTexCoord1sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glTexCoord1sv");
	if(!GL15__ptrc_glTexCoord1sv) numFailed++;
	GL15__ptrc_glTexCoord2d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble))IntGetProcAddress("glTexCoord2d");
	if(!GL15__ptrc_glTexCoord2d) numFailed++;
	GL15__ptrc_glTexCoord2dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glTexCoord2dv");
	if(!GL15__ptrc_glTexCoord2dv) numFailed++;
	GL15__ptrc_glTexCoord2f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat))IntGetProcAddress("glTexCoord2f");
	if(!GL15__ptrc_glTexCoord2f) numFailed++;
	GL15__ptrc_glTexCoord2fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glTexCoord2fv");
	if(!GL15__ptrc_glTexCoord2fv) numFailed++;
	GL15__ptrc_glTexCoord2i = (void (CODEGEN_FUNCPTR *)(GLint, GLint))IntGetProcAddress("glTexCoord2i");
	if(!GL15__ptrc_glTexCoord2i) numFailed++;
	GL15__ptrc_glTexCoord2iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glTexCoord2iv");
	if(!GL15__ptrc_glTexCoord2iv) numFailed++;
	GL15__ptrc_glTexCoord2s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort))IntGetProcAddress("glTexCoord2s");
	if(!GL15__ptrc_glTexCoord2s) numFailed++;
	GL15__ptrc_glTexCoord2sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glTexCoord2sv");
	if(!GL15__ptrc_glTexCoord2sv) numFailed++;
	GL15__ptrc_glTexCoord3d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble))IntGetProcAddress("glTexCoord3d");
	if(!GL15__ptrc_glTexCoord3d) numFailed++;
	GL15__ptrc_glTexCoord3dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glTexCoord3dv");
	if(!GL15__ptrc_glTexCoord3dv) numFailed++;
	GL15__ptrc_glTexCoord3f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat))IntGetProcAddress("glTexCoord3f");
	if(!GL15__ptrc_glTexCoord3f) numFailed++;
	GL15__ptrc_glTexCoord3fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glTexCoord3fv");
	if(!GL15__ptrc_glTexCoord3fv) numFailed++;
	GL15__ptrc_glTexCoord3i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint))IntGetProcAddress("glTexCoord3i");
	if(!GL15__ptrc_glTexCoord3i) numFailed++;
	GL15__ptrc_glTexCoord3iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glTexCoord3iv");
	if(!GL15__ptrc_glTexCoord3iv) numFailed++;
	GL15__ptrc_glTexCoord3s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort))IntGetProcAddress("glTexCoord3s");
	if(!GL15__ptrc_glTexCoord3s) numFailed++;
	GL15__ptrc_glTexCoord3sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glTexCoord3sv");
	if(!GL15__ptrc_glTexCoord3sv) numFailed++;
	GL15__ptrc_glTexCoord4d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glTexCoord4d");
	if(!GL15__ptrc_glTexCoord4d) numFailed++;
	GL15__ptrc_glTexCoord4dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glTexCoord4dv");
	if(!GL15__ptrc_glTexCoord4dv) numFailed++;
	GL15__ptrc_glTexCoord4f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glTexCoord4f");
	if(!GL15__ptrc_glTexCoord4f) numFailed++;
	GL15__ptrc_glTexCoord4fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glTexCoord4fv");
	if(!GL15__ptrc_glTexCoord4fv) numFailed++;
	GL15__ptrc_glTexCoord4i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint, GLint))IntGetProcAddress("glTexCoord4i");
	if(!GL15__ptrc_glTexCoord4i) numFailed++;
	GL15__ptrc_glTexCoord4iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glTexCoord4iv");
	if(!GL15__ptrc_glTexCoord4iv) numFailed++;
	GL15__ptrc_glTexCoord4s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort, GLshort))IntGetProcAddress("glTexCoord4s");
	if(!GL15__ptrc_glTexCoord4s) numFailed++;
	GL15__ptrc_glTexCoord4sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glTexCoord4sv");
	if(!GL15__ptrc_glTexCoord4sv) numFailed++;
	GL15__ptrc_glTexEnvf = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat))IntGetProcAddress("glTexEnvf");
	if(!GL15__ptrc_glTexEnvf) numFailed++;
	GL15__ptrc_glTexEnvfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLfloat *))IntGetProcAddress("glTexEnvfv");
	if(!GL15__ptrc_glTexEnvfv) numFailed++;
	GL15__ptrc_glTexEnvi = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint))IntGetProcAddress("glTexEnvi");
	if(!GL15__ptrc_glTexEnvi) numFailed++;
	GL15__ptrc_glTexEnviv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLint *))IntGetProcAddress("glTexEnviv");
	if(!GL15__ptrc_glTexEnviv) numFailed++;
	GL15__ptrc_glTexGend = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLdouble))IntGetProcAddress("glTexGend");
	if(!GL15__ptrc_glTexGend) numFailed++;
	GL15__ptrc_glTexGendv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLdouble *))IntGetProcAddress("glTexGendv");
	if(!GL15__ptrc_glTexGendv) numFailed++;
	GL15__ptrc_glTexGenf = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat))IntGetProcAddress("glTexGenf");
	if(!GL15__ptrc_glTexGenf) numFailed++;
	GL15__ptrc_glTexGenfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLfloat *))IntGetProcAddress("glTexGenfv");
	if(!GL15__ptrc_glTexGenfv) numFailed++;
	GL15__ptrc_glTexGeni = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint))IntGetProcAddress("glTexGeni");
	if(!GL15__ptrc_glTexGeni) numFailed++;
	GL15__ptrc_glTexGeniv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLint *))IntGetProcAddress("glTexGeniv");
	if(!GL15__ptrc_glTexGeniv) numFailed++;
	GL15__ptrc_glTexImage1D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLsizei, GLint, GLenum, GLenum, const void *))IntGetProcAddress("glTexImage1D");
	if(!GL15__ptrc_glTexImage1D) numFailed++;
	GL15__ptrc_glTexImage2D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void *))IntGetProcAddress("glTexImage2D");
	if(!GL15__ptrc_glTexImage2D) numFailed++;
	GL15__ptrc_glTexParameterf = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLfloat))IntGetProcAddress("glTexParameterf");
	if(!GL15__ptrc_glTexParameterf) numFailed++;
	GL15__ptrc_glTexParameterfv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLfloat *))IntGetProcAddress("glTexParameterfv");
	if(!GL15__ptrc_glTexParameterfv) numFailed++;
	GL15__ptrc_glTexParameteri = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint))IntGetProcAddress("glTexParameteri");
	if(!GL15__ptrc_glTexParameteri) numFailed++;
	GL15__ptrc_glTexParameteriv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, const GLint *))IntGetProcAddress("glTexParameteriv");
	if(!GL15__ptrc_glTexParameteriv) numFailed++;
	GL15__ptrc_glTranslated = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble))IntGetProcAddress("glTranslated");
	if(!GL15__ptrc_glTranslated) numFailed++;
	GL15__ptrc_glTranslatef = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat))IntGetProcAddress("glTranslatef");
	if(!GL15__ptrc_glTranslatef) numFailed++;
	GL15__ptrc_glVertex2d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble))IntGetProcAddress("glVertex2d");
	if(!GL15__ptrc_glVertex2d) numFailed++;
	GL15__ptrc_glVertex2dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glVertex2dv");
	if(!GL15__ptrc_glVertex2dv) numFailed++;
	GL15__ptrc_glVertex2f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat))IntGetProcAddress("glVertex2f");
	if(!GL15__ptrc_glVertex2f) numFailed++;
	GL15__ptrc_glVertex2fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glVertex2fv");
	if(!GL15__ptrc_glVertex2fv) numFailed++;
	GL15__ptrc_glVertex2i = (void (CODEGEN_FUNCPTR *)(GLint, GLint))IntGetProcAddress("glVertex2i");
	if(!GL15__ptrc_glVertex2i) numFailed++;
	GL15__ptrc_glVertex2iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glVertex2iv");
	if(!GL15__ptrc_glVertex2iv) numFailed++;
	GL15__ptrc_glVertex2s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort))IntGetProcAddress("glVertex2s");
	if(!GL15__ptrc_glVertex2s) numFailed++;
	GL15__ptrc_glVertex2sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glVertex2sv");
	if(!GL15__ptrc_glVertex2sv) numFailed++;
	GL15__ptrc_glVertex3d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble))IntGetProcAddress("glVertex3d");
	if(!GL15__ptrc_glVertex3d) numFailed++;
	GL15__ptrc_glVertex3dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glVertex3dv");
	if(!GL15__ptrc_glVertex3dv) numFailed++;
	GL15__ptrc_glVertex3f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat))IntGetProcAddress("glVertex3f");
	if(!GL15__ptrc_glVertex3f) numFailed++;
	GL15__ptrc_glVertex3fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glVertex3fv");
	if(!GL15__ptrc_glVertex3fv) numFailed++;
	GL15__ptrc_glVertex3i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint))IntGetProcAddress("glVertex3i");
	if(!GL15__ptrc_glVertex3i) numFailed++;
	GL15__ptrc_glVertex3iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glVertex3iv");
	if(!GL15__ptrc_glVertex3iv) numFailed++;
	GL15__ptrc_glVertex3s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort))IntGetProcAddress("glVertex3s");
	if(!GL15__ptrc_glVertex3s) numFailed++;
	GL15__ptrc_glVertex3sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glVertex3sv");
	if(!GL15__ptrc_glVertex3sv) numFailed++;
	GL15__ptrc_glVertex4d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glVertex4d");
	if(!GL15__ptrc_glVertex4d) numFailed++;
	GL15__ptrc_glVertex4dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glVertex4dv");
	if(!GL15__ptrc_glVertex4dv) numFailed++;
	GL15__ptrc_glVertex4f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glVertex4f");
	if(!GL15__ptrc_glVertex4f) numFailed++;
	GL15__ptrc_glVertex4fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glVertex4fv");
	if(!GL15__ptrc_glVertex4fv) numFailed++;
	GL15__ptrc_glVertex4i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint, GLint))IntGetProcAddress("glVertex4i");
	if(!GL15__ptrc_glVertex4i) numFailed++;
	GL15__ptrc_glVertex4iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glVertex4iv");
	if(!GL15__ptrc_glVertex4iv) numFailed++;
	GL15__ptrc_glVertex4s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort, GLshort))IntGetProcAddress("glVertex4s");
	if(!GL15__ptrc_glVertex4s) numFailed++;
	GL15__ptrc_glVertex4sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glVertex4sv");
	if(!GL15__ptrc_glVertex4sv) numFailed++;
	GL15__ptrc_glViewport = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLsizei, GLsizei))IntGetProcAddress("glViewport");
	if(!GL15__ptrc_glViewport) numFailed++;
	GL15__ptrc_glAreTexturesResident = (GLboolean (CODEGEN_FUNCPTR *)(GLsizei, const GLuint *, GLboolean *))IntGetProcAddress("glAreTexturesResident");
	if(!GL15__ptrc_glAreTexturesResident) numFailed++;
	GL15__ptrc_glArrayElement = (void (CODEGEN_FUNCPTR *)(GLint))IntGetProcAddress("glArrayElement");
	if(!GL15__ptrc_glArrayElement) numFailed++;
	GL15__ptrc_glBindTexture = (void (CODEGEN_FUNCPTR *)(GLenum, GLuint))IntGetProcAddress("glBindTexture");
	if(!GL15__ptrc_glBindTexture) numFailed++;
	GL15__ptrc_glColorPointer = (void (CODEGEN_FUNCPTR *)(GLint, GLenum, GLsizei, const void *))IntGetProcAddress("glColorPointer");
	if(!GL15__ptrc_glColorPointer) numFailed++;
	GL15__ptrc_glCopyTexImage1D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLint))IntGetProcAddress("glCopyTexImage1D");
	if(!GL15__ptrc_glCopyTexImage1D) numFailed++;
	GL15__ptrc_glCopyTexImage2D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLsizei, GLint))IntGetProcAddress("glCopyTexImage2D");
	if(!GL15__ptrc_glCopyTexImage2D) numFailed++;
	GL15__ptrc_glCopyTexSubImage1D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint, GLint, GLsizei))IntGetProcAddress("glCopyTexSubImage1D");
	if(!GL15__ptrc_glCopyTexSubImage1D) numFailed++;
	GL15__ptrc_glCopyTexSubImage2D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei))IntGetProcAddress("glCopyTexSubImage2D");
	if(!GL15__ptrc_glCopyTexSubImage2D) numFailed++;
	GL15__ptrc_glDeleteTextures = (void (CODEGEN_FUNCPTR *)(GLsizei, const GLuint *))IntGetProcAddress("glDeleteTextures");
	if(!GL15__ptrc_glDeleteTextures) numFailed++;
	GL15__ptrc_glDisableClientState = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glDisableClientState");
	if(!GL15__ptrc_glDisableClientState) numFailed++;
	GL15__ptrc_glDrawArrays = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLsizei))IntGetProcAddress("glDrawArrays");
	if(!GL15__ptrc_glDrawArrays) numFailed++;
	GL15__ptrc_glDrawElements = (void (CODEGEN_FUNCPTR *)(GLenum, GLsizei, GLenum, const void *))IntGetProcAddress("glDrawElements");
	if(!GL15__ptrc_glDrawElements) numFailed++;
	GL15__ptrc_glEdgeFlagPointer = (void (CODEGEN_FUNCPTR *)(GLsizei, const void *))IntGetProcAddress("glEdgeFlagPointer");
	if(!GL15__ptrc_glEdgeFlagPointer) numFailed++;
	GL15__ptrc_glEnableClientState = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glEnableClientState");
	if(!GL15__ptrc_glEnableClientState) numFailed++;
	GL15__ptrc_glGenTextures = (void (CODEGEN_FUNCPTR *)(GLsizei, GLuint *))IntGetProcAddress("glGenTextures");
	if(!GL15__ptrc_glGenTextures) numFailed++;
	GL15__ptrc_glGetPointerv = (void (CODEGEN_FUNCPTR *)(GLenum, void **))IntGetProcAddress("glGetPointerv");
	if(!GL15__ptrc_glGetPointerv) numFailed++;
	GL15__ptrc_glIndexPointer = (void (CODEGEN_FUNCPTR *)(GLenum, GLsizei, const void *))IntGetProcAddress("glIndexPointer");
	if(!GL15__ptrc_glIndexPointer) numFailed++;
	GL15__ptrc_glIndexub = (void (CODEGEN_FUNCPTR *)(GLubyte))IntGetProcAddress("glIndexub");
	if(!GL15__ptrc_glIndexub) numFailed++;
	GL15__ptrc_glIndexubv = (void (CODEGEN_FUNCPTR *)(const GLubyte *))IntGetProcAddress("glIndexubv");
	if(!GL15__ptrc_glIndexubv) numFailed++;
	GL15__ptrc_glInterleavedArrays = (void (CODEGEN_FUNCPTR *)(GLenum, GLsizei, const void *))IntGetProcAddress("glInterleavedArrays");
	if(!GL15__ptrc_glInterleavedArrays) numFailed++;
	GL15__ptrc_glIsTexture = (GLboolean (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glIsTexture");
	if(!GL15__ptrc_glIsTexture) numFailed++;
	GL15__ptrc_glNormalPointer = (void (CODEGEN_FUNCPTR *)(GLenum, GLsizei, const void *))IntGetProcAddress("glNormalPointer");
	if(!GL15__ptrc_glNormalPointer) numFailed++;
	GL15__ptrc_glPolygonOffset = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat))IntGetProcAddress("glPolygonOffset");
	if(!GL15__ptrc_glPolygonOffset) numFailed++;
	GL15__ptrc_glPopClientAttrib = (void (CODEGEN_FUNCPTR *)(void))IntGetProcAddress("glPopClientAttrib");
	if(!GL15__ptrc_glPopClientAttrib) numFailed++;
	GL15__ptrc_glPrioritizeTextures = (void (CODEGEN_FUNCPTR *)(GLsizei, const GLuint *, const GLfloat *))IntGetProcAddress("glPrioritizeTextures");
	if(!GL15__ptrc_glPrioritizeTextures) numFailed++;
	GL15__ptrc_glPushClientAttrib = (void (CODEGEN_FUNCPTR *)(GLbitfield))IntGetProcAddress("glPushClientAttrib");
	if(!GL15__ptrc_glPushClientAttrib) numFailed++;
	GL15__ptrc_glTexCoordPointer = (void (CODEGEN_FUNCPTR *)(GLint, GLenum, GLsizei, const void *))IntGetProcAddress("glTexCoordPointer");
	if(!GL15__ptrc_glTexCoordPointer) numFailed++;
	GL15__ptrc_glTexSubImage1D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLsizei, GLenum, GLenum, const void *))IntGetProcAddress("glTexSubImage1D");
	if(!GL15__ptrc_glTexSubImage1D) numFailed++;
	GL15__ptrc_glTexSubImage2D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, const void *))IntGetProcAddress("glTexSubImage2D");
	if(!GL15__ptrc_glTexSubImage2D) numFailed++;
	GL15__ptrc_glVertexPointer = (void (CODEGEN_FUNCPTR *)(GLint, GLenum, GLsizei, const void *))IntGetProcAddress("glVertexPointer");
	if(!GL15__ptrc_glVertexPointer) numFailed++;
	GL15__ptrc_glCopyTexSubImage3D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei))IntGetProcAddress("glCopyTexSubImage3D");
	if(!GL15__ptrc_glCopyTexSubImage3D) numFailed++;
	GL15__ptrc_glDrawRangeElements = (void (CODEGEN_FUNCPTR *)(GLenum, GLuint, GLuint, GLsizei, GLenum, const void *))IntGetProcAddress("glDrawRangeElements");
	if(!GL15__ptrc_glDrawRangeElements) numFailed++;
	GL15__ptrc_glTexImage3D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLsizei, GLsizei, GLsizei, GLint, GLenum, GLenum, const void *))IntGetProcAddress("glTexImage3D");
	if(!GL15__ptrc_glTexImage3D) numFailed++;
	GL15__ptrc_glTexSubImage3D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei, GLenum, GLenum, const void *))IntGetProcAddress("glTexSubImage3D");
	if(!GL15__ptrc_glTexSubImage3D) numFailed++;
	GL15__ptrc_glActiveTexture = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glActiveTexture");
	if(!GL15__ptrc_glActiveTexture) numFailed++;
	GL15__ptrc_glClientActiveTexture = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glClientActiveTexture");
	if(!GL15__ptrc_glClientActiveTexture) numFailed++;
	GL15__ptrc_glCompressedTexImage1D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLenum, GLsizei, GLint, GLsizei, const void *))IntGetProcAddress("glCompressedTexImage1D");
	if(!GL15__ptrc_glCompressedTexImage1D) numFailed++;
	GL15__ptrc_glCompressedTexImage2D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLint, GLsizei, const void *))IntGetProcAddress("glCompressedTexImage2D");
	if(!GL15__ptrc_glCompressedTexImage2D) numFailed++;
	GL15__ptrc_glCompressedTexImage3D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint, GLsizei, const void *))IntGetProcAddress("glCompressedTexImage3D");
	if(!GL15__ptrc_glCompressedTexImage3D) numFailed++;
	GL15__ptrc_glCompressedTexSubImage1D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLsizei, GLenum, GLsizei, const void *))IntGetProcAddress("glCompressedTexSubImage1D");
	if(!GL15__ptrc_glCompressedTexSubImage1D) numFailed++;
	GL15__ptrc_glCompressedTexSubImage2D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint, GLsizei, GLsizei, GLenum, GLsizei, const void *))IntGetProcAddress("glCompressedTexSubImage2D");
	if(!GL15__ptrc_glCompressedTexSubImage2D) numFailed++;
	GL15__ptrc_glCompressedTexSubImage3D = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei, GLenum, GLsizei, const void *))IntGetProcAddress("glCompressedTexSubImage3D");
	if(!GL15__ptrc_glCompressedTexSubImage3D) numFailed++;
	GL15__ptrc_glGetCompressedTexImage = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, void *))IntGetProcAddress("glGetCompressedTexImage");
	if(!GL15__ptrc_glGetCompressedTexImage) numFailed++;
	GL15__ptrc_glLoadTransposeMatrixd = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glLoadTransposeMatrixd");
	if(!GL15__ptrc_glLoadTransposeMatrixd) numFailed++;
	GL15__ptrc_glLoadTransposeMatrixf = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glLoadTransposeMatrixf");
	if(!GL15__ptrc_glLoadTransposeMatrixf) numFailed++;
	GL15__ptrc_glMultTransposeMatrixd = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glMultTransposeMatrixd");
	if(!GL15__ptrc_glMultTransposeMatrixd) numFailed++;
	GL15__ptrc_glMultTransposeMatrixf = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glMultTransposeMatrixf");
	if(!GL15__ptrc_glMultTransposeMatrixf) numFailed++;
	GL15__ptrc_glMultiTexCoord1d = (void (CODEGEN_FUNCPTR *)(GLenum, GLdouble))IntGetProcAddress("glMultiTexCoord1d");
	if(!GL15__ptrc_glMultiTexCoord1d) numFailed++;
	GL15__ptrc_glMultiTexCoord1dv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLdouble *))IntGetProcAddress("glMultiTexCoord1dv");
	if(!GL15__ptrc_glMultiTexCoord1dv) numFailed++;
	GL15__ptrc_glMultiTexCoord1f = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat))IntGetProcAddress("glMultiTexCoord1f");
	if(!GL15__ptrc_glMultiTexCoord1f) numFailed++;
	GL15__ptrc_glMultiTexCoord1fv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLfloat *))IntGetProcAddress("glMultiTexCoord1fv");
	if(!GL15__ptrc_glMultiTexCoord1fv) numFailed++;
	GL15__ptrc_glMultiTexCoord1i = (void (CODEGEN_FUNCPTR *)(GLenum, GLint))IntGetProcAddress("glMultiTexCoord1i");
	if(!GL15__ptrc_glMultiTexCoord1i) numFailed++;
	GL15__ptrc_glMultiTexCoord1iv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLint *))IntGetProcAddress("glMultiTexCoord1iv");
	if(!GL15__ptrc_glMultiTexCoord1iv) numFailed++;
	GL15__ptrc_glMultiTexCoord1s = (void (CODEGEN_FUNCPTR *)(GLenum, GLshort))IntGetProcAddress("glMultiTexCoord1s");
	if(!GL15__ptrc_glMultiTexCoord1s) numFailed++;
	GL15__ptrc_glMultiTexCoord1sv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLshort *))IntGetProcAddress("glMultiTexCoord1sv");
	if(!GL15__ptrc_glMultiTexCoord1sv) numFailed++;
	GL15__ptrc_glMultiTexCoord2d = (void (CODEGEN_FUNCPTR *)(GLenum, GLdouble, GLdouble))IntGetProcAddress("glMultiTexCoord2d");
	if(!GL15__ptrc_glMultiTexCoord2d) numFailed++;
	GL15__ptrc_glMultiTexCoord2dv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLdouble *))IntGetProcAddress("glMultiTexCoord2dv");
	if(!GL15__ptrc_glMultiTexCoord2dv) numFailed++;
	GL15__ptrc_glMultiTexCoord2f = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat, GLfloat))IntGetProcAddress("glMultiTexCoord2f");
	if(!GL15__ptrc_glMultiTexCoord2f) numFailed++;
	GL15__ptrc_glMultiTexCoord2fv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLfloat *))IntGetProcAddress("glMultiTexCoord2fv");
	if(!GL15__ptrc_glMultiTexCoord2fv) numFailed++;
	GL15__ptrc_glMultiTexCoord2i = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint))IntGetProcAddress("glMultiTexCoord2i");
	if(!GL15__ptrc_glMultiTexCoord2i) numFailed++;
	GL15__ptrc_glMultiTexCoord2iv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLint *))IntGetProcAddress("glMultiTexCoord2iv");
	if(!GL15__ptrc_glMultiTexCoord2iv) numFailed++;
	GL15__ptrc_glMultiTexCoord2s = (void (CODEGEN_FUNCPTR *)(GLenum, GLshort, GLshort))IntGetProcAddress("glMultiTexCoord2s");
	if(!GL15__ptrc_glMultiTexCoord2s) numFailed++;
	GL15__ptrc_glMultiTexCoord2sv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLshort *))IntGetProcAddress("glMultiTexCoord2sv");
	if(!GL15__ptrc_glMultiTexCoord2sv) numFailed++;
	GL15__ptrc_glMultiTexCoord3d = (void (CODEGEN_FUNCPTR *)(GLenum, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glMultiTexCoord3d");
	if(!GL15__ptrc_glMultiTexCoord3d) numFailed++;
	GL15__ptrc_glMultiTexCoord3dv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLdouble *))IntGetProcAddress("glMultiTexCoord3dv");
	if(!GL15__ptrc_glMultiTexCoord3dv) numFailed++;
	GL15__ptrc_glMultiTexCoord3f = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glMultiTexCoord3f");
	if(!GL15__ptrc_glMultiTexCoord3f) numFailed++;
	GL15__ptrc_glMultiTexCoord3fv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLfloat *))IntGetProcAddress("glMultiTexCoord3fv");
	if(!GL15__ptrc_glMultiTexCoord3fv) numFailed++;
	GL15__ptrc_glMultiTexCoord3i = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint))IntGetProcAddress("glMultiTexCoord3i");
	if(!GL15__ptrc_glMultiTexCoord3i) numFailed++;
	GL15__ptrc_glMultiTexCoord3iv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLint *))IntGetProcAddress("glMultiTexCoord3iv");
	if(!GL15__ptrc_glMultiTexCoord3iv) numFailed++;
	GL15__ptrc_glMultiTexCoord3s = (void (CODEGEN_FUNCPTR *)(GLenum, GLshort, GLshort, GLshort))IntGetProcAddress("glMultiTexCoord3s");
	if(!GL15__ptrc_glMultiTexCoord3s) numFailed++;
	GL15__ptrc_glMultiTexCoord3sv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLshort *))IntGetProcAddress("glMultiTexCoord3sv");
	if(!GL15__ptrc_glMultiTexCoord3sv) numFailed++;
	GL15__ptrc_glMultiTexCoord4d = (void (CODEGEN_FUNCPTR *)(GLenum, GLdouble, GLdouble, GLdouble, GLdouble))IntGetProcAddress("glMultiTexCoord4d");
	if(!GL15__ptrc_glMultiTexCoord4d) numFailed++;
	GL15__ptrc_glMultiTexCoord4dv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLdouble *))IntGetProcAddress("glMultiTexCoord4dv");
	if(!GL15__ptrc_glMultiTexCoord4dv) numFailed++;
	GL15__ptrc_glMultiTexCoord4f = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glMultiTexCoord4f");
	if(!GL15__ptrc_glMultiTexCoord4f) numFailed++;
	GL15__ptrc_glMultiTexCoord4fv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLfloat *))IntGetProcAddress("glMultiTexCoord4fv");
	if(!GL15__ptrc_glMultiTexCoord4fv) numFailed++;
	GL15__ptrc_glMultiTexCoord4i = (void (CODEGEN_FUNCPTR *)(GLenum, GLint, GLint, GLint, GLint))IntGetProcAddress("glMultiTexCoord4i");
	if(!GL15__ptrc_glMultiTexCoord4i) numFailed++;
	GL15__ptrc_glMultiTexCoord4iv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLint *))IntGetProcAddress("glMultiTexCoord4iv");
	if(!GL15__ptrc_glMultiTexCoord4iv) numFailed++;
	GL15__ptrc_glMultiTexCoord4s = (void (CODEGEN_FUNCPTR *)(GLenum, GLshort, GLshort, GLshort, GLshort))IntGetProcAddress("glMultiTexCoord4s");
	if(!GL15__ptrc_glMultiTexCoord4s) numFailed++;
	GL15__ptrc_glMultiTexCoord4sv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLshort *))IntGetProcAddress("glMultiTexCoord4sv");
	if(!GL15__ptrc_glMultiTexCoord4sv) numFailed++;
	GL15__ptrc_glSampleCoverage = (void (CODEGEN_FUNCPTR *)(GLfloat, GLboolean))IntGetProcAddress("glSampleCoverage");
	if(!GL15__ptrc_glSampleCoverage) numFailed++;
	GL15__ptrc_glBlendColor = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat, GLfloat))IntGetProcAddress("glBlendColor");
	if(!GL15__ptrc_glBlendColor) numFailed++;
	GL15__ptrc_glBlendEquation = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glBlendEquation");
	if(!GL15__ptrc_glBlendEquation) numFailed++;
	GL15__ptrc_glBlendFuncSeparate = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLenum, GLenum))IntGetProcAddress("glBlendFuncSeparate");
	if(!GL15__ptrc_glBlendFuncSeparate) numFailed++;
	GL15__ptrc_glFogCoordPointer = (void (CODEGEN_FUNCPTR *)(GLenum, GLsizei, const void *))IntGetProcAddress("glFogCoordPointer");
	if(!GL15__ptrc_glFogCoordPointer) numFailed++;
	GL15__ptrc_glFogCoordd = (void (CODEGEN_FUNCPTR *)(GLdouble))IntGetProcAddress("glFogCoordd");
	if(!GL15__ptrc_glFogCoordd) numFailed++;
	GL15__ptrc_glFogCoorddv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glFogCoorddv");
	if(!GL15__ptrc_glFogCoorddv) numFailed++;
	GL15__ptrc_glFogCoordf = (void (CODEGEN_FUNCPTR *)(GLfloat))IntGetProcAddress("glFogCoordf");
	if(!GL15__ptrc_glFogCoordf) numFailed++;
	GL15__ptrc_glFogCoordfv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glFogCoordfv");
	if(!GL15__ptrc_glFogCoordfv) numFailed++;
	GL15__ptrc_glMultiDrawArrays = (void (CODEGEN_FUNCPTR *)(GLenum, const GLint *, const GLsizei *, GLsizei))IntGetProcAddress("glMultiDrawArrays");
	if(!GL15__ptrc_glMultiDrawArrays) numFailed++;
	GL15__ptrc_glMultiDrawElements = (void (CODEGEN_FUNCPTR *)(GLenum, const GLsizei *, GLenum, const void *const*, GLsizei))IntGetProcAddress("glMultiDrawElements");
	if(!GL15__ptrc_glMultiDrawElements) numFailed++;
	GL15__ptrc_glPointParameterf = (void (CODEGEN_FUNCPTR *)(GLenum, GLfloat))IntGetProcAddress("glPointParameterf");
	if(!GL15__ptrc_glPointParameterf) numFailed++;
	GL15__ptrc_glPointParameterfv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLfloat *))IntGetProcAddress("glPointParameterfv");
	if(!GL15__ptrc_glPointParameterfv) numFailed++;
	GL15__ptrc_glPointParameteri = (void (CODEGEN_FUNCPTR *)(GLenum, GLint))IntGetProcAddress("glPointParameteri");
	if(!GL15__ptrc_glPointParameteri) numFailed++;
	GL15__ptrc_glPointParameteriv = (void (CODEGEN_FUNCPTR *)(GLenum, const GLint *))IntGetProcAddress("glPointParameteriv");
	if(!GL15__ptrc_glPointParameteriv) numFailed++;
	GL15__ptrc_glSecondaryColor3b = (void (CODEGEN_FUNCPTR *)(GLbyte, GLbyte, GLbyte))IntGetProcAddress("glSecondaryColor3b");
	if(!GL15__ptrc_glSecondaryColor3b) numFailed++;
	GL15__ptrc_glSecondaryColor3bv = (void (CODEGEN_FUNCPTR *)(const GLbyte *))IntGetProcAddress("glSecondaryColor3bv");
	if(!GL15__ptrc_glSecondaryColor3bv) numFailed++;
	GL15__ptrc_glSecondaryColor3d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble))IntGetProcAddress("glSecondaryColor3d");
	if(!GL15__ptrc_glSecondaryColor3d) numFailed++;
	GL15__ptrc_glSecondaryColor3dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glSecondaryColor3dv");
	if(!GL15__ptrc_glSecondaryColor3dv) numFailed++;
	GL15__ptrc_glSecondaryColor3f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat))IntGetProcAddress("glSecondaryColor3f");
	if(!GL15__ptrc_glSecondaryColor3f) numFailed++;
	GL15__ptrc_glSecondaryColor3fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glSecondaryColor3fv");
	if(!GL15__ptrc_glSecondaryColor3fv) numFailed++;
	GL15__ptrc_glSecondaryColor3i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint))IntGetProcAddress("glSecondaryColor3i");
	if(!GL15__ptrc_glSecondaryColor3i) numFailed++;
	GL15__ptrc_glSecondaryColor3iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glSecondaryColor3iv");
	if(!GL15__ptrc_glSecondaryColor3iv) numFailed++;
	GL15__ptrc_glSecondaryColor3s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort))IntGetProcAddress("glSecondaryColor3s");
	if(!GL15__ptrc_glSecondaryColor3s) numFailed++;
	GL15__ptrc_glSecondaryColor3sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glSecondaryColor3sv");
	if(!GL15__ptrc_glSecondaryColor3sv) numFailed++;
	GL15__ptrc_glSecondaryColor3ub = (void (CODEGEN_FUNCPTR *)(GLubyte, GLubyte, GLubyte))IntGetProcAddress("glSecondaryColor3ub");
	if(!GL15__ptrc_glSecondaryColor3ub) numFailed++;
	GL15__ptrc_glSecondaryColor3ubv = (void (CODEGEN_FUNCPTR *)(const GLubyte *))IntGetProcAddress("glSecondaryColor3ubv");
	if(!GL15__ptrc_glSecondaryColor3ubv) numFailed++;
	GL15__ptrc_glSecondaryColor3ui = (void (CODEGEN_FUNCPTR *)(GLuint, GLuint, GLuint))IntGetProcAddress("glSecondaryColor3ui");
	if(!GL15__ptrc_glSecondaryColor3ui) numFailed++;
	GL15__ptrc_glSecondaryColor3uiv = (void (CODEGEN_FUNCPTR *)(const GLuint *))IntGetProcAddress("glSecondaryColor3uiv");
	if(!GL15__ptrc_glSecondaryColor3uiv) numFailed++;
	GL15__ptrc_glSecondaryColor3us = (void (CODEGEN_FUNCPTR *)(GLushort, GLushort, GLushort))IntGetProcAddress("glSecondaryColor3us");
	if(!GL15__ptrc_glSecondaryColor3us) numFailed++;
	GL15__ptrc_glSecondaryColor3usv = (void (CODEGEN_FUNCPTR *)(const GLushort *))IntGetProcAddress("glSecondaryColor3usv");
	if(!GL15__ptrc_glSecondaryColor3usv) numFailed++;
	GL15__ptrc_glSecondaryColorPointer = (void (CODEGEN_FUNCPTR *)(GLint, GLenum, GLsizei, const void *))IntGetProcAddress("glSecondaryColorPointer");
	if(!GL15__ptrc_glSecondaryColorPointer) numFailed++;
	GL15__ptrc_glWindowPos2d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble))IntGetProcAddress("glWindowPos2d");
	if(!GL15__ptrc_glWindowPos2d) numFailed++;
	GL15__ptrc_glWindowPos2dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glWindowPos2dv");
	if(!GL15__ptrc_glWindowPos2dv) numFailed++;
	GL15__ptrc_glWindowPos2f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat))IntGetProcAddress("glWindowPos2f");
	if(!GL15__ptrc_glWindowPos2f) numFailed++;
	GL15__ptrc_glWindowPos2fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glWindowPos2fv");
	if(!GL15__ptrc_glWindowPos2fv) numFailed++;
	GL15__ptrc_glWindowPos2i = (void (CODEGEN_FUNCPTR *)(GLint, GLint))IntGetProcAddress("glWindowPos2i");
	if(!GL15__ptrc_glWindowPos2i) numFailed++;
	GL15__ptrc_glWindowPos2iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glWindowPos2iv");
	if(!GL15__ptrc_glWindowPos2iv) numFailed++;
	GL15__ptrc_glWindowPos2s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort))IntGetProcAddress("glWindowPos2s");
	if(!GL15__ptrc_glWindowPos2s) numFailed++;
	GL15__ptrc_glWindowPos2sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glWindowPos2sv");
	if(!GL15__ptrc_glWindowPos2sv) numFailed++;
	GL15__ptrc_glWindowPos3d = (void (CODEGEN_FUNCPTR *)(GLdouble, GLdouble, GLdouble))IntGetProcAddress("glWindowPos3d");
	if(!GL15__ptrc_glWindowPos3d) numFailed++;
	GL15__ptrc_glWindowPos3dv = (void (CODEGEN_FUNCPTR *)(const GLdouble *))IntGetProcAddress("glWindowPos3dv");
	if(!GL15__ptrc_glWindowPos3dv) numFailed++;
	GL15__ptrc_glWindowPos3f = (void (CODEGEN_FUNCPTR *)(GLfloat, GLfloat, GLfloat))IntGetProcAddress("glWindowPos3f");
	if(!GL15__ptrc_glWindowPos3f) numFailed++;
	GL15__ptrc_glWindowPos3fv = (void (CODEGEN_FUNCPTR *)(const GLfloat *))IntGetProcAddress("glWindowPos3fv");
	if(!GL15__ptrc_glWindowPos3fv) numFailed++;
	GL15__ptrc_glWindowPos3i = (void (CODEGEN_FUNCPTR *)(GLint, GLint, GLint))IntGetProcAddress("glWindowPos3i");
	if(!GL15__ptrc_glWindowPos3i) numFailed++;
	GL15__ptrc_glWindowPos3iv = (void (CODEGEN_FUNCPTR *)(const GLint *))IntGetProcAddress("glWindowPos3iv");
	if(!GL15__ptrc_glWindowPos3iv) numFailed++;
	GL15__ptrc_glWindowPos3s = (void (CODEGEN_FUNCPTR *)(GLshort, GLshort, GLshort))IntGetProcAddress("glWindowPos3s");
	if(!GL15__ptrc_glWindowPos3s) numFailed++;
	GL15__ptrc_glWindowPos3sv = (void (CODEGEN_FUNCPTR *)(const GLshort *))IntGetProcAddress("glWindowPos3sv");
	if(!GL15__ptrc_glWindowPos3sv) numFailed++;
	GL15__ptrc_glBeginQuery = (void (CODEGEN_FUNCPTR *)(GLenum, GLuint))IntGetProcAddress("glBeginQuery");
	if(!GL15__ptrc_glBeginQuery) numFailed++;
	GL15__ptrc_glBindBuffer = (void (CODEGEN_FUNCPTR *)(GLenum, GLuint))IntGetProcAddress("glBindBuffer");
	if(!GL15__ptrc_glBindBuffer) numFailed++;
	GL15__ptrc_glBufferData = (void (CODEGEN_FUNCPTR *)(GLenum, GLsizeiptr, const void *, GLenum))IntGetProcAddress("glBufferData");
	if(!GL15__ptrc_glBufferData) numFailed++;
	GL15__ptrc_glBufferSubData = (void (CODEGEN_FUNCPTR *)(GLenum, GLintptr, GLsizeiptr, const void *))IntGetProcAddress("glBufferSubData");
	if(!GL15__ptrc_glBufferSubData) numFailed++;
	GL15__ptrc_glDeleteBuffers = (void (CODEGEN_FUNCPTR *)(GLsizei, const GLuint *))IntGetProcAddress("glDeleteBuffers");
	if(!GL15__ptrc_glDeleteBuffers) numFailed++;
	GL15__ptrc_glDeleteQueries = (void (CODEGEN_FUNCPTR *)(GLsizei, const GLuint *))IntGetProcAddress("glDeleteQueries");
	if(!GL15__ptrc_glDeleteQueries) numFailed++;
	GL15__ptrc_glEndQuery = (void (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glEndQuery");
	if(!GL15__ptrc_glEndQuery) numFailed++;
	GL15__ptrc_glGenBuffers = (void (CODEGEN_FUNCPTR *)(GLsizei, GLuint *))IntGetProcAddress("glGenBuffers");
	if(!GL15__ptrc_glGenBuffers) numFailed++;
	GL15__ptrc_glGenQueries = (void (CODEGEN_FUNCPTR *)(GLsizei, GLuint *))IntGetProcAddress("glGenQueries");
	if(!GL15__ptrc_glGenQueries) numFailed++;
	GL15__ptrc_glGetBufferParameteriv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint *))IntGetProcAddress("glGetBufferParameteriv");
	if(!GL15__ptrc_glGetBufferParameteriv) numFailed++;
	GL15__ptrc_glGetBufferPointerv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, void **))IntGetProcAddress("glGetBufferPointerv");
	if(!GL15__ptrc_glGetBufferPointerv) numFailed++;
	GL15__ptrc_glGetBufferSubData = (void (CODEGEN_FUNCPTR *)(GLenum, GLintptr, GLsizeiptr, void *))IntGetProcAddress("glGetBufferSubData");
	if(!GL15__ptrc_glGetBufferSubData) numFailed++;
	GL15__ptrc_glGetQueryObjectiv = (void (CODEGEN_FUNCPTR *)(GLuint, GLenum, GLint *))IntGetProcAddress("glGetQueryObjectiv");
	if(!GL15__ptrc_glGetQueryObjectiv) numFailed++;
	GL15__ptrc_glGetQueryObjectuiv = (void (CODEGEN_FUNCPTR *)(GLuint, GLenum, GLuint *))IntGetProcAddress("glGetQueryObjectuiv");
	if(!GL15__ptrc_glGetQueryObjectuiv) numFailed++;
	GL15__ptrc_glGetQueryiv = (void (CODEGEN_FUNCPTR *)(GLenum, GLenum, GLint *))IntGetProcAddress("glGetQueryiv");
	if(!GL15__ptrc_glGetQueryiv) numFailed++;
	GL15__ptrc_glIsBuffer = (GLboolean (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glIsBuffer");
	if(!GL15__ptrc_glIsBuffer) numFailed++;
	GL15__ptrc_glIsQuery = (GLboolean (CODEGEN_FUNCPTR *)(GLuint))IntGetProcAddress("glIsQuery");
	if(!GL15__ptrc_glIsQuery) numFailed++;
	GL15__ptrc_glMapBuffer = (void * (CODEGEN_FUNCPTR *)(GLenum, GLenum))IntGetProcAddress("glMapBuffer");
	if(!GL15__ptrc_glMapBuffer) numFailed++;
	GL15__ptrc_glUnmapBuffer = (GLboolean (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glUnmapBuffer");
	if(!GL15__ptrc_glUnmapBuffer) numFailed++;
	return numFailed;
}

typedef int (*PFN_LOADFUNCPOINTERS)(void);
typedef struct GL15_ogl_StrToExtMap_s
{
	char *extensionName;
	int *extensionVariable;
	PFN_LOADFUNCPOINTERS LoadExtension;
} GL15_ogl_StrToExtMap;

static GL15_ogl_StrToExtMap ExtensionMap[1] = {
	{"", NULL, NULL},
};

static int g_extensionMapSize = 0;

static GL15_ogl_StrToExtMap *FindExtEntry(const char *extensionName)
{
	int loop;
	GL15_ogl_StrToExtMap *currLoc = ExtensionMap;
	for(loop = 0; loop < g_extensionMapSize; ++loop, ++currLoc)
	{
		if(strcmp(extensionName, currLoc->extensionName) == 0)
			return currLoc;
	}
	
	return NULL;
}

static void ClearExtensionVars(void)
{
}


static void LoadExtByName(const char *extensionName)
{
	GL15_ogl_StrToExtMap *entry = NULL;
	entry = FindExtEntry(extensionName);
	if(entry)
	{
		if(entry->LoadExtension)
		{
			int numFailed = entry->LoadExtension();
			if(numFailed == 0)
			{
				*(entry->extensionVariable) = GL15_ogl_LOAD_SUCCEEDED;
			}
			else
			{
				*(entry->extensionVariable) = GL15_ogl_LOAD_SUCCEEDED + numFailed;
			}
		}
		else
		{
			*(entry->extensionVariable) = GL15_ogl_LOAD_SUCCEEDED;
		}
	}
}


static void ProcExtsFromExtString(const char *strExtList)
{
	size_t iExtListLen = strlen(strExtList);
	const char *strExtListEnd = strExtList + iExtListLen;
	const char *strCurrPos = strExtList;
	char strWorkBuff[256];

	while(*strCurrPos)
	{
		/*Get the extension at our position.*/
		int iStrLen = 0;
		const char *strEndStr = strchr(strCurrPos, ' ');
		int iStop = 0;
		if(strEndStr == NULL)
		{
			strEndStr = strExtListEnd;
			iStop = 1;
		}

		iStrLen = (int)((ptrdiff_t)strEndStr - (ptrdiff_t)strCurrPos);

		if(iStrLen > 255)
			return;

		strncpy(strWorkBuff, strCurrPos, iStrLen);
		strWorkBuff[iStrLen] = '\0';

		LoadExtByName(strWorkBuff);

		strCurrPos = strEndStr + 1;
		if(iStop) break;
	}
}

int GL15_ogl_LoadFunctions()
{
	int numFailed = 0;
	ClearExtensionVars();
	
	GL15__ptrc_glGetString = (const GLubyte * (CODEGEN_FUNCPTR *)(GLenum))IntGetProcAddress("glGetString");
	if(!GL15__ptrc_glGetString) return GL15_ogl_LOAD_FAILED;
	
	ProcExtsFromExtString((const char *)GL15__ptrc_glGetString(GL_EXTENSIONS));
	numFailed = Load_Version_1_5();
	
	if(numFailed == 0)
		return GL15_ogl_LOAD_SUCCEEDED;
	else
		return GL15_ogl_LOAD_SUCCEEDED + numFailed;
}

static int g_major_version = 0;
static int g_minor_version = 0;

static void ParseVersionFromString(int *pOutMajor, int *pOutMinor, const char *strVersion)
{
	const char *strDotPos = NULL;
	int iLength = 0;
	char strWorkBuff[10];
	*pOutMinor = 0;
	*pOutMajor = 0;

	strDotPos = strchr(strVersion, '.');
	if(!strDotPos)
		return;

	iLength = (int)((ptrdiff_t)strDotPos - (ptrdiff_t)strVersion);
	strncpy(strWorkBuff, strVersion, iLength);
	strWorkBuff[iLength] = '\0';

	*pOutMajor = atoi(strWorkBuff);
	strDotPos = strchr(strVersion + iLength + 1, ' ');
	if(!strDotPos)
	{
		/*No extra data. Take the whole rest of the string.*/
		strcpy(strWorkBuff, strVersion + iLength + 1);
	}
	else
	{
		/*Copy only up until the space.*/
		int iLengthMinor = (int)((ptrdiff_t)strDotPos - (ptrdiff_t)strVersion);
		iLengthMinor = iLengthMinor - (iLength + 1);
		strncpy(strWorkBuff, strVersion + iLength + 1, iLengthMinor);
		strWorkBuff[iLengthMinor] = '\0';
	}

	*pOutMinor = atoi(strWorkBuff);
}

static void GetGLVersion(void)
{
	ParseVersionFromString(&g_major_version, &g_minor_version, (const char*)glGetString(GL_VERSION));
}

int GL15_ogl_GetMajorVersion(void)
{
	if(g_major_version == 0)
		GetGLVersion();
	return g_major_version;
}

int GL15_ogl_GetMinorVersion(void)
{
	if(g_major_version == 0) /*Yes, check the major version to get the minor one.*/
		GetGLVersion();
	return g_minor_version;
}

int GL15_ogl_IsVersionGEQ(int majorVersion, int minorVersion)
{
	if(g_major_version == 0)
		GetGLVersion();
	
	if(majorVersion < g_major_version) return 1;
	if(majorVersion > g_major_version) return 0;
	if(minorVersion <= g_minor_version) return 1;
	return 0;
}

