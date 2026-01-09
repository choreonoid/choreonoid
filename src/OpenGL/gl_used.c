/**
 * Custom OpenGL function loader for Choreonoid
 *
 * This file loads only the OpenGL functions that are actually used by Choreonoid,
 * instead of loading all ~3000 functions from the full OpenGL specification.
 * The GLAD-generated gl.h is still used for type definitions and constants.
 */

#include "gl.h"
#include <stdio.h>
#include <string.h>

/**
 * Load only the OpenGL functions used by Choreonoid.
 * This is a replacement for gladLoadGL() that loads ~120 functions
 * instead of ~3000.
 *
 * @param load Function pointer to get OpenGL function addresses
 *             (e.g., eglGetProcAddress, wglGetProcAddress, etc.)
 * @return OpenGL version as (major * 10000 + minor), or 0 on failure
 */
int cnoidLoadGL(GLADloadfunc load)
{
    /* Load basic functions first to get version info */
    glad_glGetString = (PFNGLGETSTRINGPROC)load("glGetString");
    glad_glGetIntegerv = (PFNGLGETINTEGERVPROC)load("glGetIntegerv");

    if(!glad_glGetString) return 0;
    const char* version = (const char*)glad_glGetString(GL_VERSION);
    if(!version) return 0;

    int major = 0, minor = 0;
    sscanf(version, "%d.%d", &major, &minor);

    /* Load all OpenGL functions used by Choreonoid (alphabetical order) */
    glad_glActiveTexture = (PFNGLACTIVETEXTUREPROC)load("glActiveTexture");
    glad_glAttachShader = (PFNGLATTACHSHADERPROC)load("glAttachShader");
    glad_glBindBuffer = (PFNGLBINDBUFFERPROC)load("glBindBuffer");
    glad_glBindBufferBase = (PFNGLBINDBUFFERBASEPROC)load("glBindBufferBase");
    glad_glBindFramebuffer = (PFNGLBINDFRAMEBUFFERPROC)load("glBindFramebuffer");
    glad_glBindRenderbuffer = (PFNGLBINDRENDERBUFFERPROC)load("glBindRenderbuffer");
    glad_glBindSampler = (PFNGLBINDSAMPLERPROC)load("glBindSampler");
    glad_glBindTexture = (PFNGLBINDTEXTUREPROC)load("glBindTexture");
    glad_glBindVertexArray = (PFNGLBINDVERTEXARRAYPROC)load("glBindVertexArray");
    glad_glBlendFunc = (PFNGLBLENDFUNCPROC)load("glBlendFunc");
    glad_glBlitFramebuffer = (PFNGLBLITFRAMEBUFFERPROC)load("glBlitFramebuffer");
    glad_glBufferData = (PFNGLBUFFERDATAPROC)load("glBufferData");
    glad_glBufferSubData = (PFNGLBUFFERSUBDATAPROC)load("glBufferSubData");
    glad_glCheckFramebufferStatus = (PFNGLCHECKFRAMEBUFFERSTATUSPROC)load("glCheckFramebufferStatus");
    glad_glClear = (PFNGLCLEARPROC)load("glClear");
    glad_glClearColor = (PFNGLCLEARCOLORPROC)load("glClearColor");
    glad_glClearDepth = (PFNGLCLEARDEPTHPROC)load("glClearDepth");
    glad_glClearStencil = (PFNGLCLEARSTENCILPROC)load("glClearStencil");
    glad_glColor4f = (PFNGLCOLOR4FPROC)load("glColor4f");
    glad_glColorMask = (PFNGLCOLORMASKPROC)load("glColorMask");
    glad_glColorMaterial = (PFNGLCOLORMATERIALPROC)load("glColorMaterial");
    glad_glColorPointer = (PFNGLCOLORPOINTERPROC)load("glColorPointer");
    glad_glCompileShader = (PFNGLCOMPILESHADERPROC)load("glCompileShader");
    glad_glCreateProgram = (PFNGLCREATEPROGRAMPROC)load("glCreateProgram");
    glad_glCreateShader = (PFNGLCREATESHADERPROC)load("glCreateShader");
    glad_glCullFace = (PFNGLCULLFACEPROC)load("glCullFace");
    glad_glDeleteBuffers = (PFNGLDELETEBUFFERSPROC)load("glDeleteBuffers");
    glad_glDeleteFramebuffers = (PFNGLDELETEFRAMEBUFFERSPROC)load("glDeleteFramebuffers");
    glad_glDeleteProgram = (PFNGLDELETEPROGRAMPROC)load("glDeleteProgram");
    glad_glDeleteRenderbuffers = (PFNGLDELETERENDERBUFFERSPROC)load("glDeleteRenderbuffers");
    glad_glDeleteSamplers = (PFNGLDELETESAMPLERSPROC)load("glDeleteSamplers");
    glad_glDeleteShader = (PFNGLDELETESHADERPROC)load("glDeleteShader");
    glad_glDeleteTextures = (PFNGLDELETETEXTURESPROC)load("glDeleteTextures");
    glad_glDeleteVertexArrays = (PFNGLDELETEVERTEXARRAYSPROC)load("glDeleteVertexArrays");
    glad_glDepthFunc = (PFNGLDEPTHFUNCPROC)load("glDepthFunc");
    glad_glDepthMask = (PFNGLDEPTHMASKPROC)load("glDepthMask");
    glad_glDisable = (PFNGLDISABLEPROC)load("glDisable");
    glad_glDisableClientState = (PFNGLDISABLECLIENTSTATEPROC)load("glDisableClientState");
    glad_glDrawArrays = (PFNGLDRAWARRAYSPROC)load("glDrawArrays");
    glad_glDrawBuffer = (PFNGLDRAWBUFFERPROC)load("glDrawBuffer");
    glad_glEnable = (PFNGLENABLEPROC)load("glEnable");
    glad_glEnableClientState = (PFNGLENABLECLIENTSTATEPROC)load("glEnableClientState");
    glad_glEnableVertexAttribArray = (PFNGLENABLEVERTEXATTRIBARRAYPROC)load("glEnableVertexAttribArray");
    glad_glFlush = (PFNGLFLUSHPROC)load("glFlush");
    glad_glFogf = (PFNGLFOGFPROC)load("glFogf");
    glad_glFogfv = (PFNGLFOGFVPROC)load("glFogfv");
    glad_glFogi = (PFNGLFOGIPROC)load("glFogi");
    glad_glFramebufferRenderbuffer = (PFNGLFRAMEBUFFERRENDERBUFFERPROC)load("glFramebufferRenderbuffer");
    glad_glFramebufferTexture2D = (PFNGLFRAMEBUFFERTEXTURE2DPROC)load("glFramebufferTexture2D");
    glad_glFrontFace = (PFNGLFRONTFACEPROC)load("glFrontFace");
    glad_glGenBuffers = (PFNGLGENBUFFERSPROC)load("glGenBuffers");
    glad_glGenFramebuffers = (PFNGLGENFRAMEBUFFERSPROC)load("glGenFramebuffers");
    glad_glGenRenderbuffers = (PFNGLGENRENDERBUFFERSPROC)load("glGenRenderbuffers");
    glad_glGenSamplers = (PFNGLGENSAMPLERSPROC)load("glGenSamplers");
    glad_glGenTextures = (PFNGLGENTEXTURESPROC)load("glGenTextures");
    glad_glGenVertexArrays = (PFNGLGENVERTEXARRAYSPROC)load("glGenVertexArrays");
    glad_glGenerateMipmap = (PFNGLGENERATEMIPMAPPROC)load("glGenerateMipmap");
    glad_glGetActiveUniformBlockiv = (PFNGLGETACTIVEUNIFORMBLOCKIVPROC)load("glGetActiveUniformBlockiv");
    glad_glGetActiveUniformsiv = (PFNGLGETACTIVEUNIFORMSIVPROC)load("glGetActiveUniformsiv");
    glad_glGetAttachedShaders = (PFNGLGETATTACHEDSHADERSPROC)load("glGetAttachedShaders");
    glad_glGetProgramInfoLog = (PFNGLGETPROGRAMINFOLOGPROC)load("glGetProgramInfoLog");
    glad_glGetProgramiv = (PFNGLGETPROGRAMIVPROC)load("glGetProgramiv");
    glad_glGetShaderInfoLog = (PFNGLGETSHADERINFOLOGPROC)load("glGetShaderInfoLog");
    glad_glGetShaderiv = (PFNGLGETSHADERIVPROC)load("glGetShaderiv");
    glad_glGetSubroutineIndex = (PFNGLGETSUBROUTINEINDEXPROC)load("glGetSubroutineIndex");
    glad_glGetUniformBlockIndex = (PFNGLGETUNIFORMBLOCKINDEXPROC)load("glGetUniformBlockIndex");
    glad_glGetUniformIndices = (PFNGLGETUNIFORMINDICESPROC)load("glGetUniformIndices");
    glad_glGetUniformLocation = (PFNGLGETUNIFORMLOCATIONPROC)load("glGetUniformLocation");
    glad_glLightModelfv = (PFNGLLIGHTMODELFVPROC)load("glLightModelfv");
    glad_glLightModeli = (PFNGLLIGHTMODELIPROC)load("glLightModeli");
    glad_glLightf = (PFNGLLIGHTFPROC)load("glLightf");
    glad_glLightfv = (PFNGLLIGHTFVPROC)load("glLightfv");
    glad_glLineWidth = (PFNGLLINEWIDTHPROC)load("glLineWidth");
    glad_glLinkProgram = (PFNGLLINKPROGRAMPROC)load("glLinkProgram");
    glad_glLoadIdentity = (PFNGLLOADIDENTITYPROC)load("glLoadIdentity");
    glad_glLoadMatrixd = (PFNGLLOADMATRIXDPROC)load("glLoadMatrixd");
    glad_glMaterialf = (PFNGLMATERIALFPROC)load("glMaterialf");
    glad_glMaterialfv = (PFNGLMATERIALFVPROC)load("glMaterialfv");
    glad_glMatrixMode = (PFNGLMATRIXMODEPROC)load("glMatrixMode");
    glad_glMultMatrixd = (PFNGLMULTMATRIXDPROC)load("glMultMatrixd");
    glad_glNormalPointer = (PFNGLNORMALPOINTERPROC)load("glNormalPointer");
    glad_glOrtho = (PFNGLORTHOPROC)load("glOrtho");
    glad_glPixelStorei = (PFNGLPIXELSTOREIPROC)load("glPixelStorei");
    glad_glPointSize = (PFNGLPOINTSIZEPROC)load("glPointSize");
    glad_glPolygonMode = (PFNGLPOLYGONMODEPROC)load("glPolygonMode");
    glad_glPopAttrib = (PFNGLPOPATTRIBPROC)load("glPopAttrib");
    glad_glPopClientAttrib = (PFNGLPOPCLIENTATTRIBPROC)load("glPopClientAttrib");
    glad_glPopMatrix = (PFNGLPOPMATRIXPROC)load("glPopMatrix");
    glad_glPushAttrib = (PFNGLPUSHATTRIBPROC)load("glPushAttrib");
    glad_glPushClientAttrib = (PFNGLPUSHCLIENTATTRIBPROC)load("glPushClientAttrib");
    glad_glPushMatrix = (PFNGLPUSHMATRIXPROC)load("glPushMatrix");
    glad_glReadBuffer = (PFNGLREADBUFFERPROC)load("glReadBuffer");
    glad_glReadPixels = (PFNGLREADPIXELSPROC)load("glReadPixels");
    glad_glRenderbufferStorage = (PFNGLRENDERBUFFERSTORAGEPROC)load("glRenderbufferStorage");
    glad_glRenderbufferStorageMultisample = (PFNGLRENDERBUFFERSTORAGEMULTISAMPLEPROC)load("glRenderbufferStorageMultisample");
    glad_glRotated = (PFNGLROTATEDPROC)load("glRotated");
    glad_glSamplerParameteri = (PFNGLSAMPLERPARAMETERIPROC)load("glSamplerParameteri");
    glad_glScaled = (PFNGLSCALEDPROC)load("glScaled");
    glad_glScissor = (PFNGLSCISSORPROC)load("glScissor");
    glad_glShadeModel = (PFNGLSHADEMODELPROC)load("glShadeModel");
    glad_glShaderSource = (PFNGLSHADERSOURCEPROC)load("glShaderSource");
    glad_glStencilFunc = (PFNGLSTENCILFUNCPROC)load("glStencilFunc");
    glad_glStencilOp = (PFNGLSTENCILOPPROC)load("glStencilOp");
    glad_glTexCoordPointer = (PFNGLTEXCOORDPOINTERPROC)load("glTexCoordPointer");
    glad_glTexEnvi = (PFNGLTEXENVIPROC)load("glTexEnvi");
    glad_glTexImage2D = (PFNGLTEXIMAGE2DPROC)load("glTexImage2D");
    glad_glTexImage2DMultisample = (PFNGLTEXIMAGE2DMULTISAMPLEPROC)load("glTexImage2DMultisample");
    glad_glTexParameterfv = (PFNGLTEXPARAMETERFVPROC)load("glTexParameterfv");
    glad_glTexParameteri = (PFNGLTEXPARAMETERIPROC)load("glTexParameteri");
    glad_glTexSubImage2D = (PFNGLTEXSUBIMAGE2DPROC)load("glTexSubImage2D");
    glad_glTranslated = (PFNGLTRANSLATEDPROC)load("glTranslated");
    glad_glUniform1f = (PFNGLUNIFORM1FPROC)load("glUniform1f");
    glad_glUniform1i = (PFNGLUNIFORM1IPROC)load("glUniform1i");
    glad_glUniform2f = (PFNGLUNIFORM2FPROC)load("glUniform2f");
    glad_glUniform2i = (PFNGLUNIFORM2IPROC)load("glUniform2i");
    glad_glUniform3fv = (PFNGLUNIFORM3FVPROC)load("glUniform3fv");
    glad_glUniform4fv = (PFNGLUNIFORM4FVPROC)load("glUniform4fv");
    glad_glUniformBlockBinding = (PFNGLUNIFORMBLOCKBINDINGPROC)load("glUniformBlockBinding");
    glad_glUniformMatrix3fv = (PFNGLUNIFORMMATRIX3FVPROC)load("glUniformMatrix3fv");
    glad_glUniformMatrix4fv = (PFNGLUNIFORMMATRIX4FVPROC)load("glUniformMatrix4fv");
    glad_glUniformSubroutinesuiv = (PFNGLUNIFORMSUBROUTINESUIVPROC)load("glUniformSubroutinesuiv");
    glad_glUseProgram = (PFNGLUSEPROGRAMPROC)load("glUseProgram");
    glad_glValidateProgram = (PFNGLVALIDATEPROGRAMPROC)load("glValidateProgram");
    glad_glVertexAttribPointer = (PFNGLVERTEXATTRIBPOINTERPROC)load("glVertexAttribPointer");
    glad_glVertexPointer = (PFNGLVERTEXPOINTERPROC)load("glVertexPointer");
    glad_glViewport = (PFNGLVIEWPORTPROC)load("glViewport");

    /* Load glGetStringi for extension detection (OpenGL 3.0+) */
    glad_glGetStringi = (PFNGLGETSTRINGIPROC)load("glGetStringi");

    /* Detect GL_ARB_clip_control extension */
    GLAD_GL_ARB_clip_control = 0;
    if(major >= 5 || (major == 4 && minor >= 5)){
        /* OpenGL 4.5+ has clip_control as core feature */
        GLAD_GL_ARB_clip_control = 1;
    } else if(glad_glGetStringi && glad_glGetIntegerv){
        /* Check for ARB_clip_control extension in OpenGL 3.0+ */
        GLint numExtensions = 0;
        glad_glGetIntegerv(GL_NUM_EXTENSIONS, &numExtensions);
        for(GLint i = 0; i < numExtensions; i++){
            const char* ext = (const char*)glad_glGetStringi(GL_EXTENSIONS, i);
            if(ext && strcmp(ext, "GL_ARB_clip_control") == 0){
                GLAD_GL_ARB_clip_control = 1;
                break;
            }
        }
    }

    /* Load glClipControl only if the extension is available */
    glad_glClipControl = NULL;
    if(GLAD_GL_ARB_clip_control){
        glad_glClipControl = (PFNGLCLIPCONTROLPROC)load("glClipControl");
    }

    return major * 10000 + minor;
}
