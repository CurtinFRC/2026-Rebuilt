from __future__ import annotations
import ctypes
import numpy as np
from pyglet.gl import (
    glMatrixMode, glLoadIdentity, glLoadMatrixf,
    glEnable, glDisable, glBlendFunc,
    glClearColor, glDepthFunc, glViewport,
    glHint, glShadeModel,
    GL_PROJECTION, GL_MODELVIEW,
    GL_DEPTH_TEST, GL_LEQUAL,
    GL_BLEND, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,
    GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST,
    GL_SMOOTH,
)

def init_gl():
    glClearColor(0.06, 0.06, 0.07, 1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    glShadeModel(GL_SMOOTH)

def _as_gl_mat4(m: np.ndarray):
    a = np.asarray(m, dtype=np.float32)
    if a.shape != (4, 4):
        a = a.reshape((4, 4))
    a = a.T.copy()
    return (ctypes.c_float * 16)(*a.ravel(order="C"))

def set_matrices(proj: np.ndarray, view: np.ndarray):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glLoadMatrixf(_as_gl_mat4(proj))

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glLoadMatrixf(_as_gl_mat4(view))

def set_viewport(w: int, h: int):
    glViewport(0, 0, int(max(1, w)), int(max(1, h)))
