from __future__ import division
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math, sys, time

import pygame


# noinspection PyPep8Naming
class Game:
    def __init__(self):
        self.TheResizedImage = None
        self.windowSize = [640, 480]
        self.proj_matrix = None
        self.modelview_matrix = None
        self.TheMarkerSize = 0.1
        self.isYPerpendicular = False
        self.ready = False


    def axis(self, size):
        glColor3f(1, 0, 0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(size, 0.0, 0.0)
        glEnd()

        glColor3f(0, 1, 0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, size, 0.0)
        glEnd()

        glColor3f(0, 0, 1)
        glBegin(GL_LINES)
        glVertex3f(0., 0.0, 0.0)
        glVertex3f(0.0, 0.0,-size)
        glEnd()


    def idle(self):
        self.vDrawScene()


    def vDrawScene(self):


        if self.TheResizedImage is None:
            return

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, self.windowSize[0], 0, self.windowSize[1], -1.0, 1.0)
        glViewport(0, 0, self.windowSize[0], self.windowSize[1])
        glDisable(GL_TEXTURE_2D)
        glPixelZoom(1, -1)
        glRasterPos3f(0, self.windowSize[1] - 0.5, -1.0)

        glDrawPixels(self.windowSize[0], self.windowSize[1], GL_RGB, GL_UNSIGNED_BYTE, self.TheResizedImage)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        if self.proj_matrix is not None:
            glLoadMatrixd(self.proj_matrix)
        glLineWidth(2)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        if self.modelview_matrix is not None:
            glLoadMatrixd(self.modelview_matrix)
            glColor3f(0, 1, 0)

        self.axis(self.TheMarkerSize)

        if self.isYPerpendicular:
            glTranslatef(0, self.TheMarkerSize / 2, 0)
        else:
            glTranslatef(0, 0, self.TheMarkerSize / 2)



        glutSwapBuffers()


    def start(self):
        argv = glutInit(sys.argv)
        glutInitWindowPosition(0, 0)
        glutInitWindowSize(self.windowSize[0], self.windowSize[1])
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)
        glutCreateWindow("Aruco test")

        glutDisplayFunc(self.vDrawScene)
        glutIdleFunc(self.idle)
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glClearDepth(1.0)

        self.vDrawScene()
        self.ready = True
        glutMainLoop()
