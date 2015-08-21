#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import serial
import time
import math

window = 0
pitch = 0.0
roll = 0.0
yaw = 0.0

LightAmbient = 0.2, 0.2, 0.2, 1.0
LightDiffuse = 0.5, 0.5, 0.5, 1.0
LightPosition =	1.0, 0.0, 2.0, 1.0

ser = serial.Serial("/dev/ttyACM0", 115200)
ser.write("output_euler\n")
ser.write("auto_on\n")
ser.write("cal_data: -854 358 -548 316 -478 813\n")

def InitGL(Width, Height):				# We call this right after our OpenGL window is created.
	global LightAmbient, LightDiffuse, LightPosition
	glClearColor(0.0, 0.0, 0.0, 0.0)	# This Will Clear The Background Color To Black
	glClearDepth(1.0)					# Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LESS)				# The Type Of Depth Test To Do
	glEnable(GL_DEPTH_TEST)				# Enables Depth Testing
	glShadeModel(GL_SMOOTH)				# Enables Smooth Color Shading
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)
	
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()					# Reset The Projection Matrix
										# Calculate The Aspect Ratio Of The Window
	gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)

	glMatrixMode(GL_MODELVIEW)

	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient)
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse)
	glLightfv(GL_LIGHT1, GL_POSITION,LightPosition)
	glEnable(GL_LIGHT1)
	glEnable(GL_LIGHTING)
	glEnable(GL_COLOR_MATERIAL)

def ReSizeGLScene(Width, Height):
    if Height == 0:
	    Height = 1

    glViewport(0, 0, Width, Height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

def DrawGLScene():
	global pitch, roll, yaw
	line = ser.readline()
	line.strip("\r\n")
	words = line.split(" ")
	if(words[0] == "euler:"):
		yaw = float(words[1])
		pitch = float(words[2])
		roll = float(words[3])

	print yaw
	print pitch
	print roll

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

	glLoadIdentity()					# Reset The View 
	glTranslatef(0.0, 0.5, -6.0)
	glPushMatrix()
	glRotatef(yaw, 0.0, 1.0, 0.0)
	glRotatef(pitch, 1.0, 0.0, 0.0)
	glRotatef(roll, 0.0, 0.0, 1.0)
	glColor3f(0.0, 0.8, 0.9)
	glutSolidTeapot(1.0)
	glPopMatrix()


	glutSwapBuffers()

def main():
	global window
	glutInit("")
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
	glutInitWindowSize(640, 480)
	glutInitWindowPosition(0, 0)
	window = glutCreateWindow("AHRS Tester")

	glutDisplayFunc (DrawGLScene)
	#glutFullScreen()
	glutIdleFunc(DrawGLScene)
	glutReshapeFunc (ReSizeGLScene)
	InitGL(640, 480)
	glutMainLoop()

print "Hit ESC key to quit."
main()
