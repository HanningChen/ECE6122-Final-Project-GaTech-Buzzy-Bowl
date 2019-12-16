/*
	Author:Hanning Chen
	Class : ECE6122
	Last Date Modified : 2019.12.03
	Description : This the source code for ECE6122 final project. In this code, I simulate
				  the movement of uavs and rendering the footable field. The rendering code 
				  is referenced Dr.Jeffery's code SimpleTexture.cpp, I make some change to 
				  make it fit for this project.  
*/
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <mpi.h>
#include "iomanip"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <GL/glut.h>
#include <chrono>
#include <thread>
#include"ECE_Bitmap.h"
using namespace std;

// Send location and velocity vector in each direction
// x, y, z, vx, vy, vz
const int numElements = 6; 
// (Main task + 15 UAVs) * numElements
const int rcvSize = 16 * 6; 
//rcvbuffer is the buffer that holds the message gather from other threads
double* rcvbuffer = new double[rcvSize];
//sendBuffer is the buffer used to send the local thread message to other thread
double sendBuffer[numElements];
//Here we set this to identift the phase UAVs's during the kinetic process
int uavPhase = 0;
//To use Hook's law we should set the spring coefficient k
double k = -1 * 0.1;
//Here we define the coordination of the UAVs
double coordX;
double coordY;
double coordZ;
//Here we define the velocity of the UAVs
double velocityX;
double velocityY;
double velocityZ;
//Here we define the thruster of the UAVs
double forceX;
double forceY;
double forceZ;
//Here we define the acceleraton of UVAs
double acceX;
double acceY;
double acceZ;
//Here we transfer the width and length of field from yard to meter
double fieldWidth = (53 + (double)1 / 3) * 0.9144;
double fieldLength = 0.9144 * 100;
//Here we define each ball's x direction and y direction's gap
double gapX = 0.25 * fieldLength;
double gapY = 0.5 * fieldWidth;
//Here we define the .bmp dimension
double fieldDrawLength = 0.9144 * 126;
double fieldDrawWidth  = (53 + (double)1 / 3 + 6) * 0.9144;
//Here we define the color parameter of UAVs
double colorR = 0;
double colorG = 0;
double colorB = 255;
//We use color mark indentify whether color value drop or rise 
int colorMark = -1;
//Here we set the parameters of OpenGL
double eye_x = 00;
double eye_y = 80;
double eye_z = 100;
//Here we set the center that the eyes looking that
double center_x = 0;
double center_y = 0;
double center_z = 0;
//Here we define the time function
void timerFunction(int id);

//Here we set for texture mapping
//We set the textture array 
GLuint texture[2];
//Here we define image struct to store the data about the image
struct Image 
{
	unsigned long sizeX;
	unsigned long sizeY;
	char* data;
};
typedef struct Image Image;
//Here we define a dirver named:inBitmap to read the image
BMP inBitmap;
//Here we define the image width
#define checkImageWidth 64
//Here we define the image height
#define checkImageHeight 64
//Here we define the checkImage array to store the imformation of image
GLubyte checkImage[checkImageWidth][checkImageHeight][3];
//makeCheckImage is used to initiate the image parameter
void makeCheckImage(void) {
	//i,j are pixel's x and y coordination
	//c is the value of imgae's R,G,B
	int i, j, c;
	//we iterate an image to allocate value for each pixel's R,G,B
	for (i = 0; i < checkImageWidth; i++) {
		for (j = 0; j < checkImageHeight; j++) {
			c = ((((i & 0x8) == 0) ^ ((j & 0x8) == 0))) * 255;
			checkImage[i][j][0] = (GLubyte)c;
			checkImage[i][j][1] = (GLubyte)c;
			checkImage[i][j][2] = (GLubyte)c;
		}
	}
}
//Here we calculate the normal vector of the uav,we input the rank id
//and return the normal vector as an array
double* calNormal(int rank)
{
	//normal is the return array
	double* normal = new double[3];
	//tempX, tempY and tempZ is the value identify the x,y,z of normal vector
	//whihc points to the center of the sphere
	double tempX, tempY, tempZ;
	tempX = 0 - coordX;
	tempY = 0 - coordY;
	tempZ = 50 - coordZ;
	//We use normal vector divide the magnititude of vector to get the unit normal vector
	normal[0] = tempX / (sqrt(tempX * tempX + tempY * tempY + tempZ * tempZ));
	normal[1] = tempY / (sqrt(tempX * tempX + tempY * tempY + tempZ * tempZ));
	normal[2] = tempZ / (sqrt(tempX * tempX + tempY * tempY + tempZ * tempZ));
	return normal;
}
//Here we define the Field Drawing function
void displayFootballField()
{
	glPushMatrix();
	//We should bind texture array first
	glBindTexture(GL_TEXTURE_2D, texture[1]);
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	//We draw the 4 points of a field
	glBegin(GL_QUADS);
	glTexCoord2f(1, 0);
	glVertex3d(-1 * fieldDrawLength * 0.5, fieldDrawWidth * 0.5, 0);
	glTexCoord2f(0, 0);
	glVertex3d(1 * fieldDrawLength * 0.5, fieldDrawWidth * 0.5, 0);
	glTexCoord2f(0, 1);
	glVertex3d(1 * fieldDrawLength * 0.5, -1 * fieldDrawWidth * 0.5, 0);
	glTexCoord2f(1, 1);
	glVertex3d(-1 * fieldDrawLength * 0.5, -1 * fieldDrawWidth * 0.5, 0);
	glEnd();
	//I add this to avoid color destortion as suggested on piazza
	glBindTexture(GL_TEXTURE_2D, 0);
	glPopMatrix();
}

//Here we draw the virtual sphere
void displayVirtualSphere()
{
	glColor4f(1, 0, 0, 1);
	glPushMatrix();
	//The coordinate of the sphere is (0,0,50)
	glTranslatef(0, 0, 50);
	glutWireSphere(10, 10, 10);
	glPopMatrix();
}

//drawUAVs is the function draw the UAV using OpenGL 
void drawUAVs()
{	
	//Since the value of RGB is changing I first normalize the R G B value into 0 to 1
	double colorDrawRed = colorR / 255;
	double colorDrawGreen = colorG / 255;
	double colorDrawBlue = colorB / 255;
	//We set the color of the uav based on the cuurent value of RGB
	glColor4f(colorDrawRed, colorDrawGreen,colorDrawBlue, 1);
	//The main thread draw 15 UAVs in this loop
	//The coordinates of uav is stored in the rcvbuffer
	//the i th coordinate of uav is starting from i*6 to i*6+5
	for (int i = 1; i <= 15; i++) 
	{
		glPushMatrix();
		//We should draw the uavs at the location it is every timestamp
		glTranslatef(rcvbuffer[i*6], rcvbuffer[i*6+1], rcvbuffer[i*6+2]);
		//Here we scale the Tetrahedron to make it can be put into the 1m cube
		//glScalef(sqrt(6)/3, sqrt(6)/3, sqrt(6)/3);
		glScalef(0.5, 0.5, 0.5);
		glTranslatef(0, 0, 0);
		glutSolidTetrahedron();
		glPopMatrix();
	}
	//The color is changing from 128 to 255
	//When color reaches 128, in next time stamp the color should increase
	//When color reaches 255, in next time stamp the color should decrease
	if (colorB == 255)
		colorMark = -1;
	else if (colorB == 128)
		colorMark = 1;
	if (colorMark == -1)
		colorB--;
	else if (colorMark == 1)
		colorB++;
}

// Reshape callback
// Window size has been set/changed to w by h pixels. Set the camera
// perspective to 45 degree vertical field of view, a window aspect
// ratio of w/h, a near clipping plane at depth 1, and a far clipping
// plane at depth 100. The viewport is the entire window.
void changeSize(int w, int h)
{
	// window aspect ratio
	float ratio = ((float)w) / ((float)h); 
	// projection matrix is active
	glMatrixMode(GL_PROJECTION); 
	// reset the projection
	glLoadIdentity(); 
	// perspective transformation
	gluPerspective(60.0, ratio, 0.1, 1000.0); 
	// return to modelview mode
	glMatrixMode(GL_MODELVIEW); 
	// set viewport (drawing area) to entire window
	glViewport(0, 0, w, h); 
}
// Draw the entire scene
// We first update the camera location based on its distance from the
// origin and its direction.
void renderScene()
{
	// Clear color and depth buffers
	// background color to green??
	glClearColor(1, 1,1, 1.0); 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// Reset transformations
	glLoadIdentity();
	//Here we set the eye's position
	gluLookAt(eye_x, eye_y, eye_z, center_x, center_y, center_z, 0.0, 0.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	//Here we draw the field
	displayFootballField();
	//Here we draw the virtual sphere
	displayVirtualSphere();
	//Here we draw the UAVs.
	drawUAVs();	
	// Make it all visible
	glutSwapBuffers(); 
	//Here we gather each uav's coordination and speed
	MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
}
// Here we set the drawing function execution time
void timer(int id)
{
	glutPostRedisplay();
	glutTimerFunc(100, timer, 0);
}
// mainOpenGL  - standard GLUT initializations and callbacks
void mainOpenGL(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	//I set the window size and location here
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(400, 400);
	//The window's initial parameter is settled here
	glutCreateWindow(argv[0]);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0, 0.0, 0.0, 0.0);
	//The model's smooth and lightning is settled here
	glShadeModel(GL_SMOOTH);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE);
	glDepthFunc(GL_LESS);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//Here we set the bit map
	inBitmap.read("ff.bmp");
	//The map pixel is initiate here
	makeCheckImage();
	//Here we set pixel stored mode
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	// Create Textures
	glGenTextures(2, texture);
	// Setup first texture
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
	glTexImage2D(GL_TEXTURE_2D, 0, 3, inBitmap.bmp_info_header.width, inBitmap.bmp_info_header.height, 0,
		GL_BGR_EXT, GL_UNSIGNED_BYTE, &inBitmap.data[0]);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	// Do the second texture mapping
	glBindTexture(GL_TEXTURE_2D, texture[1]);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, checkImageWidth, checkImageHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, &checkImage[0][0][0]);
	//We enable the texture parameter here
	glEnable(GL_TEXTURE_2D);
	// Setup lights as needed
	glutReshapeFunc(changeSize);
	glutDisplayFunc(renderScene);
	//Here we use the time function to deraw the scene every 100m second
	glutTimerFunc(100, timerFunction, 0);
	glutMainLoop();
} 

// timerFunction  - called whenever the timer fires
void timerFunction(int id)
{
	glutPostRedisplay();
	glutTimerFunc(100, timerFunction, 0);
}
// calDis calculate the distance of uav to center of the virtual sphere
// the input parameter is rank index and it will return the value of the
// distance from the uav to the sphere center(0,0,50) 
double calDis(int rank)
{
	double res = sqrt((coordX - 0) * (coordX - 0) + (coordY - 0) * (coordY - 0) +
		(coordZ - 50) * (coordZ - 50));
	return res;
}
//In this simulation the uav will go through 6 phase
//Before the uav's movement calculation, I first calulate
//the status of uavs
void calPhase(int rank)
{
	//tempDis is the distance that the uav to the center
	double tempDis = calDis(rank);
	//nirmal vector is the vector that points to the center of sphere
	double* normalVector = calNormal(rank);
	//tempNowVelocity is the magnitude of uav's velocity
	double tempNowVelocity = sqrt(velocityX*velocityX +velocityY*velocityY 
		+ velocityZ*velocityZ);
	//normalForceX,normalForceY and normalForceZ is the normal force's projection on
	//x,y and z axis. The normal force includes: Hook law force and centrifigal force
	double normalForceX;
	double normalForceY;
	double normalForceZ;
	//In phase 0 the uav is accelerate to the center (0,0,50)
	if (uavPhase == 0)
	{
		if(tempNowVelocity > 1.8)
			uavPhase = 1;
	}
	//in pahse 1 the uav is flying to (0,0,50) at navigation speed 2m/s
	else if (uavPhase == 1)
	{
		//If the distance between the uav to the suface of sphere is only 1m
		//then the uav should slow down and land on the surface
		if ((tempDis - 1) < 10)
			uavPhase = 2;
	}
	//In pahse 2 the uav  will slow and land on the surface
	else if (uavPhase == 2)
	{
		if(tempNowVelocity < 0.1)
			uavPhase = 3;
	}
	//in phase 3 the uav will get a instant random foce to make it have a initial speed 
	else if (uavPhase == 3)
	{
		//uav will be given an random tan forces as long as it land on the surface
	}
	//in phase4 the tan force will be added to uav so the tan velocity will incerease to 3m/s
	else if (uavPhase == 4)
	{
		if (tempNowVelocity > 3)
		{
			uavPhase = 5;
		}
	}
	//In pase5 the uav will rotate around the sphere with 3m/s 
	else if (uavPhase == 5)
	{
		//To make sure the velocity of uav is approximately to 3m/s
		//Here when the uav's speed is deviate form 3m/s we will incerease/decrease the velocity
		if (tempNowVelocity > 3.2)
		{
			uavPhase = 6;
		}
		if (tempNowVelocity < 2.8)
		{
			uavPhase = 6;
		}
	}
	//In phase 6 after the uavs' velocity back into the normal range it will back into pahse 5
	else if (uavPhase == 6)
	{
		if (tempNowVelocity < 3.2)
		{
			uavPhase = 5;
		}
		if (tempNowVelocity > 2.8)
		{
			uavPhase = 5;
		}
	}
}
//calForce will determine the force that the uav have based on the phase
void calForce(int rank)
{
	//In phase1 uav will fly towards the center of the center of sphere
	double tempDis = calDis(rank);
	double* normalVector = calNormal(rank);
	double tempNowVelocity = sqrt(velocityX * velocityX + 
		velocityY * velocityY + velocityZ * velocityZ);
	//tempSpringForce consists 2 part, 1 is the hook law force and the 
	//other is 0.9N centrifugal force wich is based the stead navigation speed is 3m/s
	double tempSpringForce = k * (10 - tempDis) + 0.9;
	//Here is the normal force's projection on 3 dimension
	double normalForceX;
	double normalForceY;
	double normalForceZ;
	//tanForce is used to increase the uav's tan speed at the phase3 , phase4 and phase6
	//tanForceX,tanForceY and tanForceZ is the projection of tanForce
	double tanForceX;
	double tanForceY;
	double tanForceZ;
	if (uavPhase == 0)
	{
		//We set a force 2N points to the (0,0,50) and a 10N force is also provided on
		//Z direction to offset the gravity
		forceX = normalVector[0] * 2 ;
		forceY = normalVector[1] * 2 ;
		forceZ = normalVector[2] * 2 + 10;
		acceX = forceX / 1 ;
		acceY = forceY / 1 ;
		//When calulate y direction's acceleration, the gravity 10N should always be
		//taken into consideration
		acceZ = (forceZ - 10) / 1;
	}
	else if (uavPhase == 1)
	{
		//the uav is naviagte at 2m/s towards the (0,0,50) so only 10N on Z axis should be apply
		forceX = 0;
		forceY = 0;
		forceZ = 10;
		acceX = 0;
		acceY = 0;
		acceZ = (0 + 10 - 10)/1;
	}
	else if (uavPhase == 2)
	{
		//We should add an normal force to slow down the uav and make it land on the surface
		double normalTempForceX = normalVector[0] * 2 * -1 ;
		double normalTempForceY = normalVector[1] * 2 * -1 ;
		double normalTempForceZ = normalVector[2] * 2 * -1 ;
		forceX = normalTempForceX;
		forceY = normalTempForceY;
		forceZ = normalTempForceZ + 10;
		acceX = forceX / 1;
		acceY = forceY / 1;
		acceZ = (forceZ - 10) / 1;
	}
	else if (uavPhase == 3)
	{
		//Using srand function to generate a random number
		srand((unsigned int)time(NULL));
		//the x and y tan vector is randomly choosen so the path is random
		double tempTanVectorX = ((double)rand() / RAND_MAX) * 2 + -1;
		double tempTanVectorY = ((double)rand() / RAND_MAX) * 2 + -1;
		//since orthogonality vector dot product is 0 we could calculate the z axis vector
		double tempTanVectorZ = -1 * (tempTanVectorX * normalVector[0] +
			tempTanVectorY * normalVector[1]) / normalVector[2];
		//Calculating the normal orthogonality vector
		double tanVectorX = tempTanVectorX / (sqrt(tempTanVectorX * tempTanVectorX +
			tempTanVectorY * tempTanVectorY + tempTanVectorZ * tempTanVectorZ));
		double tanVectorY = tempTanVectorY / (sqrt(tempTanVectorX * tempTanVectorX +
			tempTanVectorY * tempTanVectorY + tempTanVectorZ * tempTanVectorZ));
		double tanVectorZ = tempTanVectorZ / (sqrt(tempTanVectorX * tempTanVectorX +
			tempTanVectorY * tempTanVectorY + tempTanVectorZ * tempTanVectorZ));
		//We calculate centrifugal force which is neccessary to ensure the uav rotate
		double normalDeviationForce = (tempNowVelocity * tempNowVelocity) / tempDis;
		//The total force is the addition of Hook force,orthogonality force and centrifugal force
		if (abs(normalDeviationForce + k * (10 - tempDis)) < 9) 
		{
			forceX = 1 * tanVectorX + (normalDeviationForce + k * (10 - tempDis)) * normalVector[0];
			forceY = 1 * tanVectorY + (normalDeviationForce + k * (10 - tempDis)) * normalVector[1];
			forceZ = 1 * tanVectorZ + (normalDeviationForce + k * (10 - tempDis)) * normalVector[2] + 10;
		}
		else
		{
			forceX = 1 * tanVectorX + 9 * normalVector[0];
			forceY = 1 * tanVectorY + 9 * normalVector[1];
			forceZ = 1 * tanVectorZ + 9 * normalVector[2] + 10;
		}
		acceX = forceX / 1;
		acceY = forceY / 1;
		acceZ = (forceZ - 10) / 1;
		uavPhase = 4;
	}
	else if (uavPhase == 4)
	{
		//We add an orthogonality force to the uav so it can accelerate to the 3m/s which is the rotating speed
		double vectorVelocityX = velocityX / (sqrt(velocityX * velocityX +
			velocityY * velocityY + velocityZ * velocityZ));
		double vectorVelocityY = velocityY / (sqrt(velocityX * velocityX +
			velocityY * velocityY + velocityZ * velocityZ));
		double vectorVelocityZ = velocityZ / (sqrt(velocityX * velocityX +
			velocityY * velocityY + velocityZ * velocityZ));
		double normalDeviationForce = (tempNowVelocity * tempNowVelocity) / tempDis;
		if (abs(normalDeviationForce + k * (10 - tempDis)) < 9)
		{
			forceX = 1 * vectorVelocityX + (normalDeviationForce + k * (10 - tempDis)) * normalVector[0];
			forceY = 1 * vectorVelocityY + (normalDeviationForce + k * (10 - tempDis)) * normalVector[1];
			forceZ = 1 * vectorVelocityZ + (normalDeviationForce + k * (10 - tempDis)) * normalVector[2] + 10;
		}
		else
		{
			forceX = 1 * vectorVelocityX + 9 * normalVector[0];
			forceY = 1 * vectorVelocityY + 9 * normalVector[1];
			forceZ = 1 * vectorVelocityZ + 9 * normalVector[2] + 10;
		}
		acceX = forceX / 1;
		acceY = forceY / 1;
		acceZ = (forceZ - 10) / 1;
	}
	else if (uavPhase == 5)
	{
		//During navigation stage we should consider the situation that the total force maybe too large to exceed 20N
		//However since the speed is 3m/s and the k is only 0.1 it is nearly impossible for the
		//total force to exceed 20N, but we still do the calculation here to make sure
		if (tempSpringForce < 9 && tempSpringForce > -9)
		{
			//Here I set the centrifugal force to be 0.9N since I balance the velcoity to
			//be around 3m/s. The max Hook's force is 9N. So the total force is less than 20N. 
			forceX = normalVector[0] * (k * (10 - tempDis) + 0.9);
			forceY = normalVector[1] * (k * (10 - tempDis) + 0.9);
			forceZ = normalVector[2] * (k * (10 - tempDis) + 0.9) + 10;
		}
		else
		{
			if (tempSpringForce > 0)
			{
				forceX = normalVector[0] * (9 * 1 + 0.9);
				forceY = normalVector[1] * (9 * 1 + 0.9);
				forceZ = normalVector[2] * (9 * 1 + 0.9) + 10;
			}
			else
			{
				forceX = normalVector[0] * (9 * -1 + 0.9);
				forceY = normalVector[1] * (9 * -1 + 0.9);
				forceZ = normalVector[2] * (9 * -1 + 0.9) + 10;
			}
		}
		acceX = forceX / 1;
		acceY = forceY / 1;
		acceZ = (forceZ - 10) / 1;
	}
	else if (uavPhase == 6)
	{
		//We set dir to mark whether this uav should accelerate to 3m/s or slow down to 3m/s
		double dir;
		if (tempNowVelocity > 3.2)
			dir = -1;
		else if (tempNowVelocity < 2.8)
			dir = 1;
		else
			dir = 0;
		//We add a force on the direction of uav's velocity
		double vectorVelocityX = velocityX / (sqrt(velocityX * velocityX + 
			velocityY * velocityY +velocityZ * velocityZ));
		double vectorVelocityY = velocityY / (sqrt(velocityX * velocityX +
			velocityY * velocityY + velocityZ * velocityZ));
		double vectorVelocityZ = velocityZ / (sqrt(velocityX * velocityX +
			velocityY * velocityY + velocityZ * velocityZ));
		//During the process of balancing uav to the normal speed range we do not add Hook's law
		//we just add a 0.2N tan direction force and 0.9N centrifugal force 
		forceX = vectorVelocityX * 0.2 * dir + normalVector[0]*0.9;
		forceY = vectorVelocityY * 0.2 * dir + normalVector[1]*0.9;
		forceZ = vectorVelocityZ * 0.2 * dir + normalVector[2]*0.9+ 10;
		acceX = forceX / 1;
		acceY = forceY / 1;
		acceZ = (forceZ - 10) / 1;
	}
}
//Here we use physics law to calculate the coordination of uav based on the force
//and uav's previous status. Since each iteration is 0.1s we add it in equation
void CalcualteUAVsLocation(int rank)
{
	coordX = coordX + velocityX * 0.1 + 0.5 * acceX * 0.01 ;
	coordY = coordY + velocityY * 0.1 + 0.5 * acceY * 0.01;
	coordZ = coordZ + velocityZ * 0.1 + 0.5 * acceZ * 0.01 ;
	//Updating the sendBuffer used in Algather
	sendBuffer[0] = coordX;
	sendBuffer[1] = coordY;
	sendBuffer[2] = coordZ;
}
//Here we use Newton's law to update the velocity 
void calUAVsVelocity(int rank)
{	
	velocityX = velocityX + acceX * 0.1 ;
	velocityY = velocityY + acceY * 0.1 ;
	velocityZ = velocityZ + acceZ * 0.1 ;
	//Updating the sendBuffer used in Algather
	sendBuffer[3] = velocityX;
	sendBuffer[4] = velocityY;
	sendBuffer[5] = velocityZ;
}

// Main entry point determines rank of the process and follows the correct program path
int main(int argc, char** argv)
{
	//We initiate the MPI communication process
	int numTasks, rank;
	int rc = MPI_Init(&argc, &argv);
	if (rc != MPI_SUCCESS)
	{
		printf("Error starting MPI program. Terminating.\n");
		MPI_Abort(MPI_COMM_WORLD, rc);
	}
	MPI_Comm_size(MPI_COMM_WORLD, &numTasks);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	int gsize = 0;
	MPI_Comm_size(MPI_COMM_WORLD, &gsize);
	//Here we initial the x,y,z parameter of UAVs
	if (rank == 0)
	{
		//Coordination,velocity,force and acceleration in rank1 should be 0
		coordX = 0;
		coordY = 0;
		coordZ = 0;
		velocityX = 0;
		velocityY = 0;
		velocityZ = 0;
		forceX = 0;
		forceY = 0;
		forceZ = 0;
		acceX = 0;
		acceY = 0;
		acceZ = 0;
	}
	else 
	{
		//rank 0 to rank 4 has positive y
		if (rank <= 5 && rank>0)
		{
			coordX = (rank - 3) * gapX;
			coordY = gapY;
			coordZ = 0;
			velocityX = 0;
			velocityY = 0;
			velocityZ = 0;
			forceX = 0;
			forceY = 0;
			forceZ = 0;
			acceX = 0;
			acceY = 0;
			acceZ = 0;
		}
		//rank 5 to rank 9 has 0 y
		else if (rank > 5 && rank <= 10)
		{
			coordX = (rank - 8) * gapX;
			coordY = 0;
			coordZ = 0;
			velocityX = 0;
			velocityY = 0;
			velocityZ = 0;
			forceX = 0;
			forceY = 0;
			forceZ = 0;
			acceX = 0;
			acceY = 0;
			acceZ = 0;
		}
		// rank 11 to rank 15 has negative y
		else if (rank > 10)
		{
			coordX = (rank - 13) * gapX;
			coordY = -1 * gapY;
			coordZ = 0;
			velocityX = 0;
			velocityY = 0;
			velocityZ = 0;
			forceX = 0;
			forceY = 0;
			forceZ = 0;
			acceX = 0;
			acceY = 0;
			acceZ = 0;
		}
	}
	//The sendBuffer and rcvbuffer is initiated here
	sendBuffer[0] = coordX;
	sendBuffer[1] = coordY;
	sendBuffer[2] = coordZ;
	sendBuffer[3] = velocityX;
	sendBuffer[4] = velocityY;
	sendBuffer[5] = velocityZ;
	for (int i = 0; i < rcvSize; i++)
	{
		rcvbuffer[i] = 0;
	}
	//Here we first gather the initial position of UAVs whihc is used to first render the scene
	MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
	//Here the simulation begins
	if (rank == 0)
	{
		//main thread is charging rendering the scene
		mainOpenGL(argc, argv);
	}
	else
	{
		// Sleep for 5 seconds
		std::this_thread::sleep_for(std::chrono::seconds(5));
		for (int ii = 0; ii < 820; ii++)
		{
			//double tempNowVelocity = sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ);
			//cout << tempNowVelocity << endl;
			//We first deternmine the phase of the UAVs
			calPhase(rank);
			//According to phase we then calculate the force
			calForce(rank);
			//Then we calculate the coordiantion
			CalcualteUAVsLocation(rank);
			//Then we calculate the velocity
			calUAVsVelocity(rank);
			//Here we have to test whether two UAVs will get collision
			MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE,
				rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
			for (int i = 1; i < 16; i++)
			{
				//a uvs will not collide with itself
				if (i == rank)
					continue;
				//if two uav's distance is smaller than 1.01 m(whihc means the cube's face distance is less than 1cm)
				if (sqrt(pow(coordX - rcvbuffer[i * 6], 2) +
					pow(coordY - rcvbuffer[i * 6 + 1], 2)
					+ pow(coordZ - rcvbuffer[i * 6 + 2], 2)) 
					< pow(1.01, 2))
				{
					//Changing the velocity
					velocityX = rcvbuffer[i * 6 + 3];
					velocityY = rcvbuffer[i * 6 + 4];
					velocityZ = rcvbuffer[i * 6 + 5];
					//If the uav which is still not landing but hit a uav it will change to phase4 to rotate the sphere 
					if (uavPhase == 1 || uavPhase == 2)
					{
						//cout << "HHHHH" << endl;
						uavPhase = 4;
					}
					
				}
			}
		}
	}
	//return 0;
	//MPI_Barrier(MPI_COMM_WORLD);
	//MPI_Finalize();
	return 0;
}
