/* Flight Simulator program for CVG
   Joshua Tyler 6213642 */

#include <Windows.h>
#include <gl/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include "mesh.h"
#include "imageloader.h"

typedef struct
{
	GLfloat x;
	GLfloat y;
	GLfloat z;
} vector3d;


void initGl(void);
void display(void);
void mouse(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);
void readInput(char* filename);
void adjForce(unsigned int dir);
void setPosition(int x);
void keyDown(unsigned char key, int x, int y);
void keyUp(unsigned char key, int x, int y);
void timer(int x);
GLfloat vectorMag(vector3d vector);
vector3d vectorConstMult(vector3d vector, GLfloat constant);
vector3d vectorNorm(vector3d vector);
vector3d vectorAdd(vector3d vector1, vector3d vector2);
GLfloat procDirIn(GLfloat direction, unsigned char keyPlus, unsigned char keyMinus, GLfloat max, GLfloat inc, unsigned int retToZero);
void ringCollDetect(vector3d centre, int ringID);
vector3d vectorConvert(Vector3f vector);
void drawAxis(void);
void renderText(char *string, GLfloat x, GLfloat y);
void calcFps(void);
void idle(void);
vector3d rotateAboutY(vector3d position, GLfloat angle);
void passiveMouse(int x, int y);
void reshape(int width, int height);

int mRows, mCols;
int **map; /* Pointer to the level map array */

int keystate[256] = {0}; // Store if a key is pressed or not

/* Window size */

int windowWidth = 1280;
int windowHeight = 720;

GLfloat mouseX, mouseY;

int mouseLatch = FALSE;

/* Mathematical constants */
#define RADS_TO_DEGS (180/3.141592654)
#define DEGS_TO_RADS (3.141592654/180)


/* Scaling factors to convert array to real space */
#define X_SCLR 100
#define Y_SCLR 10
#define Z_SCLR 10

/* Constants to define how far wall is from real edge */
#define X_MARGIN 2
#define Y_MARGIN 1
#define Z_MARGIN 10

vector3d pos = {0,30,0}, ang;

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define ANG_INC 2
#define POS_INC 0.05

/* Light position */
static GLfloat light0_position[] = {1.0,1.0,1.0,0.0};


#define TORUS_DETAIL 100
#define TORUS_OUTER	3.0
#define TORUS_INNER	0.3

/* Velocity */
GLfloat force =0;

vector3d direction = {0,0,0}, velocity= {0,0,0}, normalisedDir = {1,0,0};

GLfloat yAng = 0;

#define UP 1
#define DOWN 0

const GLfloat maxForce = 10000;
const GLfloat forceIncrement = 100;
const GLfloat airResistanceCoefficient = 0.5;
const unsigned int delay = 10;

/* Plane mesh and texture stuff */

#define PLANE_MESH_FILENAME "raptor.obj"
#define PLANE_TEXTURE_FILENAME "raptor.bmp"
Mesh planeMesh;
vector3d planeCentre, planeMax, planeMin;

Image *planeTex;

#define SKY_TEXTURE_FILENAME "sky.bmp"
Image *skyTex;

#define GROUND_TEXTURE_NAME "ground.bmp"
Image *groundTex;

/* Other stuff */

int score = 0;
int lives = 3;
float fps;
int pause = 0;



int main(int argc, char **argv)
{
	/* Read input map */
	readInput("map.txt");

	/* Initialise the GLUT window manager */
	glutInit(&argc, argv);       
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(windowWidth, windowHeight);
	glutCreateWindow("Flight Simulator");

	/* Register callback functions */
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
//	glutKeyboardFunc(keyboard);
	glutIgnoreKeyRepeat(GL_TRUE);
	glutKeyboardFunc(keyDown);
	glutKeyboardUpFunc(keyUp);
	glutTimerFunc(delay,timer,0);
	glutIdleFunc(idle);
	glutPassiveMotionFunc(passiveMouse);
	glutReshapeFunc(reshape);

	/* Load mesh and texture for plane */
	loadMesh(planeMesh, PLANE_MESH_FILENAME);

	planeTex = loadBMP(PLANE_TEXTURE_FILENAME);

	skyTex = loadBMP(SKY_TEXTURE_FILENAME);

	groundTex = loadBMP(GROUND_TEXTURE_NAME);

	/* Find centre, max and min co-ordinates */

	planeCentre = vectorConvert(getCentroid(planeMesh));
	planeMax = vectorConvert(getMax(planeMesh));
	planeMin = vectorConvert(getMin(planeMesh));

	/* Initialise OpenGL*/
	initGl();

	/* Loop forever and ever (but still call callback functions...) */
	glutMainLoop();

	return(EXIT_SUCCESS);
}

void initGl(void)
{

	glMatrixMode(GL_PROJECTION); /* GL_PROJECTION is used for setting up viewing properties e.g lens angle etc. */
	glLoadIdentity(); /* Initialise to identity matirix */
	reshape(windowWidth, windowHeight); /* Set up field of view */
//	gluPerspective(45.0, (GLdouble)windowWidth/(GLdouble)windowHeight, 1.0, 400.0); /* Set up the field of view as perspective */
	glClearColor (0.529, 0.808, 0.980, 1.0); /* Set background colour to blue */

	glLineWidth(5.0);

	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_COLOR_MATERIAL);
	//glEnable(GL_CULL_FACE);
    //glFrontFace(GL_CW);

	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0_position);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0_position);


	/* Set up texture mapping */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
}

/* This callback occurs whenever the system determines the window needs redrawing (or upon a call of glutPostRedisplay()) */
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); /*Clear color and depth buffers*/

	glMatrixMode(GL_MODELVIEW); /* GL_MODELVIEW is used to set up the model and translate into camera space */
	glLoadIdentity(); /* Initialise to identity matirix */

	gluLookAt(-5,0.5,0, 1,0,0, 0,1,0); // Normal ("behind") camera view
//	gluLookAt(-5,0,0, 0,0,0, 0,1,0); // Debug - behind view
//	gluLookAt(0,100,0, 0,0,0, 1,0,0); // Debug - above view
	glRotatef(-yAng,0.0,1.0,0.0);

	glRotatef(-45.0+90*mouseX,0.0,1.0,0.0);
	glRotatef(-45.0+90*mouseY,0.0,0.0,1.0);


	glTranslatef(-pos.x,-pos.y,-pos.z); /* Translate to viewpoint */

	/* Draw the plane */
	glPushMatrix();
	glTranslatef(pos.x, pos.y, pos.z);
	drawAxis();
	glRotatef(yAng,0.0,1.0,0.0);

	glColor3f(1.0,0.0,0.0); /* Draw in Red */

	glRotatef(-90.0,1.0,0.0,0.0); /* Rotate around X axis */
	glRotatef(-90,0.0,0.0,1.0); /* Rotate around Z axis */

	glRotatef(RADS_TO_DEGS*sin(normalisedDir.z),0.0,1.0,0.0); /* L/R rotation */
	glRotatef(RADS_TO_DEGS*sin(normalisedDir.y),1.0,0.0,0.0); /* U/D rotation */


	glEnable(GL_TEXTURE_2D);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, planeTex->width, planeTex->height, 0, GL_RGB, GL_UNSIGNED_BYTE, planeTex->pixels);
	drawMesh(planeMesh);
	glDisable(GL_TEXTURE_2D);

	glPopMatrix();



	/* Draw the rings */
	GLfloat midpoint = (mCols-1)/(GLfloat)2;
	vector3d ringPos;
	int i,j, ringID;
	ringID = 0;
	for(i=0; i < mRows; i++)
		for(j=0; j < mCols; j++)
			if(map[i][j] != 0)
			{
				ringPos.x = (GLfloat)(X_SCLR*(i+1));
				ringPos.y = (GLfloat)(Y_SCLR*map[i][j])/(GLfloat)3.0;
				ringPos.z = Z_SCLR*(j - midpoint);
				ringCollDetect(ringPos, ringID);
				glPushMatrix();
				glTranslatef(ringPos.x,ringPos.y, ringPos.z); /* Distance then height then left/right */
				glRotatef(90.0,0.0,1.0,0.0);
				glutSolidTorus(TORUS_INNER,TORUS_OUTER,TORUS_DETAIL,TORUS_DETAIL);
				glPopMatrix();
				ringID++;
			}

	/* Draw the walls */
	GLfloat xUprBnd = (GLfloat)(X_SCLR* (mRows + X_MARGIN));
	GLfloat xLwrBnd = -X_SCLR*X_MARGIN;
	GLfloat yUprBnd = (GLfloat)(Y_SCLR + Y_SCLR*(5 + Y_MARGIN) ); /* WARNING - NEEDS FILLING WITH REAL VALUE!! */
	GLfloat yLwrBnd = -(TORUS_OUTER + Y_MARGIN*Y_SCLR);
	GLfloat zUprBnd = Z_SCLR*(Z_MARGIN + midpoint) + TORUS_OUTER;
	GLfloat zLwrBnd = -zUprBnd;

	glColor3f(0.0,1.0,0.0); /* Draw walls in green */

	glBegin(GL_POLYGON); /* Back wall */
		glVertex3f (xUprBnd, yLwrBnd, zLwrBnd);
		glVertex3f (xUprBnd, yLwrBnd, zUprBnd);
		glVertex3f (xUprBnd, yUprBnd, zUprBnd);
		glVertex3f (xUprBnd, yUprBnd, zLwrBnd);
	glEnd();

//	glBegin(GL_POLYGON); /* Front wall */
//		glVertex3f (xLwrBnd, yLwrBnd, zLwrBnd);
//		glVertex3f (xLwrBnd, yLwrBnd, zUprBnd);
//		glVertex3f (xLwrBnd, yUprBnd, zUprBnd);
//		glVertex3f (xLwrBnd, yUprBnd, zLwrBnd);
//	glEnd();

	glEnable(GL_TEXTURE_2D);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, groundTex->width-1, groundTex->height-1, 0, GL_RGB, GL_UNSIGNED_BYTE, groundTex->pixels);

	glBegin(GL_POLYGON); /* Floor */
		glTexCoord2f(0.0, 0.0);
		glVertex3f (xLwrBnd, yLwrBnd, zLwrBnd); /* Distance, then Height, then L/R */
		glTexCoord2f(0.0, 5.0);
		glVertex3f (xLwrBnd, yLwrBnd, zUprBnd);
		glTexCoord2f(10.0, 0.0);
		glVertex3f (xUprBnd, yLwrBnd, zUprBnd);
		glTexCoord2f(10.0, 5.0);
		glVertex3f (xUprBnd, yLwrBnd, zLwrBnd);
	glEnd();

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, skyTex->width-1, skyTex->height-1, 0, GL_RGB, GL_UNSIGNED_BYTE, skyTex->pixels);
	glBegin(GL_POLYGON); /* Left wall */
		glTexCoord2f(0.0, 0.0);
		glVertex3f (xLwrBnd, yLwrBnd, zLwrBnd);
		glTexCoord2f(0.0, 1.0);
		glVertex3f (xLwrBnd, yUprBnd, zLwrBnd);
		glTexCoord2f(1.0, 0.0);
		glVertex3f (xUprBnd, yUprBnd, zLwrBnd);
		glTexCoord2f(1.0, 1.0);
		glVertex3f (xUprBnd, yLwrBnd, zLwrBnd);
	glEnd();

	glBegin(GL_POLYGON); /* Right wall */
		glTexCoord2f(0.0, 0.0);
		glVertex3f (xLwrBnd, yLwrBnd, zUprBnd);
		glTexCoord2f(0.0, 1.0);
		glVertex3f (xLwrBnd, yUprBnd, zUprBnd);
		glTexCoord2f(1.0, 0.0);
		glVertex3f (xUprBnd, yUprBnd, zUprBnd);
		glTexCoord2f(1.0, 1.0);
		glVertex3f (xUprBnd, yLwrBnd, zUprBnd);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_TEXTURE_2D);

	/* Render score */
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();
	glDisable( GL_DEPTH_TEST );

	char stringToPrint[100];
	sprintf(stringToPrint,"Score: %d Lives: %d FPS: %f", score, lives, fps);
	renderText(stringToPrint,-1,0.9);
	glPopMatrix();

	glEnable( GL_DEPTH_TEST );
	glMatrixMode( GL_PROJECTION ) ;
	glPopMatrix() ;
	glMatrixMode( GL_MODELVIEW ) ;
	glPopMatrix() ;
	
	glFlush(); /* Execute all isssued commands */

}

/* This callback occurs upon button press */
void mouse(int button, int state, int x, int y)
{
	printf("MOUSE! %d\n", button);
	if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
		mouseLatch = !mouseLatch;

}


void readInput(char* filename)
{

	/* Open file */
	FILE *filePtr;
	filePtr = fopen(filename,"r");
	if(filePtr == NULL)
	{
		fputs("Error, could not open input file.\n", stderr);
		exit(EXIT_FAILURE);
	}

	/* Determine map size */

	if (fscanf(filePtr,"%d%d", &mRows, &mCols) < 2)
	{
		fputs("Error, could not read input file.\n", stderr);
		exit(EXIT_FAILURE);
	}

	/* Allocate map */
	int i;
	map = (int **)malloc(mRows * sizeof(int *));
	for(i = 0; i < mRows; i++)
		map[i] = (int *)malloc(mCols * sizeof(int));

	/* Read input to map */
	int j, temp;
	for(i=0; i < mRows; i++)
		for(j=0; j < mCols; j++)
		{
			if (fscanf(filePtr,"%d", &map[i][j]) != 1)
			{
				fputs("Error, could not read input file.\n", stderr);
				exit(EXIT_FAILURE);
			}
		}

	return;
}


void adjForce(unsigned int dir)
{
	if(dir == UP)
	{
		if(force < maxForce)
			force += forceIncrement;
	} else {
		if(force > 0)
			force -= forceIncrement;
	}
//	printf("Force: %f\n", force);
}

void setPosition(void)
{
	GLfloat delayInSeconds = (GLfloat)delay/(GLfloat)1000;

	GLfloat acceleration;

	GLfloat velocityMagnitude = vectorMag(velocity);

	if(velocity.x >= 0)
		acceleration= force - airResistanceCoefficient*velocityMagnitude*velocityMagnitude;
	else
		acceleration= force + airResistanceCoefficient*velocityMagnitude*velocityMagnitude;

	velocityMagnitude += delayInSeconds * acceleration; // Find new velocity magnitude

	normalisedDir = vectorNorm(direction);

	vector3d rotatedDir = rotateAboutY(normalisedDir, yAng);

	velocity = vectorConstMult(rotatedDir, velocityMagnitude);

	pos = vectorAdd(pos, vectorConstMult(velocity,delayInSeconds) );


}

void keyDown(unsigned char key, int x, int y)
{
	keystate[key] = TRUE;

//	printf("Key: %c pressed.\n", key);
}

void keyUp(unsigned char key, int x, int y)
{
	keystate[key] = FALSE;

//	printf("Key: %c released.\n", key);
}

void timer(int x)
{

	direction.x = 1;

	if(keystate['p'] == TRUE)
		pause = !pause;

	if(pause)
	{
		glutTimerFunc(delay, timer, 0);
		return;
	}


	if(keystate['o'] == TRUE)
		adjForce(UP);
	
	if(keystate['l'] == TRUE)
		adjForce(DOWN);


	direction.y = procDirIn(direction.y, 'w', 's', 5.0, POS_INC, TRUE);
	direction.z = procDirIn(direction.z, 'd', 'a', 5.0, POS_INC, TRUE);

	yAng = procDirIn(yAng, 'i','k', 45.0, 1.0, TRUE);


//	printf("direction: x:%f, y:%f, z:%f\nvelocity: x:%f, y:%f, z:%f\npos: x:%f, y:%f, z:%f\nyAng: %f\n",direction.x,direction.y,direction.z, velocity.x,velocity.y, velocity.z,pos.x,pos.y,pos.z,yAng);

	setPosition();
	
	glutTimerFunc(delay, timer, 0);

}

GLfloat vectorMag(vector3d vector)
{
	GLfloat magnitude;

	magnitude = sqrt(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);

	return magnitude;
}

vector3d vectorNorm(vector3d vector)
{
	GLfloat magnitude = vectorMag(vector);

	vector.x /= magnitude;
	vector.y /= magnitude;
	vector.z /= magnitude;

	return vector;
}

vector3d vectorConstMult(vector3d vector, GLfloat constant)
{
	vector.x *= constant;
	vector.y *= constant;
	vector.z *= constant;

	return vector;
}

vector3d vectorAdd(vector3d vector1, vector3d vector2)
{
	vector1.x += vector2.x;
	vector1.y += vector2.y;
	vector1.z += vector2.z;

	return vector1;
}


GLfloat procDirIn(GLfloat direction, unsigned char keyPlus, unsigned char keyMinus, GLfloat max, GLfloat inc, unsigned int retToZero)
{
	if(keystate[keyPlus] == keystate [keyMinus]) // If both "up" and "down" keys are held return to zero.
	{
		if(retToZero == FALSE)
			return direction;
		if(abs(direction) < 2*inc)
			direction = 0;
		if(direction > 0)
			direction -= inc;
		if(direction < 0)
			direction += inc;
		return direction;
	}



	if(keystate[keyPlus] == TRUE && direction < max) // Increment if we need to go up
		direction += inc;

	if(keystate[keyMinus] == TRUE && direction > -max) // Decrement if we need to go down
		direction -= inc;

	return direction;
}

void ringCollDetect(vector3d centre, int ringID)
{

	static int lastIn = -1;

	const GLfloat torusTotal = TORUS_OUTER+TORUS_INNER;
	const GLfloat torusGap = TORUS_OUTER-TORUS_INNER;


	/* Check if inside */
	if(lastIn != ringID)
	{
		if(pos.x + planeMax.x > centre.x - TORUS_INNER && pos.x + planeMin.x < centre.x + TORUS_INNER)
		{
			if(pos.z + planeMin.z < centre.z + torusTotal && pos.z + planeMax.z > centre.z - torusTotal )
			{
				if(pos.y + planeMax.y > centre.y - torusTotal && pos.y + planeMin.y < centre.y + torusTotal)
				{
					/* If we're here, we're inside */
					score++;
					lastIn = ringID;
					/* Check if collided */
					printf("%d %d %d %d\n", (pos.z + planeMax.z > centre.z + torusGap), (pos.z + planeMin.z < centre.z - torusGap), (pos.y + planeMin.y < centre.y - torusGap), (pos.y + planeMax.y > centre.y + torusGap));
					if( (pos.z + planeMax.z > centre.z + torusGap) || (pos.z + planeMin.z < centre.z - torusGap) || (pos.y + planeMin.y < centre.y - torusGap) || (pos.y + planeMax.y > centre.y + torusGap) )
					{
						lives--;
						printf("Collided.\n");
					}
				}
			}

		}
	}

}

vector3d vectorConvert(Vector3f vector)
{
	vector3d returnVector;

	returnVector.x = vector.x;
	returnVector.y = vector.y;
	returnVector.z = vector.z;

	return returnVector;
}

void drawAxis(void)
{
		glColor3f(1.0,0.0,0.0); /* x axis in red */
		glBegin(GL_LINES); /* x */
		glVertex3f(0.0,0.0,0.0);
		glVertex3f(1.0,0.0,0.0);
		glEnd();


		glColor3f(1.0,1.0,0.0); /* y axis in white */
		glBegin(GL_LINES); /* y */
		glVertex3f(0.0,0.0,0.0);
		glVertex3f(0.0,1.0,0.0);
		glEnd();

		glColor3f(0.0,0.0,1.0); /* z axis in blue */
		glBegin(GL_LINES); /* z */
		glVertex3f(0.0,0.0,0.0);
		glVertex3f(0.0,0.0,1.0);
		glEnd();
}

void renderText(char *string, GLfloat x, GLfloat y)
{
	glColor3f(0,0,0);
	glRasterPos2f(x,y);

	int stringLength = strlen(string);
	int i;

	for(i=0; i < stringLength; i++)
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, string[i]);

}

void idle(void)
{
	if(!pause)
	{
		calcFps();
		glutPostRedisplay();
	}

}

void calcFps(void)
{
	static int curTime = 0;
	static int prevTime = 0;
	int interval;

	static int frameCount = 0;

	frameCount++;

    //  Get the number of milliseconds since glutInit called
    //  (or first call to glutGet(GLUT ELAPSED TIME)).
    curTime = glutGet(GLUT_ELAPSED_TIME);

	interval = curTime - prevTime;

	if (interval >=1000)
	{
		fps = frameCount*(interval/1000.0);

		prevTime = curTime;

		frameCount = 0;
	}



}

vector3d rotateAboutY(vector3d position, GLfloat angle)
{
	GLfloat cosAng = cos(DEGS_TO_RADS*angle);
	GLfloat sinAng = sin(DEGS_TO_RADS*angle);

	position.x = cosAng*position.x + sinAng*position.z;
	position.z = -sinAng*position.x + cosAng*position.z;

	return position;

}

void passiveMouse(int x, int y)
{
	if (mouseLatch)
		return;

	mouseX = (GLfloat)x/(GLfloat)windowWidth;
	mouseY = (GLfloat)y/(GLfloat)windowHeight;
//	printf("x: %f, y:%f\n", mouseX, mouseY);
}

void reshape(int width, int height)
{
	if(height == 0)
		height = 1;
	windowWidth = width;
	windowHeight = height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glViewport(0,0,windowWidth, windowHeight);

	gluPerspective(45.0, (GLdouble)windowWidth/(GLdouble)windowHeight, 1.0, 1000.0); /* Set up the field of view as perspective */
	glMatrixMode(GL_MODELVIEW);
	//	initGl();
}