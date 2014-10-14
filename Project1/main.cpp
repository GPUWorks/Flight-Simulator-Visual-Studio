/* Flight Simulator program for CVG
   Joshua Tyler 6213642 */

#include <Windows.h>
#include <gl/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include "mesh.h"
#include "imageloader.h"

void initGl(void);
void display(void);
void mouse(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);
void readInput(char* filename);
void adjForce(unsigned int dir);
void setPosition(int x);

int mRows, mCols;
int **map; /* Pointer to the level map array */

typedef struct
{
	GLfloat x;
	GLfloat y;
	GLfloat z;
} vector3d;

/* Scaling factors to convert array to real space */
#define X_SCLR 10
#define Y_SCLR 10
#define Z_SCLR 10

vector3d pos, ang;

#define ANG_INC 2
#define POS_INC 2

/* Light position */
static GLfloat light0_position[] = {1.0,1.0,1.0,0.0};

/* Camera position */
vector3d camera = {-20,0,0};

#define TORUS_DETAIL 100
#define TORUS_OUTER	3.0
#define TORUS_INNER	0.3

/* Velocity */
GLfloat force, velocity;

#define UP 1
#define DOWN 0

const GLfloat maxForce = 400;
const GLfloat forceIncrement = 20;
const GLfloat airResistanceCoefficient = 0.5;
const unsigned int delay = 10;

/* Plane mesh and texture stuff */

#define PLANE_MESH_FILENAME "raptor.obj"
#define PLANE_TEXTURE_FILENAME "raptor.bmp"
Mesh planeMesh;
vector3d planeCentre;

vector3d planeOffset = {3.5,-0.5,0};

Image *planeTex;


int main(int argc, char **argv)
{
	/* Read input map */
	readInput("map.txt");

	pos.x = pos.y = pos.z = ang.x = ang.y = ang.z = 0;

	force = velocity = 0;

	/* Initialise the GLUT window manager */
	glutInit(&argc, argv);       
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(1280, 720);
	glutCreateWindow("Flight Simulator");

	/* Register callback functions */
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(delay,setPosition,0);

	/* Load mesh and texture for plane */
	loadMesh(planeMesh, PLANE_MESH_FILENAME);

	planeTex = loadBMP(PLANE_TEXTURE_FILENAME);

	/* Find centre co-ordinates */
	Vector3f tempPlaneCentre;
	tempPlaneCentre = getCentroid(planeMesh);
	planeCentre.x = tempPlaneCentre.x;
	planeCentre.y = tempPlaneCentre.y;
	planeCentre.z = tempPlaneCentre.z;

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
	gluPerspective(45.0, (GLdouble)16/(GLdouble)9, 1.0, 400.0); /* Set up the field of view as perspective */
	glClearColor (0.529, 0.808, 0.980, 1.0); /* Set background colour to blue */

	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_COLOR_MATERIAL);
	//glEnable(GL_CULL_FACE);
    //glFrontFace(GL_CW);

	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);

	/* Set up texture mapping */
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, planeTex->width, planeTex->height, 0, GL_RGB, GL_UNSIGNED_BYTE, planeTex->pixels);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}

/* This callback occurs whenever the system determines the window needs redrawing (or upon a call of glutPostRedisplay()) */
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); /*Clear color and depth buffers*/

	glMatrixMode(GL_MODELVIEW); /* GL_MODELVIEW is used to set up the model and translate into camera space */
	glLoadIdentity(); /* Initialise to identity matirix */


	glColor3f(1.0,0.0,0.0); /* Draw in Red */

	gluLookAt(camera.x,camera.y,camera.z, 0,0,0, 0,1,0); /* Eye, then target, then up */
	glTranslatef(pos.x,pos.y,pos.z); /* Translate to viewpoint */
	glRotatef(ang.x,1.0,0.0,0.0); /* Rotate around X axis */
	glRotatef(ang.y,0.0,1.0,0.0); /* Rotate around Y axis */
	glRotatef(ang.z,0.0,0.0,1.0); /* Rotate around Z axis */

	/* Draw the plane */
	glPushMatrix();
	glTranslatef(planeOffset.x+camera.x-pos.x, planeOffset.y+camera.y-pos.y, planeOffset.z+camera.z-pos.z);
	glRotatef(-90.0,1.0,0.0,0.0); /* Rotate around X axis */
	glRotatef(-90,0.0,0.0,1.0); /* Rotate around Z axis */

	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	drawMesh(planeMesh);
	glDisable(GL_TEXTURE_2D);

	glPopMatrix();



	/* Draw the rings */
	GLfloat midpoint = (mCols-1)/(GLfloat)2;
	int i,j;
	for(i=0; i < mRows; i++)
		for(j=0; j < mCols; j++)
			if(map[i][j] != 0)
			{
				glPushMatrix();
				glTranslatef((GLfloat)(X_SCLR*(i+1)),(GLfloat)(Y_SCLR*map[i][j])/(GLfloat)3.0, Z_SCLR*(j - midpoint) ); /* Distance then height then left/right */
				glRotatef(90.0,0.0,1.0,0.0);
				glutSolidTorus(TORUS_INNER,TORUS_OUTER,TORUS_DETAIL,TORUS_DETAIL);
				glPopMatrix();
			}

	/* Draw the walls */
	GLfloat xUprBnd = (GLfloat)(X_SCLR*(mRows)) + 5.0;
	GLfloat xLwrBnd = -5.0;
	GLfloat yUprBnd = (GLfloat)(Y_SCLR*5); /* WARNING - NEEDS FILLING WITH REAL VALUE!! */
	GLfloat yLwrBnd = -TORUS_OUTER;
	GLfloat zUprBnd = Z_SCLR*midpoint + TORUS_OUTER;
	GLfloat zLwrBnd = -zUprBnd;

	glColor3f(0.0,1.0,0.0); /* Draw walls in green */
	glBegin(GL_POLYGON); /* Floor */
		glVertex3f (xLwrBnd, yLwrBnd, zLwrBnd); /* Distance, then Height, then L/R */
		glVertex3f (xLwrBnd, yLwrBnd, zUprBnd);
		glVertex3f (xUprBnd, yLwrBnd, zUprBnd);
		glVertex3f (xUprBnd, yLwrBnd, zLwrBnd);
	glEnd();

	glBegin(GL_POLYGON); /* Back wall */
	glVertex3f (xUprBnd, yLwrBnd, zLwrBnd);
		glVertex3f (xUprBnd, yLwrBnd, zUprBnd);
		glVertex3f (xUprBnd, yUprBnd, zUprBnd);
		glVertex3f (xUprBnd, yUprBnd, zLwrBnd);
	glEnd();


	
	glFlush(); /* Execute all isssued commands */

}

/* This callback occurs upon button press */
void mouse(int button, int state, int x, int y)
{

}

void keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
	case 'W': case 'w':
//		pos.x -=POS_INC;
//		glutPostRedisplay();
		adjForce(UP);
		break;

	case 'S': case 's':
//		pos.x +=POS_INC;
//		glutPostRedisplay();
		adjForce(DOWN);
		break;

	case 'A': case 'a':
		pos.z +=POS_INC;
		glutPostRedisplay();
		break;

	case 'D': case 'd':
		pos.z -=POS_INC;
		glutPostRedisplay();
		break;

	case 'O': case 'o':
		pos.y +=POS_INC;
		glutPostRedisplay();
		break;

	case 'L': case 'l':
		pos.y -=POS_INC;
		glutPostRedisplay();
		break;

	case 'Y': case 'y':
		ang.x +=ANG_INC;
		glutPostRedisplay();
		break;

	case 'H': case 'h':
		ang.x -=ANG_INC;
		glutPostRedisplay();
		break;

	case 'U': case 'u':
		ang.y +=ANG_INC;
		glutPostRedisplay();
		break;

	case 'J': case 'j':
		ang.y -=ANG_INC;
		glutPostRedisplay();
		break;

	case 'I': case 'i':
		ang.z +=ANG_INC;
		glutPostRedisplay();
		break;

	case 'K': case 'k':
		ang.z -=ANG_INC;
		glutPostRedisplay();
		break;
	default:
		break;
	}
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
//		if(force > 0)
			force -= forceIncrement;
	}
}

void setPosition(int x)
{
	GLfloat delayInSeconds = (GLfloat)delay/(GLfloat)1000;

	GLfloat acceleration;

	if(velocity >= 0)
		acceleration= force - airResistanceCoefficient*velocity*velocity;
	else
		acceleration= force + airResistanceCoefficient*velocity*velocity;

	velocity = velocity + acceleration*delayInSeconds; // v = u + at

	pos.x += velocity*delayInSeconds;

	glutPostRedisplay();
	
	glutTimerFunc(delay, setPosition, 0);
}