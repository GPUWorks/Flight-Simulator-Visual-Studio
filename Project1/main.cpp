/* Flight Simulator program for CVG
   Joshua Tyler 6213642 */

#include <Windows.h>
#include <gl/glut.h>
#include <stdio.h>
#include <stdlib.h>

void initGl(void);
void display(void);
void mouse(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);
void readInput(char* filename);

int mRows, mCols;
int **map; /* Pointer to the level map array */

/* Scaling factors to convert array to real space */
#define X_SCLR 10
#define Y_SCLR 10
#define Z_SCLR 10

GLfloat posX, posY, posZ, angX, angY, angZ;
#define ANG_INC 2
#define POS_INC 2

/* Light position */
static GLfloat light0_position[] = {1.0,1.0,1.0,0.0};

#define TORUS_DETAIL 100

int main(int argc, char **argv)
{
	/* Read input map */
	readInput("map.txt");

	posX = posY = posZ = angX = angY = angZ = 0;

	/* Initialise the GLUT window manager */
	glutInit(&argc, argv);       
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(1280, 720);
	glutCreateWindow("Flight Simulator");

	/* Register callback functions */
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);

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
}

/* This callback occurs whenever the system determines the window needs redrawing (or upon a call of glutPostRedisplay()) */
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); /*Clear color and depth buffers*/

	glMatrixMode(GL_MODELVIEW); /* GL_MODELVIEW is used to set up the model and translate into camera space */
	glLoadIdentity(); /* Initialise to identity matirix */


	glColor3f(1.0,0.0,0.0); /* Draw in Red */

	gluLookAt(-20,0,0, 0,0,0, 0,1,0); /* Eye, then target, then up */
	glTranslatef(posX,posY,posZ); /* Translate to viewpoint */
	glRotatef(angX,1.0,0.0,0.0); /* Rotate around X axis */
	glRotatef(angY,0.0,1.0,0.0); /* Rotate around Y axis */
	glRotatef(angZ,0.0,0.0,1.0); /* Rotate around Z axis */

	/* Draw the rings */
	int i,j;
	for(i=0; i < mRows; i++)
		for(j=0; j < mCols; j++)
			if(map[i][j] != 0)
			{
				glPushMatrix();
				glTranslatef((GLfloat)(X_SCLR*(i+1)),(GLfloat)(Y_SCLR*map[i][j])/(GLfloat)3.0,(GLfloat)(Z_SCLR*(j+1))); /* Distance then height then left/right */
				glRotatef(90.0,0.0,1.0,0.0);
				glutSolidTorus(0.3,3.0,TORUS_DETAIL,TORUS_DETAIL);
				glPopMatrix();
			}

	/* Draw the walls */
	GLfloat xUprBnd = (GLfloat)(X_SCLR*(mRows)) + 5.0;
	GLfloat xLwrBnd = -5.0;
	GLfloat yUprBnd = (GLfloat)(Y_SCLR*5); /* WARNING - NEEDS FILLING WITH REAL VALUE!! */
	GLfloat yLwrBnd = 0.0;
	GLfloat zUprBnd = (GLfloat)(Z_SCLR*(mCols)) +5.0;
	GLfloat zLwrBnd = -5.0;

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

	///* Draw two teapots */
	//glPushMatrix();
	//glTranslatef(5.0,0.0,1.25); /* Move origin so that teapot will appear 5.0 units away in the x direction */
	//glutSolidTeapot(0.5);
	//glPopMatrix();

	//glPushMatrix();
	//glTranslatef(5.0,0.0,-1.25); /* Move origin so that teapot will appear 5.0 units away in the x direction */
	//glutSolidTeapot(0.5);
	//glPopMatrix();
	
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
		posX -=POS_INC;
		glutPostRedisplay();
		break;

	case 'S': case 's':
		posX +=POS_INC;
		glutPostRedisplay();
		break;

	case 'A': case 'a':
		posZ +=POS_INC;
		glutPostRedisplay();
		break;

	case 'D': case 'd':
		posZ -=POS_INC;
		glutPostRedisplay();
		break;

	case 'O': case 'o':
		posY +=POS_INC;
		glutPostRedisplay();
		break;

	case 'L': case 'l':
		posY -=POS_INC;
		glutPostRedisplay();
		break;

	case 'Y': case 'y':
		angX +=ANG_INC;
		glutPostRedisplay();
		break;

	case 'H': case 'h':
		angX -=ANG_INC;
		glutPostRedisplay();
		break;

	case 'U': case 'u':
		angY +=ANG_INC;
		glutPostRedisplay();
		break;

	case 'J': case 'j':
		angY -=ANG_INC;
		glutPostRedisplay();
		break;

	case 'I': case 'i':
		angZ +=ANG_INC;
		glutPostRedisplay();
		break;

	case 'K': case 'k':
		angZ -=ANG_INC;
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
