/* Flight Simulator program for CVG
   Joshua Tyler 6213642 */

#define _CRT_SECURE_NO_WARNINGS // Disble warnings for using standard read/write/open functions

#include <Windows.h>
#include <gl/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mesh.h"
#include "imageloader.h"
#include <Xinput.h>

typedef struct
{
	GLfloat x;
	GLfloat y;
	GLfloat z;
} vector3d;

typedef struct
{
	GLfloat x;
	GLfloat y;
} vector2d;

typedef struct
{
	int rows;
	int cols;
	int height;
} mapParams;

enum ringMovement
{
	still,
	horizontal,
	vertical
};

typedef struct ringList ringList;
struct ringList
{
	vector3d position;
	enum ringMovement movement;
	int direction;
	ringList *next;
};

typedef enum
{
	behind,
	cockpit
} viewpoint;

/* Opengl functions */
void initGl(void);
void display(void);
void mouse(int button, int state, int x, int y);
void keyDown(unsigned char key, int x, int y);
void keyUp(unsigned char key, int x, int y);
void timer(int x);
void idle(void);
void passiveMouse(int x, int y);
void reshape(int width, int height);

/* File input functions */
int **readInput(char* filename, mapParams *levelParameters, int position);
ringList *arrayToLinkedList(int **posMap, int **stateMap, mapParams *params);
void storeRing(ringList **ringToProc,vector3d ringPos, int ringState);

/* Velocity/position/force functions */
void adjForce(int up, int down);
void calculatePosition(int x);

/* Vector math functions */
GLfloat vectorMag(vector3d vector);
vector3d vectorConstMult(vector3d vector, GLfloat constant);
vector3d vectorNorm(vector3d vector);
vector3d vectorAdd(vector3d vector1, vector3d vector2);
vector3d vectorInvert(vector3d vector);
vector3d vectorConvert(Vector3f vector);
vector3d rotateAboutY(vector3d position, GLfloat angle);
vector3d set3DVector(GLfloat a, GLfloat b, GLfloat c);

/* Other math functions */
GLfloat det3( vector3d col1, vector3d col2, vector3d col3);
GLfloat det2(GLfloat a, GLfloat b, GLfloat c, GLfloat d);
GLfloat coordAvg2(const GLfloat *vertices, int even);

/* Human input processing functions */
GLfloat procDirIn(GLfloat direction, int up, int down, GLfloat max, GLfloat inc, unsigned int retToZero);

/* Collision detection functions */
int ringCollDetect(vector3d centre);
int planeCollDetect(GLfloat *vertices, vector3d planePos);

/* Drawing functions */
void drawAxis(void);
void renderText(char *string, GLfloat x, GLfloat y, int centred);

/* Menu functions */
void drawMenu(char *item1, char *item2, char*item3, int activeItem);
void printItem(char *item, const GLfloat *vertices, int activeItem);
int findCurMenuBox(void);
int checkMenuBox(const GLfloat *vertices);

/* Texture functions */
void loadTexture(GLuint texture, char *filename);
void setCoordArray(GLfloat *array, GLfloat e0, GLfloat e1, GLfloat e2, GLfloat e3, GLfloat e4, GLfloat e5, GLfloat e6, GLfloat e7, GLfloat e8, GLfloat e9, GLfloat e10, GLfloat e11);
void setTexArray(GLfloat *array, GLfloat e0, GLfloat e1, GLfloat e2, GLfloat e3, GLfloat e4, GLfloat e5, GLfloat e6, GLfloat e7);
void loadCheckerTexData(void);

/* Misc functions */
void calcFps(void);
void newGame(int computerGame, int reset);
void nextLevel(void);
void setWalls(void);
void moveRings(void);




/* Variables relating to the ring arrays/linked lists */
//int totalRows, totalCols;
//GLfloat maxHeight;
//GLfloat midpoint;
ringList *firstRing;
ringList *currentRing;

/* Variables relating to levels */
#define NO_LEVELS 2
ringList *initialRing[NO_LEVELS];
int currentLevel;
mapParams levelParams[NO_LEVELS];

/* Keyboard state variables */

int keystate[256] = {0}; // Store if a key is pressed or not
int keyToggle[256] = {0}; //Store if a key change has not been read yet

/* Variables to store environment parameters */
int fogState = FALSE;
int pause = FALSE;
int autopilot = FALSE;
viewpoint cameraAngle;

/* Window size */
int windowWidth = 1280;
int windowHeight = 720;

/* Mouse control */
vector2d mousePos;

typedef enum
{
	none = 0,
	view = 1,
	control = 2
} mouseState;

mouseState mouseAction; // Store the current function of the mouse

int mouseViewLatch = TRUE;

/* Mathematical conversion factors */
const float radsToDegs (180.0/3.141592654);
const float degsToRads (3.141592654/180.0);

/* Scaling factors to convert array to real space */
const vector3d dirSclr = {100,15,25};

/* Constants to define how far wall is from real edge */
const vector3d dirMargin = {3.5,5,6};

/* Postition of the plane */
const vector3d initialPos = {-80,30,0};
vector3d pos; 

/* Amount to incrememnt the position when the input is processed */
const GLfloat posInc = 0.05;

/* Amount to move a ring by */
const GLfloat ringInc = 0.5;

/* Light parameters */
const GLfloat lightParam[2][4] = { {0.1,0.1,0.1,0.0},	//Ambient light - value is intensity
                                   {0.5,0.5,0.5,0.5} };	//Specular light

const GLfloat ringSpecular[] = {1.0, 1.0, 1.0, 1.0};
const GLfloat ringShininess[] = {25.0};

/* Fog parameters */
const GLfloat fogColor[] = {0.5,0.5,0.5,1.0};
const GLfloat fogDensity = 0.003;
const GLfloat fogStartDepth = 0.005;
const GLfloat fogEndDepth = 0.05;

/* Torus parameters */
const GLint torusSides = 5;
const GLint torusRings = 20;
const GLfloat torusOuterRad = 8.0;
const GLfloat torusInnerRad = 1;

/* Wall parameters */
GLfloat leftWallVertices[12];
GLfloat rightWallVertices[12];
GLfloat frontWallVertices[12];
GLfloat backWallVertices[12];
GLfloat ceilingVertices[12];
GLfloat floorVertices[12];

GLfloat leftWallTexCoords[8];
GLfloat rightWallTexCoords[8];
GLfloat backWallTexCoords[8];
GLfloat ceilingTexCoords[8];
GLfloat floorTexCoords[8];


/* Parameters for movement of the plane */
GLfloat force = 0; // Force output by the planes engine
GLfloat yAng = 0; // Angle ship rotates around y axis (yaw)

vector3d direction = {0,0,0}, velocity= {0,0,0}, normalisedDir = {1,0,0};

/* Constants to define if a quantity should be increased or decreased */
#define UP 1
#define DOWN 0

const GLfloat maxForce = 10000;
const GLfloat forceIncrement = 100;
const GLfloat airResistanceCoefficient = 0.1;
const unsigned int delay = 10;

/* Mesh and texture stuff */
#define PLANE_MESH_FILENAME "raptor.obj"
#define PLANE_TEXTURE_FILENAME "raptor.bmp"

#define SKY_TEXTURE_FILENAME "sky.bmp"
#define GROUND_TEXTURE_FILENAME "ground.bmp"

#define CHECKER_TEX_ROWS 128
#define CHECKER_TEX_COLS 128
GLubyte checkerTexData[CHECKER_TEX_ROWS][CHECKER_TEX_COLS][3];

Mesh planeMesh;
vector3d planeCentre, planeMax, planeMin; // Centre co-ordinates of plane, max and mix co-ordinates (used for collisision detection)

/* Array for opengl to store textures */
const GLsizei numTextures = 4;
GLuint texName[numTextures];

#define PLANE_TEXTURE_NUM 0
#define GROUND_TEXTURE_NUM 1
#define SKY_TEXTURE_NUM 2
#define CHECKER_TEXTURE_NUM 3

/* The size of the textures (i.e how many times they are repeated along the shortest edge */
const GLfloat wallTexSize = 1;
const GLfloat floorTexSize = 3;
const GLfloat endTexSize = 1;

/* Implement the GL_MIRRORED_REPEAT feature */
#ifndef GL_MIRRORED_REPEAT
#define GL_MIRRORED_REPEAT 0x8370
#endif

/* Parameters for the menu */
const GLfloat boxWidth = 0.5;
const GLfloat boxHeight = 0.2;

const GLfloat box1Coords[8] = { 0.0 - boxWidth/2.0 , 0.0 + boxHeight*2.5, // TL
                                0.0 + boxWidth/2.0 , 0.0 + boxHeight*2.5, // TR
                                0.0 + boxWidth/2.0 , 0.0 + boxHeight*1.5, // BR
                                0.0 - boxWidth/2.0 , 0.0 + boxHeight*1.5}; //BL

const GLfloat box2Coords[8] = { 0.0 - boxWidth/2.0 , 0.0 + boxHeight/2.0,
                                0.0 + boxWidth/2.0 , 0.0 + boxHeight/2.0,
                                0.0 + boxWidth/2.0 , 0.0 - boxHeight/2.0,
                                0.0 - boxWidth/2.0 , 0.0 - boxHeight/2.0,};


const GLfloat box3Coords[8] = { 0.0 - boxWidth/2.0 , 0.0 - boxHeight*1.5,
                                0.0 + boxWidth/2.0 , 0.0 - boxHeight*1.5,
                                0.0 + boxWidth/2.0 , 0.0 - boxHeight*2.5,
                                0.0 - boxWidth/2.0 , 0.0 - boxHeight*2.5};


/* Other stuff */
#define NO_LIVES 3

int showMenu = FALSE;
int score = 0;
int lives = 3;
int gameOver = FALSE;
float elapsedTime = 0;
float timeOffset = 0;
float fps;


int main(int argc, char **argv)
{
//	printf("Vendor: %s\nRenderer: %s\nVersion: %s\nExtensions: %s\n",(const char*)glGetString( GL_VENDOR),(const char*)glGetString( GL_RENDERER),(const char*)glGetString( GL_VERSION),(const char*)glGetString( GL_EXTENSIONS));

	/* Read levels */

	int **currentPosMap; /* Pointer to the level map array */
	int **currentStateMap;

	int i;
	char buf[100];
//	char stateBuf[100];
	for(i=0; i<NO_LEVELS; i++)
	{
		sprintf(buf,"level%d.txt",i);
//		sprintf(stateBuf,"levelState%d.txt",i);
		currentPosMap = readInput(buf, &levelParams[i], TRUE);
		currentStateMap = readInput(buf, &levelParams[i], FALSE);
		initialRing[i] = arrayToLinkedList(currentPosMap, currentStateMap, &levelParams[i]);
	}

	currentLevel = 0;

	cameraAngle = behind;

	firstRing = currentRing = initialRing[currentLevel];

	/* Initialise the GLUT window manager */
	glutInit(&argc, argv);       
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(windowWidth, windowHeight);
	glutCreateWindow("Flight Simulator");
	glutIgnoreKeyRepeat(GL_TRUE);

	/* Register callback functions */
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutKeyboardFunc(keyDown);
	glutKeyboardUpFunc(keyUp);
	glutTimerFunc(delay,timer,0);
	glutIdleFunc(idle);
	glutPassiveMotionFunc(passiveMouse);
	glutReshapeFunc(reshape);

	/* Load mesh  for plane */
	loadMesh(planeMesh, PLANE_MESH_FILENAME);

	/* Find centre, max and min co-ordinates */
	planeCentre = vectorConvert(getCentroid(planeMesh));
	planeMax = vectorConvert(getMax(planeMesh));
	planeMin = vectorConvert(getMin(planeMesh));

	/* Initialise OpenGL*/
	initGl();

	newGame(TRUE, TRUE);

	/* Loop forever and ever (but still call callback functions...) */
	glutMainLoop();

	return(EXIT_SUCCESS);
}

void initGl(void)
{
	reshape(windowWidth, windowHeight); /* Set up field of view */

	glClearColor (0.529, 0.808, 0.980, 1.0); /* Set background colour to blue */

	glLineWidth(5.0); // Line width as 5 if we need to draw some lines (i.e for drawing the axis)

	glEnable(GL_DEPTH_TEST); // Enable depth buffering
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Persepcitve correction - makes checkerboard undistorted


	/* Set up lighting */
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightParam[0]);
	glLightfv(GL_LIGHT1, GL_SPECULAR, lightParam[1]);

	glShadeModel (GL_SMOOTH);
	glMaterialfv(GL_FRONT, GL_SPECULAR, ringSpecular);
	glMaterialfv(GL_FRONT, GL_SHININESS, ringShininess);

	/* Set up fog */
	glFogi(GL_FOG_MODE, GL_EXP); // Rate of fade
	glFogfv(GL_FOG_COLOR, fogColor); // Colour (RGBA)
	glFogf(GL_FOG_DENSITY,fogDensity); // Density
    glHint(GL_FOG_HINT, GL_DONT_CARE);
	glFogf(GL_FOG_START, fogStartDepth); // Start depth
	glFogf(GL_FOG_END,fogEndDepth); // End depth

	/* Load the textures */
	glGenTextures(numTextures, texName);
	loadTexture(PLANE_TEXTURE_NUM, PLANE_TEXTURE_FILENAME);
	loadTexture(GROUND_TEXTURE_NUM, GROUND_TEXTURE_FILENAME);
	loadTexture(SKY_TEXTURE_NUM, SKY_TEXTURE_FILENAME);
	loadCheckerTexData();

	setWalls();
}

/* This callback occurs whenever the system determines the window needs redrawing (or upon a call of glutPostRedisplay()) */
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); /*Clear color and depth buffers*/

	glMatrixMode(GL_MODELVIEW); /* GL_MODELVIEW is used to set up the model and translate into camera space */
	glLoadIdentity(); /* Initialise to identity matirix */


//	gluLookAt(-5,0,0, 0,0,0, 0,1,0); // Debug - directly behind view
//	gluLookAt(0,100,0, 0,0,0, 1,0,0); // Debug - directly above view

	switch (cameraAngle)
	{
		case behind: default:
			gluLookAt(-5,0.5,0, 1,0,0, 0,1,0); // Normal ("behind") camera view
			break;

		case cockpit:
			gluLookAt(planeMax.x,0,0, 1,0,0, 0,1,0); // Normal ("behind") camera view
			break;
	}

	glRotatef(-yAng,0.0,1.0,0.0); // Rotate the viewpoint for the yaw rotation of the plane

	/* If the mouse is being used to rotate the camera, process that */
	static vector2d mouseRotation = {0,0}; // Store the mouse rotation - allows the rotation to be latched
	if(mouseAction == view && cameraAngle == behind)
	{
		if(mouseViewLatch == FALSE)
		{
			mouseRotation.x = -45.0+90*mousePos.x;
			mouseRotation.y = -45.0+90*mousePos.y;
		}
		glRotatef(mouseRotation.x,0.0,1.0,0.0);
		glRotatef(mouseRotation.y,0.0,0.0,1.0);
	}

	/* Translate to the current viewpoint */
	glTranslatef(-pos.x,-pos.y,-pos.z); 

	/* Draw the plane */
	glPushMatrix();

	glTranslatef(pos.x, pos.y, pos.z);	// Translate the plane to the current position
//	drawAxis(); 
	glRotatef(yAng,0.0,1.0,0.0); // Rotate the viewpoint for the yaw rotation of the plane

	/* Rotate camera so that plane is correctly orientated */
	glRotatef(-90.0,1.0,0.0,0.0); 
	glRotatef(-90,0.0,0.0,1.0);

	/* Rotate the plane in accordance with the current pith and roll direction of the plane */
	glRotatef(radsToDegs*sin(normalisedDir.z),0.0,1.0,0.0); /* L/R rotation */
	glRotatef(radsToDegs*sin(normalisedDir.y),1.0,0.0,0.0); /* U/D rotation */

	/* Draw the plane */
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, PLANE_TEXTURE_NUM);
	drawMesh(planeMesh);
	glDisable(GL_TEXTURE_2D);

	glPopMatrix();

	/* Draw the rings */
	glColor3f(1.0,0.0,0.0); // Draw first ring in red
	ringList *nextToDraw = currentRing;
	while(nextToDraw != NULL)
	{
		glPushMatrix();
		glTranslatef(nextToDraw->position.x,nextToDraw->position.y, nextToDraw->position.z); /* Distance then height then left/right */
		glRotatef(90.0,0.0,1.0,0.0);
		glutSolidTorus(torusInnerRad,torusOuterRad,torusSides,torusRings);
		glPopMatrix();

		glColor3f(0.0,0.0,1.0); // Draw remaining rings in green
		nextToDraw = nextToDraw->next;
	}

	/* Draw walls */
	glColor3f(0.0,1.0,0.0); /* Draw walls in green */


	glVertexPointer(3,GL_FLOAT,0,frontWallVertices);
	glDrawArrays(GL_POLYGON,0,4);

	glEnable(GL_TEXTURE_2D);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glBindTexture(GL_TEXTURE_2D, CHECKER_TEXTURE_NUM);
	glVertexPointer(3,GL_FLOAT,0,backWallVertices);
	glTexCoordPointer(2,GL_FLOAT,0,backWallTexCoords);
	glDrawArrays(GL_POLYGON,0,4);


	glBindTexture(GL_TEXTURE_2D, SKY_TEXTURE_NUM);
	glVertexPointer(3,GL_FLOAT,0,rightWallVertices);
	glTexCoordPointer(2,GL_FLOAT,0,rightWallTexCoords);
	glDrawArrays(GL_POLYGON,0,4);

	glVertexPointer(3,GL_FLOAT,0,leftWallVertices);
	glTexCoordPointer(2,GL_FLOAT,0,leftWallTexCoords);
	glDrawArrays(GL_POLYGON,0,4);

	glVertexPointer(3,GL_FLOAT,0,ceilingVertices);
	glTexCoordPointer(2,GL_FLOAT,0,ceilingTexCoords);
	glDrawArrays(GL_POLYGON,0,4);

	glBindTexture(GL_TEXTURE_2D, GROUND_TEXTURE_NUM);
	glVertexPointer(3,GL_FLOAT,0,floorVertices);
	glTexCoordPointer(2,GL_FLOAT,0,floorTexCoords);
	glDrawArrays(GL_POLYGON,0,4);

	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisable(GL_TEXTURE_2D);

	/* Render score */
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING);

	char stringToPrint[100];
	sprintf(stringToPrint,"Score: %d Lives: %d Level: %d FPS: %0.2f Timer: %0.2f", score, lives, currentLevel + 1, fps, elapsedTime);
	renderText(stringToPrint,-1,0.9, FALSE);
	glPopMatrix();

	glEnable( GL_LIGHTING);
	glEnable( GL_DEPTH_TEST );
	glMatrixMode( GL_PROJECTION ) ;
	glPopMatrix() ;
	glMatrixMode( GL_MODELVIEW ) ;
//	glPopMatrix() ;

	if(showMenu == TRUE)
		drawMenu("New Game", "Exit", NULL, findCurMenuBox());
	
	glFlush(); /* Execute all isssued commands */

}

/* This callback occurs upon button press */
void mouse(int button, int state, int x, int y)
{
	printf("MOUSE! %d\n", button);
	if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		if(showMenu == TRUE)
		{
			switch(findCurMenuBox())
			{
				case 1:
					newGame(FALSE, TRUE);
					break;
				case 2:
					exit(EXIT_SUCCESS);
					break;
				default:
					break;
			}

		} else {
			mouseViewLatch = !mouseViewLatch;
		}
	}

}

int **readInput(char* filename, mapParams *levelParameters, int position)
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

	if (fscanf(filePtr,"%d%d", &(levelParameters->rows), &(levelParameters->cols)) < 2)
	{
		fputs("Error, could not read input file.\n", stderr);
		exit(EXIT_FAILURE);
	}

//	midpoint = (totalCols-1)/(GLfloat)2; // Midpoint of the ring array, aligned with zero in real-world.

	int **map;

	/* Allocate map */
	int i;
	map = (int **)malloc(levelParameters->rows * sizeof(int *));
	for(i = 0; i < levelParameters->rows; i++)
		map[i] = (int *)malloc(levelParams->cols * sizeof(int));

	/* Read input to map */
	int j;
	char c;
	for(i=0; i < levelParameters->rows; i++)
		for(j=0; j < levelParameters->cols; j++)
		{
			if(position)
			{
				if (fscanf(filePtr,"%*1s%d", &map[i][j]) != 1)
				{
					fputs("Error, could not read input file.\n", stderr);
					exit(EXIT_FAILURE);
				}
			} else {
				do
				{
					c = fgetc(filePtr);
				} while( c != 'S' && c != 'H' && c != 'V');

				map[i][j] = c;
			}
		}
		fclose(filePtr);
	return map;
}


void adjForce(int up, int down)
{
	if(up == down)
	{
		if(force > 0)
			force -= forceIncrement;
		if(force < 0)
			force = 0;
		return;
	}

	if(up)
	{
		if(force < maxForce)
			force += forceIncrement;
	} else {
		if(force > 0)
			force -= 2*forceIncrement;
	}
//	printf("Force: %f\n", force);
}

void calculatePosition(void)
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
	keyToggle[key] = TRUE;

//	printf("Key: %c pressed.\n", key);
}

void keyUp(unsigned char key, int x, int y)
{
	keystate[key] = FALSE;
	keyToggle[key] = TRUE;

//	printf("Key: %c released.\n", key);
}

void timer(int x)
{
	if(gameOver)
	{
		glutTimerFunc(delay, timer, 0);
		return;
	}

	/* Collision test */
	static ringList *lastCollision = NULL;
	if(currentRing!= NULL && lastCollision != currentRing)
		{
			if(ringCollDetect(currentRing->position) == TRUE) // Detect a collision with the ring
				lastCollision = currentRing;
		}

	vector3d minPos = vectorAdd(pos, planeMin);
	vector3d maxPos = vectorAdd(pos, planeMax);
	if( planeCollDetect(leftWallVertices, minPos ) == TRUE)
		gameOver = TRUE;
	if( planeCollDetect(rightWallVertices, maxPos) == FALSE)
		gameOver = TRUE;
	if( planeCollDetect(ceilingVertices, maxPos) == TRUE)
		gameOver = TRUE;
	if( planeCollDetect(floorVertices, minPos) == FALSE)
		gameOver = TRUE;
	if( planeCollDetect(backWallVertices, maxPos) == TRUE)
		nextLevel();
//		gameOver = TRUE;

	if(lives == 0)
		gameOver = TRUE;

	if(gameOver)
	{
		pause = TRUE;
		puts("Game Over");
		if(autopilot)
			newGame(TRUE,TRUE);
		showMenu = TRUE;
		glutTimerFunc(delay, timer, 0);
		return;
	}

	moveRings();

	/* Process key presses */
	direction.x = 1;

	if(keystate['p'] == TRUE && keyToggle['p'] == TRUE) // Toggle pause
	{
		keyToggle['p'] = FALSE;
		pause = !pause;
	}

	if(keystate['q'] == TRUE && keyToggle['q'] == TRUE) // Toggle autopilot
	{
		keyToggle['q'] = FALSE;
		autopilot = !autopilot;
	}

	if(keystate['m'] == TRUE && keyToggle['m'] == TRUE) // Toggle mouse control
	{
		keyToggle['m'] = FALSE;
		switch(mouseAction)
		{
			case none:
				mouseAction = view;
				break;

			case view:
				mouseAction = control;
				break;

			case control:
				mouseAction = none;
				break;

			default:
				fprintf(stderr,"Invalid enum value on line %d.\n", __LINE__);
				exit(EXIT_FAILURE);
				break;
		}
	}

	if(keystate['v'] == TRUE && keyToggle['v'] == TRUE) // Toggle mouse control
	{
		keyToggle['v'] = FALSE;
		switch(cameraAngle)
		{
			case behind:
				cameraAngle = cockpit;
				break;

			case cockpit:
				cameraAngle = behind;
				break;

			default:
				fprintf(stderr,"Invalid enum value on line %d.\n", __LINE__);
				exit(EXIT_FAILURE);
				break;
		}
	}


	if(keystate['f'] == TRUE && keyToggle['f'] == TRUE) // Toggle fog
	{
		keyToggle['f'] = FALSE;
		fogState = ! fogState;

		if(fogState == TRUE)
			glEnable(GL_FOG);
		else
			glDisable(GL_FOG);
	}

	if(pause)
	{
		glutTimerFunc(delay, timer, 0);
		return;
	}

	adjForce(keystate['o'], keystate['l']);

//	/* Process new position */
//	if(keystate['o'] == TRUE)
//		adjForce(UP);
	
//	if(keystate['l'] == TRUE)
//		adjForce(DOWN);


	if(autopilot == TRUE)
	{
		force = 5000;

		if(currentRing != NULL)
		{
			if(pos.x > currentRing->position.x - torusInnerRad)
			{
				if(currentRing ->next != NULL)
					direction = vectorAdd(currentRing->next->position, vectorInvert(pos));
				else
				{
					direction.x = 1;
					direction.y = 0;
					direction.z = 0;
				}
			} else
				direction = vectorAdd(currentRing->position, vectorInvert(pos));
		} else {
			direction.x = 1;
			direction.y = 0;
			direction.z = 0;
		}


//			else if(currentRing->next != NULL)
//		{
//			direction = vectorAdd(currentRing->next->position, vectorInvert(pos));
//		}

	}


	else if(mouseAction == control)
	{
		direction.z = -5.0f + 10.0f*mousePos.x;
		direction.y = 5.0f - 10.0f*mousePos.y;
	} else {
		direction.y = procDirIn(direction.y, keystate['w'], keystate['s'], 5.0, posInc, TRUE);
		direction.z = procDirIn(direction.z, keystate['d'], keystate['a'], 5.0, posInc, TRUE);
	}

	yAng = procDirIn(yAng, keystate['i'],keystate['k'], 45.0, 1.0, TRUE);


//	printf("direction: x:%f, y:%f, z:%f\nvelocity: x:%f, y:%f, z:%f\npos: x:%f, y:%f, z:%f\nyAng: %f\n",direction.x,direction.y,direction.z, velocity.x,velocity.y, velocity.z,pos.x,pos.y,pos.z,yAng);

	calculatePosition();

	/* Check if we are passed the current ring, if so move to the next */
	if(currentRing != NULL)
	{
		if(currentRing->position.x + torusInnerRad < pos.x + planeMin.x) // If we are passed the ring
			currentRing = currentRing->next;
	}
	
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

vector3d vectorInvert(vector3d vector)
{
	vector.x = -vector.x;
	vector.y = -vector.y;
	vector.z = -vector.z;

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


GLfloat procDirIn(GLfloat direction, int up, int down, GLfloat max, GLfloat inc, unsigned int retToZero)
{
	if(up == down) // If both "up" and "down" keys are held return to zero.
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

	if(up == TRUE && direction < max) // Increment if we need to go up
	{
		if(direction < 0)
			direction += 2*inc;
		else
			direction += inc;
	}

	if(down == TRUE && direction > -max) // Decrement if we need to go down
	{
		if(direction > 0)
			direction -= 2*inc;
		else
			direction -= inc;
	}

	return direction;
}

int ringCollDetect(vector3d centre)
{

	const GLfloat torusTotal = torusOuterRad+torusInnerRad;
	const GLfloat torusGap = torusOuterRad-torusInnerRad;

	/* Check if inside */
	if(pos.x + planeMax.x > centre.x - torusInnerRad && pos.x + planeMin.x < centre.x + torusInnerRad)
	{
		if(pos.z + planeMin.z < centre.z + torusTotal && pos.z + planeMax.z > centre.z - torusTotal )
		{
			if(pos.y + planeMax.y > centre.y - torusTotal && pos.y + planeMin.y < centre.y + torusTotal)
			{
				/* If we're here, we're inside */
				score++;
				/* Check if collided */
				printf("%d %d %d %d\n", (pos.z + planeMax.z > centre.z + torusGap), (pos.z + planeMin.z < centre.z - torusGap), (pos.y + planeMin.y < centre.y - torusGap), (pos.y + planeMax.y > centre.y + torusGap));
				if( (pos.z + planeMax.z > centre.z + torusGap) || (pos.z + planeMin.z < centre.z - torusGap) || (pos.y + planeMin.y < centre.y - torusGap) || (pos.y + planeMax.y > centre.y + torusGap) )
				{
					score--;
					lives--;
//					printf("Collided.\n");
				}
				return TRUE;
			}
		}
	}

	return FALSE;
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

void renderText(char *string, GLfloat x, GLfloat y, int centred)
{
	int stringLength = strlen(string);
	int i;
	
	GLfloat xOffset = 0;
	GLfloat yOffset = 0;

	if(centred)
	{
		xOffset = -(GLfloat)glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char *)string)/(GLfloat)windowWidth;
		yOffset = -(GLfloat)9/(GLfloat)windowHeight;
	}
	
	glColor3f(0,0,0);
	glRasterPos2f(x + xOffset,y + yOffset); // Centralise on given position

	for(i=0; i < stringLength; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, string[i]);

}

void idle(void)
{
	static ringList *lastCollision = NULL;

	if(currentRing != NULL)
	{
		if(currentRing->position.x + torusInnerRad < pos.x + planeMin.x) // If we are passed the ring
			currentRing = currentRing->next;
	}

	if(!pause)
		calcFps();

	glutPostRedisplay();


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
    elapsedTime = curTime = glutGet(GLUT_ELAPSED_TIME);

	elapsedTime /= 1000; // Stores the time the user has been playing in seconds

	elapsedTime -= timeOffset;

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
	GLfloat cosAng = cos(degsToRads*angle);
	GLfloat sinAng = sin(degsToRads*angle);

	position.x = cosAng*position.x + sinAng*position.z;
	position.z = -sinAng*position.x + cosAng*position.z;

	return position;

}

void passiveMouse(int x, int y)
{
	mousePos.x = (GLfloat)x/(GLfloat)windowWidth;
	mousePos.y = (GLfloat)y/(GLfloat)windowHeight;
//	printf("x: %f, y:%f\n", mousePos.x, mousePos.y);
}

void reshape(int width, int height)
{
	if(height == 0) // Avoids division by zero problems
		height = 1;
	windowWidth = width;
	windowHeight = height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glViewport(0,0,windowWidth, windowHeight);

	gluPerspective(45.0, (GLdouble)windowWidth/(GLdouble)windowHeight, 1.0, 4000.0); /* Set up the field of view as perspective */
	glMatrixMode(GL_MODELVIEW);
}

void loadTexture(GLuint texture, char *filename)
{
	Image *img;
	img = loadBMP(filename);

	glEnable(GL_TEXTURE);

	glBindTexture(GL_TEXTURE_2D, texture);

	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

//	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img->width -1, img->height -1, 0, GL_RGB, GL_UNSIGNED_BYTE, img->pixels);

	gluBuild2DMipmaps(GL_TEXTURE_2D, 3, img->width, img->height, GL_RGB, GL_UNSIGNED_BYTE, img->pixels);

	glDisable(GL_TEXTURE);
}

void setCoordArray(GLfloat *array, GLfloat e0, GLfloat e1, GLfloat e2, GLfloat e3, GLfloat e4, GLfloat e5, GLfloat e6, GLfloat e7, GLfloat e8, GLfloat e9, GLfloat e10, GLfloat e11)
{
	array[0] = e0;
	array[1] = e1;
	array[2] = e2;
	array[3] = e3;
	array[4] = e4;
	array[5] = e5;
	array[6] = e6;
	array[7] = e7;
	array[8] = e8;
	array[9] = e9;
	array[10] = e10;
	array[11] = e11;
}

void setTexArray(GLfloat *array, GLfloat e0, GLfloat e1, GLfloat e2, GLfloat e3, GLfloat e4, GLfloat e5, GLfloat e6, GLfloat e7)
{
	array[0] = e0;
	array[1] = e1;
	array[2] = e2;
	array[3] = e3;
	array[4] = e4;
	array[5] = e5;
	array[6] = e6;
	array[7] = e7;
}


ringList *arrayToLinkedList(int **posMap, int **stateMap, mapParams *params)
{
	vector3d ringPos; // Stores the position co-ordinates of the current ring
//	int ringID = 0; // An identifier to store the current ring
	int i,j;

	params->height = 0;
	GLfloat midpoint = (GLfloat)( params->cols -1) / 2.0;

	ringList *firstRing = NULL, *currentRing = NULL;

	/* Loop across all rows */
	for(i=0; i < params->rows; i++)
	{
		ringPos.x = (GLfloat)(dirSclr.x*(i+1));
		for(j=0; j < params->cols; j++)
		{
			if(posMap[i][j] != 0) // i.e if there is a ring in the space
			{
				ringPos.y = (GLfloat)(dirSclr.y*posMap[i][j])/(GLfloat)3.0;
				ringPos.z = dirSclr.z*(j - midpoint);

				if(ringPos.y > params->height)
					params->height = ringPos.y;

				storeRing(&currentRing, ringPos, stateMap[i][j]);

				if(firstRing == NULL)
					firstRing = currentRing;

//				ringID++; // Increment ring ID
			}
		}
	}

	/* Add map freeing routine here */
	return firstRing;

}

void storeRing(ringList **ringToProc,vector3d ringPos, int ringState)
{
	if(*ringToProc == NULL)
	{
		*ringToProc = (ringList*) malloc(sizeof(ringList));
	} else {
		(*ringToProc)->next = (ringList*) malloc(sizeof(ringList));
		(*ringToProc) = (*ringToProc)->next;
	}

	(*ringToProc)->next = NULL;
	(*ringToProc)->position = ringPos;
	(*ringToProc)->direction = TRUE;
	switch(ringState)
	{
		case 'S':
			(*ringToProc)->movement = still;
			break;

		case 'H':
			(*ringToProc)->movement = horizontal;
			break;

		case 'V':
			(*ringToProc)->movement = vertical;
			break;

		default:
			fputs("Incorrect direction specifier.\n", stderr);
			exit(EXIT_FAILURE);
			break;
	}

}


int planeCollDetect(GLfloat *vertices, vector3d planePos)
{
	/* Using method found here
	http://math.stackexchange.com/questions/214187/point-on-the-left-or-right-side-of-a-plane-in-3d-space
	(Top answer) */

	
	vector3d pointA = set3DVector(vertices[0],vertices[1],vertices[2]);
	vector3d pointB = set3DVector(vertices[3],vertices[4],vertices[5]);
	vector3d pointC = set3DVector(vertices[6],vertices[7],vertices[8]);

	pointA = vectorInvert(pointA);

	pointB = vectorAdd(pointB,pointA);
	pointC = vectorAdd(pointC,pointA);
	planePos = vectorAdd(planePos,pointA);

	if(det3(pointB, pointC, planePos) < 0)
		return TRUE;


	return FALSE;



}

GLfloat det3( vector3d col1, vector3d col2, vector3d col3)
{
	return col1.x*det2(col2.y, col3.y, col2.z, col3.z) + col2.x*det2(col1.y, col3.y, col1.z, col3.z) + col3.x*det2(col1.y, col2.y, col1.z, col2.z);
}

GLfloat det2(GLfloat a, GLfloat b, GLfloat c, GLfloat d)
{
	return a*d-b*c;
}

vector3d set3DVector(GLfloat a, GLfloat b, GLfloat c)
{
	vector3d vect;

	vect.x = a;
	vect.y = b;
	vect.z = c;

	return vect;
}

void drawMenu(char *item1, char *item2, char*item3, int activeItem)
{
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();	
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING);
	glEnable(GL_BLEND); // Enable translucency
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 


	printItem(item1, box1Coords, activeItem == 1);

	printItem(item2, box2Coords, activeItem == 2);

	printItem(item3, box3Coords, activeItem == 3);

	glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);
	glEnable( GL_DEPTH_TEST );
	glMatrixMode( GL_PROJECTION ) ;
	glPopMatrix() ;
	glMatrixMode( GL_MODELVIEW ) ;
	glPopMatrix() ;

}

void printItem(char *item, const GLfloat *vertices, int activeItem)
{
	if(item != NULL)
	{
		if(activeItem == TRUE)
			glColor4f(1.0,0.0,0.0,0.7);
		else
			glColor4f(0.0,0.0,1.0,0.7);
		glVertexPointer(2,GL_FLOAT,0,vertices);
		glDrawArrays(GL_POLYGON,0,4);
		renderText(item,coordAvg2(vertices,TRUE),coordAvg2(vertices,FALSE), TRUE);
	}
}

GLfloat coordAvg2(const GLfloat *vertices, int even)
{
	if(even)
		return (vertices[0]+vertices[2]+vertices[4]+vertices[6])/4.0;
	else
		return (vertices[1]+vertices[3]+vertices[5]+vertices[7])/4.0;
}

int findCurMenuBox(void)
{
	if( checkMenuBox(box1Coords) == TRUE)
		return 1;

	if( checkMenuBox(box2Coords) == TRUE)
		return 2;

	if( checkMenuBox(box3Coords) == TRUE)
		return 3;

	return 0;
}

int checkMenuBox(const GLfloat *vertices)
{
	vector2d transformedMousePos;
	transformedMousePos.x = -1.0 +2.0*mousePos.x;
	transformedMousePos.y = 1.0 -2.0*mousePos.y;

	if(transformedMousePos.x >= vertices[0]) // To the right of top right
		if(transformedMousePos.x <= vertices[2]) // To the left of top left
			if(transformedMousePos.y <= vertices[1]) // Below top right
				if(transformedMousePos.y >= vertices[5]) // Above bottom right
					return TRUE;
	return FALSE;
}

void newGame(int computerGame, int reset)
{
	direction.x = direction.y = direction.z = velocity.x = velocity.y = velocity.z = force = yAng = 0;
	if(reset)
	{
		lives = NO_LIVES;
		score = currentLevel = 0;
	}
	pos = initialPos;
	currentRing = firstRing = initialRing[currentLevel];
	gameOver = pause = mouseViewLatch = fogState =FALSE;
	mouseAction = none;

	
	

	timeOffset = elapsedTime;
	elapsedTime = 0;

	if(computerGame)
		showMenu = autopilot = TRUE;
	else
		showMenu = autopilot = FALSE;

	setWalls();
}

/* Generate checker pattern, adapted from http://www.csc.villanova.edu/~mdamian/Past/graphicsS13/notes/GLTextures/Checkerboard.htm */
void loadCheckerTexData(void)
{
   int value;
   for (int row = 0; row < CHECKER_TEX_ROWS; row++) {
      for (int col = 0; col < CHECKER_TEX_COLS; col++) {
         // Each cell is 8x8, value is 0 or 255 (black or white)
         value = (((row & 0x8) == 0) ^ ((col & 0x8) == 0)) * 255;
         checkerTexData[row][col][0] = (GLubyte)value;
         checkerTexData[row][col][1] = (GLubyte)value;
         checkerTexData[row][col][2] = (GLubyte)value;
      }
   }

   	glEnable(GL_TEXTURE);

	glBindTexture(GL_TEXTURE_2D, CHECKER_TEXTURE_NUM);

	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glTexImage2D(GL_TEXTURE_2D, 0, 3, CHECKER_TEX_COLS, CHECKER_TEX_ROWS, 0, GL_RGB, GL_UNSIGNED_BYTE, checkerTexData);
//   glTexImage2D(GL_TEXTURE_2D, 0, 3, IMAGE_COLS, IMAGE_ROWS, 0, GL_RGB,  GL_UNSIGNED_BYTE, imageData);  // Create texture from image data

	glDisable(GL_TEXTURE);
}

void nextLevel(void)
{
	if(currentLevel < NO_LEVELS - 1)
		currentLevel++;
	else
		currentLevel = 0;
	newGame(autopilot, FALSE);
}

void setWalls(void)
{
		/* Set up parameters for the walls */
	GLfloat midpoint = (GLfloat)(levelParams[currentLevel].cols -1)/2.0;
	GLfloat xUprBnd = (GLfloat)(dirSclr.x* (levelParams[currentLevel].rows + dirMargin.x));
	GLfloat xLwrBnd = -dirSclr.x*dirMargin.x;
	GLfloat yUprBnd = ((GLfloat)levelParams[currentLevel].height + dirSclr.y*dirMargin.y );
	GLfloat yLwrBnd = -(torusOuterRad + dirMargin.y*dirSclr.y);
	GLfloat zUprBnd = dirSclr.z*(dirMargin.z + midpoint) + torusOuterRad;
	GLfloat zLwrBnd = -zUprBnd;

	GLfloat sideHeight = wallTexSize;
	GLfloat sideLength = ( (xUprBnd - xLwrBnd)/(yUprBnd - yLwrBnd) )*wallTexSize;

	GLfloat floorWidth = floorTexSize;
	GLfloat floorLength = ( (xUprBnd - xLwrBnd)/(zUprBnd - zLwrBnd) )*floorTexSize;

	GLfloat endHeight = endTexSize;
	GLfloat endLength = ( (zUprBnd - zLwrBnd)/(yUprBnd - yLwrBnd) )*endTexSize;

	/* Set co-ordinate arrays for walls */
	glEnableClientState(GL_VERTEX_ARRAY);
	setCoordArray(backWallVertices, xUprBnd, yLwrBnd, zLwrBnd,
		                            xUprBnd, yLwrBnd, zUprBnd,
		                            xUprBnd, yUprBnd, zUprBnd,
		                            xUprBnd, yUprBnd, zLwrBnd);

	setTexArray(backWallTexCoords, 0.0, 0.0,
		                           endLength, 0.0,
		                           endLength, endHeight,
		                           0.0, endHeight);

	setCoordArray(frontWallVertices, xLwrBnd, yLwrBnd, zLwrBnd,
		                             xLwrBnd, yLwrBnd, zUprBnd,
		                             xLwrBnd, yUprBnd, zUprBnd,
		                             xLwrBnd, yUprBnd, zLwrBnd);

	setCoordArray(leftWallVertices, xLwrBnd, yLwrBnd, zLwrBnd,
		                            xLwrBnd, yUprBnd, zLwrBnd,
		                            xUprBnd, yUprBnd, zLwrBnd,
		                            xUprBnd, yLwrBnd, zLwrBnd);

	setTexArray(leftWallTexCoords, 0.0, 0.0,
		                           0.0, sideHeight,
		                           sideLength, sideHeight,
		                           sideLength, 0.0);

	setCoordArray(rightWallVertices, xLwrBnd, yLwrBnd, zUprBnd,
		                            xLwrBnd, yUprBnd, zUprBnd,
		                            xUprBnd, yUprBnd, zUprBnd,
		                            xUprBnd, yLwrBnd, zUprBnd);

	setTexArray(rightWallTexCoords, 0.0, 0.0,
		                           0.0, sideHeight,
		                           sideLength, sideHeight,
		                           sideLength, 0.0);

	setCoordArray(ceilingVertices, xLwrBnd, yUprBnd, zLwrBnd,
		                           xLwrBnd, yUprBnd, zUprBnd,
		                           xUprBnd, yUprBnd, zUprBnd,
		                           xUprBnd, yUprBnd, zLwrBnd);

	setTexArray(ceilingTexCoords, 0.0, 0.0,
		                          floorWidth, 0.0,
		                          floorWidth, floorLength,
		                          0.0, floorLength);

	setCoordArray(floorVertices, xLwrBnd, yLwrBnd, zLwrBnd,
		                         xLwrBnd, yLwrBnd, zUprBnd,
		                         xUprBnd, yLwrBnd, zUprBnd,
		                         xUprBnd, yLwrBnd, zLwrBnd);

	setTexArray(floorTexCoords, 0.0, 0.0,
		                        floorWidth, 0.0,
		                        floorWidth, floorLength,
		                        0.0, floorLength);

}


void moveRings(void)
{
	GLfloat zLimit = (GLfloat)(levelParams[currentLevel].cols * dirSclr.z)/2.0;
	GLfloat yLimit = (GLfloat)(levelParams[currentLevel].height);

	ringList *nextToProc;
	for(nextToProc = currentRing; nextToProc != NULL; nextToProc = nextToProc->next )
	{
		if(nextToProc->movement == still)
			continue;
		else if(nextToProc->movement == horizontal)
		{
			if(nextToProc->direction == TRUE)
			{
				(nextToProc->position.z)+= ringInc;

				if(nextToProc->position.z > zLimit - torusOuterRad)
					nextToProc->direction = FALSE;
			} else {
				(nextToProc->position.z)-= ringInc;

				if(nextToProc->position.z < -zLimit + torusOuterRad)
					nextToProc->direction = TRUE;
			}
		} else {
			if(nextToProc->direction == TRUE)
			{
				(nextToProc->position.y)+= ringInc;

				if(nextToProc->position.y > yLimit - torusOuterRad)
					nextToProc->direction = FALSE;
			} else {
				(nextToProc->position.y)-= ringInc;

				if(nextToProc->position.y < 0 + torusOuterRad)
					nextToProc->direction = TRUE;
			}
		}
	}
}
