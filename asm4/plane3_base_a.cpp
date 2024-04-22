//|___________________________________________________________________
//!
//! \file plane3_base.cpp
//!
//! \brief Base source code for the third plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard inputs for plane and propeller (subpart):
//!   s   = moves the plane forward
//!   f   = moves the plane backward
//!   q,e = rolls the plane
//!   a   = yaws the plane
//!   x   = pitches the plane
//!
//!   r   = rotates propeller
//!
//!   i,k = translates light up/down
//!   9   = toggles diffuse light on/off
//!
//! Mouse inputs for world-relative camera:
//!   Hold left button and drag  = controls azimuth and elevation 
//!                                (Press CTRL (and hold) before left button to restrict to azimuth control only,
//!                                 Press SHIFT (and hold) before left button to restrict to elevation control only)   
//!   Hold right button and drag = controls distance
//!
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

#define _CRT_SECURE_NO_WARNINGS

//|___________________
//|
//| Includes
//|___________________

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>
//
//|___________________
//|
//| Constants
//|___________________

// Plane dimensions
const float P_WIDTH        = 3;
const float P_LENGTH       = 3;
const float P_HEIGHT       = 1.5f;

// Plane transforms
const gmtl::Vec3f PLANE_FORWARD(0, 0, 1.0f);            // Plane's forward translation vector (w.r.t. local frame)
const float PLANE_ROTATION = 5.0f;                      // Plane rotated by 5 degs per input

// Propeller dimensions (subpart)
const float PP_WIDTH       = 0.25f;
const float PP_LENGTH      = 1.5f;

// Propeller transforms
const gmtl::Point3f PROPELLER_POS(P_WIDTH/4, 0, 0);     // Propeller position on the plane (w.r.t. plane's frame)
const float PROPELLER_ROTATION = 5.0f;                  // Propeller rotated by 5 degs per input

// Camera's view frustum 
const float CAM_FOV        = 90.0f;                     // Field of view in degs

// Keyboard modifiers
enum KeyModifier {KM_SHIFT = 0, KM_CTRL, KM_ALT};

// Textures
enum TextureID {TID_SKYBACK = 0, TID_SKYLEFT, TID_SKYBOTTOM, TEXTURE_NB};  // Texture IDs, with the last ID indicating the total number of textures

// Skybox
const float SB_SIZE        = 1000.0f;                     // Skybox dimension

// Lighting
const GLfloat NO_LIGHT[] = {0.0, 0.0, 0.0, 1.0};
const GLfloat AMBIENT_LIGHT[]  = { 0.1, 0.1, 0.1, 1.0 };                
const GLfloat DIFFUSE_LIGHT[]  = { 0.5, 0.5, 0.5, 1.0 };                
const GLfloat SPECULAR_LIGHT[] = { 0.5, 0.5, 0.5, 1.0 };

// Materials
const GLfloat DARKRED_COL[]     = { 0.1, 0.0, 0.0, 1.0 };
const GLfloat BRIGHTRED_COL[]   = { 0.7, 0.0, 0.0, 1.0 };
const GLfloat DARKBLUE_COL[]    = { 0.0, 0.0, 0.1, 1.0 };      
const GLfloat BRIGHTBLUE_COL[]  = { 0.0, 0.0, 0.7, 1.0 };
const GLfloat DARK_COL[]        = { 0.1, 0.1, 0.1, 1.0 };      
const GLfloat MEDIUMWHITE_COL[] = { 0.7, 0.7, 0.7, 1.0 };
const GLfloat SPECULAR_COL[]    = { 0.7, 0.7, 0.7, 1.0 };

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width    = 800;
int w_height   = 600;

// Plane pose (position-quaternion pair)
gmtl::Point4f plane_p;      // Position (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q;        // Quaternion

// Quaternions to rotate plane
gmtl::Quatf zrotp_q;        // Positive and negative Z rotations
gmtl::Quatf zrotn_q;

gmtl::Quatf xrotp_q;
gmtl::Quatf yrotp_q;

// Propeller rotation (subpart)
float pp_angle = 0;         // Rotation angle

// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3]   = {false, false, false};
bool kmodifiers[3] = {false, false, false};

// Cameras
int cam_id         = 0;                                // Selects which camera to view
int camctrl_id     = 0;                                // Selects which camera to control
float distance[2]  = { 30.0f,  30.0f};                 // Distance of the camera from world's origin
float elevation[2] = {-45.0f, -45.0f};                 // Elevation of the camera (in degs)
float azimuth[2]   = { 15.0f,  15.0f};                 // Azimuth of the camera (in degs)

// Lighting
gmtl::Point4f light_pos(0.0, 20.0, 20.0, 1.0);
bool is_diffuse_on = true;

// Textures
GLuint textures[TEXTURE_NB];                           // Textures

//|___________________
//|
//| Function Prototypes
//|___________________

gmtl::Vec3f FindNormal(const gmtl::Point3f &p1, const gmtl::Point3f &p2, const gmtl::Point3f &p3);
void InitTransforms();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawPlaneBody(const float width, const float length, const float height);
void DrawPropeller(const float width, const float length);
void DrawSkybox(const float s);
void SetLight(const gmtl::Point4f &pos, const bool is_ambient, const bool is_diffuse, const bool is_specular);
void LoadPPM(const char *fname, unsigned int *w, unsigned int *h, unsigned char **data, const int mallocflag);

//|____________________________________________________________________
//|
//| Function: FindNormal
//|
//! \param p1	[in] Point 1.
//! \param p2	[in] Point 2.
//! \param p3	[in] Point 3.
//! \return Normalized surface normal.
//!
//! Finds the surface normal of a triangle. The input must be in CCW order.
//|____________________________________________________________________

gmtl::Vec3f FindNormal(const gmtl::Point3f &p1, 
					   const gmtl::Point3f &p2, 
					   const gmtl::Point3f &p3)
{
	gmtl::Vec3f v12 = p2-p1;
	gmtl::Vec3f v13 = p3-p1;
	
	gmtl::Vec3f normal;
	gmtl::cross(normal, v12, v13);
	gmtl::normalize(normal);

	return normal;
}

//|____________________________________________________________________
//|
//| Function: InitTransforms
//|
//! \param None.
//! \return None.
//!
//! Initializes all the transforms
//|____________________________________________________________________

void InitTransforms()
{
  const float COSTHETA_D2  = cos(gmtl::Math::deg2Rad(PLANE_ROTATION/2));  // cos() and sin() expect radians 
  const float SINTHETA_D2  = sin(gmtl::Math::deg2Rad(PLANE_ROTATION/2));

  // Inits plane pose
  plane_p.set(1.0f, 0.0f, 4.0f, 1.0f);
  plane_q.set(0, 0, 0, 1);

  // Z rotations (roll)
  zrotp_q.set(0, 0, SINTHETA_D2, COSTHETA_D2);      // +Z
  zrotn_q = gmtl::makeConj(zrotp_q);                // -Z

  // X rotation (pitch)
  xrotp_q.set(SINTHETA_D2, 0, 0, COSTHETA_D2);      // +X

  // Y rotation (yaw)
  yrotp_q.set(0, SINTHETA_D2, 0, COSTHETA_D2);      // +Y
  
  // TODO: Initializes the remaining transforms
}

//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
  unsigned char *img_data;               // Texture image data
  unsigned int  width;                   // Texture width
  unsigned int  height;                  // Texture height

  glClearColor(0.7f, 0.7f, 0.7f, 1.0f); 
  glEnable(GL_DEPTH_TEST); 
  glShadeModel(GL_SMOOTH);

//|___________________________________________________________________
//|
//| Setup lighting
//|___________________________________________________________________

  // Disable global ambient
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, NO_LIGHT);

  // NOTE: for specular reflections, the "local viewer" model produces better
  // results than the default, but is slower. The default would not use the correct
  // vertex-to-eyepoint vector, treating it as always parallel to Z.
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

  // Enable two sided lighting
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  // Enable lighting
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

//|___________________________________________________________________
//|
//| Setup texturing
//|___________________________________________________________________

  // Describe how data will be stored in memory
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  

  // Select the method for combining texture color with the lighting equation
	// (look up the third parameter)
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	// Generate and setup texture objects
  glGenTextures(TEXTURE_NB, textures);  

  // Skybox back wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBACK]);
  LoadPPM("skybox_back.ppm", &width, &height, &img_data, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  free(img_data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);        
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Skybox left wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYLEFT]);
  LoadPPM("skybox_left.ppm", &width, &height, &img_data, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  free(img_data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);        
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Skybox top wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBOTTOM]);
  LoadPPM("skybox_bottom.ppm", &width, &height, &img_data, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  free(img_data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);        
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // TODO: Initializes the remaining textures
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
  gmtl::AxisAnglef aa;    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
  gmtl::Vec3f axis;       // Axis component of axis-angle representation
  float angle;            // Angle component of axis-angle representation

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(CAM_FOV, (float)w_width/w_height, 0.1f, 1000.0f);     // Check MSDN: google "gluPerspective msdn"

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

//|____________________________________________________________________
//|
//| Setting up view transform by:
//| "move up to the world frame by composing all of the (inverse) transforms from the camera up to the world node"
//|____________________________________________________________________

  switch (cam_id) {
	case 0:
	  // For the world-relative camera
	  glTranslatef(0, 0, -distance[0]);
	  glRotatef(-elevation[0], 1, 0, 0);
	  glRotatef(-azimuth[0], 0, 1, 0);
	  break;

	case 1:
	  // For plane2's camera
	  glTranslatef(0, 0, -distance[1]);
	  glRotatef(-elevation[1], 1, 0, 0);
	  glRotatef(-azimuth[1], 0, 1, 0);

	  gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	  axis  = aa.getAxis();
	  angle = aa.getAngle();
	  glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
	  glTranslatef(-plane_p[0], -plane_p[1], -plane_p[2]);      
	  break;

	  // TODO: Add case for the plane1's camera
  }

//|____________________________________________________________________
//|
//| Draw traversal begins, start from world (root) node
//|____________________________________________________________________

  // Set light position wrt world
  SetLight(light_pos, true, is_diffuse_on, true);
  // DrawLight();

  // World node: draws world coordinate frame
  DrawCoordinateFrame(10);
  DrawSkybox(SB_SIZE);

  // World-relative camera:
  if (cam_id != 0) {
	glPushMatrix();        
	  glRotatef(azimuth[0], 0, 1, 0);
	  glRotatef(elevation[0], 1, 0, 0);
	  glTranslatef(0, 0, distance[0]);
	  DrawCoordinateFrame(1);
	glPopMatrix();
  }

  // Plane 2 body:
  glPushMatrix();
	gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	axis  = aa.getAxis();
	angle = aa.getAngle();
	glTranslatef(plane_p[0], plane_p[1], plane_p[2]);
	glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
	DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(3);

	// Plane 2's camera:
	glPushMatrix();
	  glRotatef(azimuth[1], 0, 1, 0);
	  glRotatef(elevation[1], 1, 0, 0);
	  glTranslatef(0, 0, distance[1]);
	  DrawCoordinateFrame(1);
	glPopMatrix();

	// Propeller (subpart):
	glPushMatrix();
	  glTranslatef(PROPELLER_POS[0], PROPELLER_POS[1], PROPELLER_POS[2]);     // Positions propeller on the plane
	  glRotatef(pp_angle, 0, 0, 1);                                           // Rotates propeller
	  DrawPropeller(PP_WIDTH, PP_LENGTH);
	  DrawCoordinateFrame(1);
	glPopMatrix();
  glPopMatrix();

  glutSwapBuffers();                          // Replaces glFlush() to use double buffering
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param key    [in] Key code.
//! \param x      [in] X-coordinate of mouse when key is pressed.
//! \param y      [in] Y-coordinate of mouse when key is pressed.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
  switch (key) {
//|____________________________________________________________________
//|
//| Camera switch
//|____________________________________________________________________

	case 'v': // Select camera to view
	  cam_id = (cam_id + 1) % 2;
	  printf("View camera = %d\n", cam_id);
	  break;
	case 'b': // Select camera to control
	  camctrl_id = (camctrl_id + 1) % 2;
	  printf("Control camera = %d\n", camctrl_id);
	  break;

//|____________________________________________________________________
//|
//| Plane controls
//|____________________________________________________________________

	case 's': { // Forward translation of the plane (+Z translation)  
	  gmtl::Quatf v_q = plane_q * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
	  plane_p         = plane_p + v_q.mData;
	  } break;
	case 'f': { // Backward translation of the plane (-Z translation)
	  gmtl::Quatf v_q = plane_q * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
	  plane_p         = plane_p + v_q.mData;
	  } break;

	case 'e': // Rolls the plane (+Z rot)
	  plane_q = plane_q * zrotp_q;
	  break;
	case 'q': // Rolls the plane (-Z rot)
	  plane_q = plane_q * zrotn_q;
	  break;

	case 'x': // Pitches the plane (+X rot)
	  plane_q = plane_q * xrotp_q;
	  break;
	case 'a': // Yaws the plane (+Y rot)
	  plane_q = plane_q * yrotp_q;
	  break;

//|____________________________________________________________________
//|
//| Propeller controls (subpart)
//|____________________________________________________________________

	case 'r': // Rotates propeller 
	  pp_angle += PROPELLER_ROTATION;
	  break;

//|____________________________________________________________________
//|
//| Lighting controls
//|____________________________________________________________________

	case 'i': // Light up (+Y translation)
	  light_pos[1]++;
	  printf("Light-Y = %.2f\n", light_pos[1]);
	  break;
	case 'k': // Light down (-Y translation)
	  light_pos[1]--;
	  printf("Light-Y = %.2f\n", light_pos[1]);
	  break;

	case '9': // Toggles diffuse light ON/OFF
	  is_diffuse_on = !is_diffuse_on;
	  printf("Light-diffuse = %s\n", is_diffuse_on ? "ON" : "OFF");
	  break;

	// TODO: Add the remaining controls/transforms        
  }

  glutPostRedisplay();                    // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MouseFunc
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void MouseFunc(int button, int state, int x, int y)
{
  int km_state;

  // Updates button's sate and mouse coordinates
  if (state == GLUT_DOWN) {
	mbuttons[button] = true;
	mx_prev          = x;
	my_prev          = y;
  } else {
	mbuttons[button] = false;
  }

  // Updates keyboard modifiers
  km_state = glutGetModifiers();
  kmodifiers[KM_SHIFT] = km_state & GLUT_ACTIVE_SHIFT ? true : false;
  kmodifiers[KM_CTRL]  = km_state & GLUT_ACTIVE_CTRL  ? true : false;
  kmodifiers[KM_ALT]   = km_state & GLUT_ACTIVE_ALT   ? true : false;

  //glutPostRedisplay();      // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MotionFunc
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void MotionFunc(int x, int y)
{
  int dx, dy, d;

  if (mbuttons[GLUT_LEFT_BUTTON] || mbuttons[GLUT_RIGHT_BUTTON]) {
	// Computes distances the mouse has moved
	dx      = x - mx_prev;
	dy      = y - my_prev;

	// Updates mouse coordinates
	mx_prev = x;
	my_prev = y;

	// Hold left button to rotate camera
	if (mbuttons[GLUT_LEFT_BUTTON]) {
	  if (!kmodifiers[KM_CTRL]) {        
		elevation[camctrl_id] += dy;            // Elevation update
	  }
	  if (!kmodifiers[KM_SHIFT]) {      
		azimuth[camctrl_id] += dx;             // Azimuth update
	  }
	}

	// Hold right button to zoom
	if (mbuttons[GLUT_RIGHT_BUTTON]) {
	  if (abs(dx) >= abs(dy)) {
		d = dx;
	  } else {
		d = -dy;
	  }
	  distance[camctrl_id] += d;    
	}

	glutPostRedisplay();      // Asks GLUT to redraw the screen
  }
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
  // Track the current window dimensions
  w_width  = w;
  w_height = h;
  glViewport(0, 0, (GLsizei) w_width, (GLsizei) w_height);
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
  glDisable(GL_LIGHTING);

  glBegin(GL_LINES);
	// X axis is red
	glColor3f( 1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(   l, 0.0f, 0.0f);

	// Y axis is green
	glColor3f( 0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(0.0f,    l, 0.0f);

	// Z axis is blue
	glColor3f( 0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(0.0f, 0.0f,    l);
  glEnd();

  glEnable(GL_LIGHTING);
}

//|____________________________________________________________________
//|
//| Function: DrawPlaneBody
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws a plane body.
//|____________________________________________________________________

void DrawPlaneBody(const float width, const float length, const float height)
{
  float w = width/2;
  float l = length/2;

  // Sets materials
  glMaterialf(GL_FRONT_AND_BACK,  GL_SHININESS, 20.0);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  SPECULAR_COL);
  
  glBegin(GL_TRIANGLES);
	// Body is red
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTRED_COL);
	glNormal3f(0.0f, 1.0f, 0.0f);     // Must be unit vector for correct lighting calculation
	glVertex3f(0.0f, 0.0f,    l);
	  glVertex3f(   w, 0.0f,   -l);
	  glVertex3f(  -w, 0.0f,   -l);

	// Wing is blue
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKBLUE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTBLUE_COL);
	glNormal3f(1.0f,    0.0f, 0.0f);  // Must be unit vector for correct lighting calculation
	glVertex3f(0.0f,    0.0f, 0.0f);
	  glVertex3f(0.0f,    0.0f,   -l);
	  glVertex3f(0.0f,  height,   -l);
  glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawPropeller
//|
//! \param width       [in] Width  of the propeller.
//! \param length      [in] Length of the propeller.
//! \return None.
//!
//! Draws a propeller.
//|____________________________________________________________________

void DrawPropeller(const float width, const float length)
{
  float w = width/2;
  float l = length/2;

  // Sets materials
  glMaterialf(GL_FRONT_AND_BACK,  GL_SHININESS, 20.0);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  SPECULAR_COL);

  glBegin(GL_QUADS);
	// Propeller is white
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glNormal3f(0.0f, 0.0f, 1.0f);  // Must be unit vector for correct lighting calculation
	glVertex3f(  -w,   -l, 0.0f);
	  glVertex3f(   w,   -l, 0.0f);
	  glVertex3f(   w,    l, 0.0f);
	glVertex3f(  -w,    l, 0.0f);
  glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawSkybox
//|
//! \param s      [in] Skybox size.
//! \return None.
//!
//! Draws a skybox.
//|____________________________________________________________________

void DrawSkybox(const float s)
{
  float s2 = s/2;

  // Turn on texture mapping and disable lighting
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);
  
  // Back wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBACK]);  // Specify which texture will be used   
  glBegin(GL_QUADS);
	glColor3f(0.0f, 0.0f, 0.5f);
	glTexCoord2f(0.0,  1.0);
	glVertex3f(-s2, -s2, -s2);
	glTexCoord2f(1.0,  1.0);
	  glVertex3f( s2, -s2, -s2);
	glTexCoord2f(1.0,  0.0);
	  glVertex3f( s2,  s2, -s2);
	glTexCoord2f(0.0,  0.0);
	glVertex3f(-s2,  s2, -s2);
  glEnd();

  // Left wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYLEFT]);
  glBegin(GL_QUADS);
	glColor3f(0.0f, 0.0f, 0.5f);
	glTexCoord2f(0.0,  1.0);
	glVertex3f(-s2, -s2,  s2);
	glTexCoord2f(1.0,  1.0);
	  glVertex3f(-s2, -s2, -s2);
	glTexCoord2f(1.0,  0.0);
	  glVertex3f(-s2,  s2, -s2);
	glTexCoord2f(0.0,  0.0);
	glVertex3f(-s2,  s2,  s2);   
  glEnd();

  // Bottom wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBOTTOM]);
  glBegin(GL_QUADS);
	glColor3f(0.0f, 0.0f, 0.5f);
	glTexCoord2f(0.0,  1.0);
	glVertex3f(-s2, -s2,  s2);
	glTexCoord2f(1.0,  1.0);
	  glVertex3f( s2, -s2,  s2);
	glTexCoord2f(1.0,  0.0);
	  glVertex3f( s2, -s2, -s2);
	glTexCoord2f(0.0,  0.0);
	glVertex3f(-s2, -s2, -s2);   
  glEnd();

  // Turn off texture mapping and enable lighting
  glEnable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
}

//|____________________________________________________________________
//|
//| Function: SetLight
//|
//! \param pos          [in] Light position.
//! \param is_ambient   [in] Is ambient enabled?
//! \param is_diffuse   [in] Is diffuse enabled?
//! \param is_specular  [in] Is specular enabled?
//! \return None.
//!
//! Set light properties.
//|____________________________________________________________________

void SetLight(const gmtl::Point4f &pos, const bool is_ambient, const bool is_diffuse, const bool is_specular)
{
	glLightfv(GL_LIGHT0, GL_POSITION, pos.mData);
  if (is_ambient) {
	  glLightfv(GL_LIGHT0, GL_AMBIENT,  AMBIENT_LIGHT);
  } else {
	glLightfv(GL_LIGHT0, GL_AMBIENT,  NO_LIGHT);
  }
  if (is_diffuse) {
	  glLightfv(GL_LIGHT0, GL_DIFFUSE,  DIFFUSE_LIGHT);
  } else {
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  NO_LIGHT);
  }
  if (is_specular) {
	  glLightfv(GL_LIGHT0, GL_SPECULAR, SPECULAR_LIGHT);
  } else {
	glLightfv(GL_LIGHT0, GL_SPECULAR,  NO_LIGHT);
  }
}

//|____________________________________________________________________
//|
//| Function: LoadPPM
//|
//! \param fname       [in]  Name of file to load.
//! \param w           [out] Width of loaded image in pixels.
//! \param h           [out] Height of loaded image in pixels.
//! \param data        [in/out] Image data address (input or output depending on mallocflag).
//! \param mallocflag  [in] 1 if memory not pre-allocated, 0 if data already points to allocated memory that can hold the image.
//! \return None.
//!
//! A very minimal Portable Pixelformat image file loader. Note that if new memory is allocated, free() should be used 
//! to deallocate when it is no longer needed.
//|____________________________________________________________________

void LoadPPM(const char *fname, unsigned int *w, unsigned int *h, unsigned char **data, const int mallocflag)
{
  FILE *fp;
  char P, num;
  int max;
  char s[1000];

  if (!(fp = fopen(fname, "rb")))
   {
	perror("cannot open image file\n");
	exit(0);
   }

  fscanf(fp, "%c%c\n", &P, &num);
  if ((P != 'P') || (num != '6'))
   {
	perror("unknown file format for image\n");
	exit(0);
   }

  do
   {
	fgets(s, 999, fp);
   }
  while (s[0] == '#');


  sscanf(s, "%d%d", w, h);
  fgets(s, 999, fp);
  sscanf(s, "%d", &max);

  if (mallocflag)
	if (!(*data = (unsigned char *)malloc(*w * *h * 3)))
	 {
	  perror("cannot allocate memory for image data\n");
	  exit(0);
	 }

  fread(*data, 3, *w * *h, fp);

  fclose(fp);
}

//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char **argv)
{ 
  InitTransforms();

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);     // Uses GLUT_DOUBLE to enable double buffering
  glutInitWindowSize(w_width, w_height);
  
  glutCreateWindow("Plane Episode 3");

  glutDisplayFunc(DisplayFunc);
  glutKeyboardFunc(KeyboardFunc);
  glutMouseFunc(MouseFunc);
  glutMotionFunc(MotionFunc);
  glutReshapeFunc(ReshapeFunc);
  
  InitGL();

  glutMainLoop();

  return 0;
}