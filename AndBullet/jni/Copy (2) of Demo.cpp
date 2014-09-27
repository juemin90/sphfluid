#include <jni.h>
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <btBulletDynamicsCommon.h>
#include "Plane.h"

#include "BulletFluids/Sph/btFluidSph.h"
#include "BulletFluids/Sph/btFluidSphSolver.h"
#include "BulletFluids/btFluidRigidCollisionConfiguration.h"
#include "LinearMath/btQuickprof.h"
#include "demos.h"
#include <android/log.h>
#define  LOG_TAG    "BulletDemo"
#define  LOG_PROFILE "FluidProfile"

#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_INFO, LOG_PROFILE, __VA_ARGS__)

#include "LinearMath/btRandom.h"


enum FluidRenderMode
{
	FRM_Points = 0,
	FRM_MediumSpheres,
	FRM_LargeSpheres,
	FRM_ScreenSpace,
	FRM_MarchingCubes
};



ScreenSpaceFluidRendererGL* m_screenSpaceRenderer;
//unsigned int vbo[2];

///Physics variables
btDiscreteDynamicsWorld* m_dynamicsWorld;
//btBroadphaseInterface* broadphase;
//btDefaultCollisionConfiguration* collisionConfiguration;
//btCollisionDispatcher* dispatcher;
//btSequentialImpulseConstraintSolver* solver;
//Demos
btAlignedObjectArray<FluidSystemDemo*> m_demos;

//Bullet
btAlignedObjectArray<btCollisionShape*> m_collisionShapes; //Keep the collision shapes, for deletion/cleanup
btBroadphaseInterface* m_broadphase;
btCollisionDispatcher* m_dispatcher;
btConstraintSolver* m_solver;
btDefaultCollisionConfiguration* m_collisionConfiguration;
//Fluid system
btFluidRigidDynamicsWorld* m_fluidWorld;
btAlignedObjectArray<btFluidSph*> m_fluids;
btFluidEmitter* m_emitter;

btFluidSphSolver* m_fluidSolverCPU;

btVector3 m_cameraPosition(0.0f, 100.0f, 20.0f);
btVector3 m_cameraTargetPosition(0.f,0.f,0.f);
btVector3 m_cameraUp(0.0f, 1.0f, 0.0f);
btScalar  m_defaultContactProcessingThreshold = BT_LARGE_FLOAT;

float	m_cameraDistance = 100.0f;
btCollisionShape* m_shootBoxShape;
btAlignedObjectArray<btRigidBody*> box_body;


btDefaultMotionState* motionState;

float m_ele = 20.f;
float m_azi = 0.f;


ESMatrix projMat;
ESMatrix viewMat;


float m_scaleBottom;
float m_scaleFactor;

int	m_forwardAxis =2;
float m_zoomStepSize;

int m_glutScreenWidth;
int m_glutScreenHeight;

float	m_frustumZNear = 1.f;
float	m_frustumZFar = 10000.f;

int	m_ortho;

float	m_ShootBoxInitialSpeed = 40.f;

bool	m_stepping;
bool m_singleStep;
bool m_idle;
int m_lastKey;

Plane *plane;

const int MIN_FLUID_PARTICLES = 2500;
const int MAX_FLUID_PARTICLES = 5000;

float plane_motion_x = 0;
float plane_motion_z = 0;

GLuint basicShader;

//short indices [] = {
//		0, 1, 2,
//		2, 3, 0
//};
float vertices []= {
		5.0f, 0.0f, 5.0f,
		-5.0f, 0.0f, 5.0f,
		-5.0f, 0.0f, -5.0f,
		5.0f, 0.0f, -5.0f,
		5.0f, 0.0f, 5.0f,
};


GLuint *mTexture;
unsigned int m_texture;

//float vertices []= {
//		0.0111112, -0.4000000, 0.0004904,
//		0.3005512, -0.1788860, 0.2107784,
//		-0.0994428, -0.1788860, 0.3407464,
//		-0.3466588, -0.1788860, 0.0004904,
//		-0.0994428, -0.1788860, -0.3397656,
//		0.3005512, -0.1788860, -0.2097976,
//		0.1216652, 0.1788860, 0.3407464,
//		-0.2783288, 0.1788860, 0.2107784,
//		-0.2783288, 0.1788860, -0.2097976,
//		0.1216652, 0.1788860, -0.3397656,
//		0.3688812, 0.1788860, 0.0004904,
//		0.0111112, 0.4000000, 0.0004904,
//		0.1812404, -0.3402616, 0.1240948,
//		-0.0538708, -0.3402616, 0.2004884,
//		0.1162588, -0.2102952, 0.3240948,
//		0.1812404, -0.3402616, -0.1231144,
//		0.3513704, -0.2102944, 0.0004904,
//		-0.1991808, -0.3402608, 0.0004904,
//		-0.2641644, -0.2102944, 0.2004892,
//		-0.0538708, -0.3402616, -0.1995080,
//		-0.2641644, -0.2102944, -0.1995084,
//		0.1162588, -0.2102952, -0.3231144,
//		0.3915344, -0.0000000, -0.1231148,
//		0.3915344, 0.0000000, 0.1240952,
//		0.2462256, 0.0000000, 0.3240968,
//		0.0111112, 0.0000000, 0.4004904,
//		-0.2240032, 0.0000000, 0.3240968,
//		-0.3693120, 0.0000000, 0.1240952,
//		-0.3693120, -0.0000000, -0.1231148,
//		-0.2240032, -0.0000000, -0.3231164,
//		0.0111112, -0.0000000, -0.3995096,
//		0.2462256, -0.0000000, -0.3231164,
//		0.2863868, 0.2102944, 0.2004892,
//		-0.0940364, 0.2102952, 0.3240948,
//		-0.3291480, 0.2102944, 0.0004904,
//		-0.0940364, 0.2102952, -0.3231144,
//		0.2863868, 0.2102944, -0.1995084,
//		0.2214032, 0.3402608, 0.0004904,
//		0.0760936, 0.3402616, 0.2004884,
//		-0.1590180, 0.3402616, 0.1240948,
//		-0.1590180, 0.3402616, -0.1231144,
//		0.0760936, 0.3402616, -0.1995080
//};

float verticesBox[24] = {
//		10.0f, 5.0f, -10.0f, 5.0f, -10.0f, -5.0f, 10.0f, -5.0f
		-1
};
short indicesBox  [24]  = { 0,1,2,3,
		0,1,5,4,
		0,2,5,6,
		7,3,4,1,
		7,3,6,2,
		7,4,6,5};

void init_physics();
void remove_physics();
btVector3 getRayTo(float x, float y, int heightPixels, int widthPixels);
btRigidBody* localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape);
void shootBox(const btVector3& destination);
void updateCamera();
void stepLeft(int width, int height);
void stepRight(int width, int height);
void stepFront(int width, int height);
void stepBack(int width, int height);

#ifndef BT_NO_PROFILE
class CProfileIterator* m_profileIterator = CProfileManager::Get_Iterator();
void solveTreeNode(CProfileIterator* profileIterator, int spacing);
#endif // BT_NO_PROFILE

extern "C" {
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_change(JNIEnv * env, jobject obj,  jint width, jint height);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_step(JNIEnv * env, jobject obj, const float r, const float g, const float b);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_create(JNIEnv * env, jobject obj);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_add(JNIEnv * env, jobject obj, jfloat normalizedX, jfloat normalizedY, int heightPixels, int widthPixels);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_stepLeft(JNIEnv * env, jobject obj, jint width, jint height);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_stepFront(JNIEnv * env, jobject obj, jint width, jint height);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_stepRight(JNIEnv * env, jobject obj, jint width, jint height);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_stepBack(JNIEnv * env, jobject obj, jint width, jint height);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_planeRight(JNIEnv * env, jobject obj);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_planeLeft(JNIEnv * env, jobject obj);
	JNIEXPORT void JNICALL Java_com_bullet_DemoLib_blurThick(JNIEnv * env, jobject obj, bool isChecked);
    JNIEXPORT void JNICALL Java_com_bullet_DemoLib_setTextures(JNIEnv * env, jobject obj,jintArray texture);


};

JNIEXPORT void JNICALL Java_com_bullet_DemoLib_change(JNIEnv * env, jobject obj,  jint width, jint height)
{
	static bool first = true;
	// init shoot box
	//box_body.clear();

	stepFront(width,height);
	LOGI("View Port and frustrum set.");

	if(first){
		LOGI("m_screenSpaceRenderer creation start");
		m_screenSpaceRenderer = new ScreenSpaceFluidRendererGL(m_glutScreenWidth, m_glutScreenHeight);
		basicShader = compileProgram(vBasicShader, fBasicShader);
		if(m_screenSpaceRenderer)
			LOGI("m_screenSpaceRenderer is created");
		first = false;
	}
	//updateCamera();
}

static void printGLString(const char *name, GLenum s) {
    const char *v = (const char *) glGetString(s);
    LOGI("GL %s = %s\n", name, v);
}

static void checkGlError(const char* op) {
    for (GLint error = glGetError(); error; error
            = glGetError()) {
        LOGI("after %s() glError (0x%x)\n", op, error);
    }
}

GLuint loadShader(GLenum shaderType, const char* pSource) {
    GLuint shader = glCreateShader(shaderType);
    if (shader) {
        glShaderSource(shader, 1, &pSource, NULL);
        glCompileShader(shader);
        GLint compiled = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if (!compiled) {
            GLint infoLen = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
            if (infoLen) {
                char* buf = (char*) malloc(infoLen);
                if (buf) {
                    glGetShaderInfoLog(shader, infoLen, NULL, buf);
                    LOGE("Could not compile shader %d:\n%s\n",
                            shaderType, buf);
                    free(buf);
                }
                glDeleteShader(shader);
                shader = 0;
            }
        }
    }
    return shader;
}

GLuint createProgram(const char* pVertexSource, const char* pFragmentSource) {
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER, pVertexSource);
    if (!vertexShader) {
        return 0;
    }

    GLuint pixelShader = loadShader(GL_FRAGMENT_SHADER, pFragmentSource);
    if (!pixelShader) {
        return 0;
    }

    GLuint program = glCreateProgram();
    if (program) {
        glAttachShader(program, vertexShader);
        checkGlError("glAttachShader");
        glAttachShader(program, pixelShader);
        checkGlError("glAttachShader");
        glLinkProgram(program);
        GLint linkStatus = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &linkStatus);
        if (linkStatus != GL_TRUE) {
            GLint bufLength = 0;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &bufLength);
            if (bufLength) {
                char* buf = (char*) malloc(bufLength);
                if (buf) {
                    glGetProgramInfoLog(program, bufLength, NULL, buf);
                    LOGE("Could not link program:\n%s\n", buf);
                    free(buf);
                }
            }
            glDeleteProgram(program);
            program = 0;
        }
    }
    return program;
}

GLuint gProgram;
GLuint gvPositionHandle;
GLuint gvTexCoorHandle;

bool setupGraphics() {
    printGLString("Version", GL_VERSION);
    printGLString("Vendor", GL_VENDOR);
    printGLString("Renderer", GL_RENDERER);
    printGLString("Extensions", GL_EXTENSIONS);

    gProgram = createProgram(gVertexShader, gFragmentShader);
    if (!gProgram) {
        LOGE("Could not create program.");
        return false;
    }
    gvPositionHandle = glGetAttribLocation(gProgram, "vPosition");
    checkGlError("glGetAttribLocation");
    LOGI("glGetAttribLocation(\"vPosition\") = %d\n",
            gvPositionHandle);
    gvTexCoorHandle = glGetAttribLocation(gProgram, "vTexCoords");
    checkGlError("glGetAttribLocation");
    LOGI("glGetAttribLocation(\"vTexCoords\") = %d\n",
    		gvTexCoorHandle);

    checkGlError("glViewport");
    return true;
}
inline int emitParticle(btFluidSph* fluid, const btVector3& position, const btVector3& velocity)
{
	int index = fluid->addParticle(position);
	if( index != fluid->numParticles() ) fluid->setVelocity(index, velocity);
	else
	{
		index = ( fluid->numParticles() - 1 ) * GEN_rand() / GEN_RAND_MAX;		//Random index

		fluid->setPosition(index, position);
		fluid->setVelocity(index, velocity);
	}

	return index;
}




JNIEXPORT void JNICALL Java_com_bullet_DemoLib_create(JNIEnv * env, jobject obj)
{

	LOGI("OpenGL Init finished");
	remove_physics();
	init_physics();
	LOGI("Physics initialized");
}


JNIEXPORT void JNICALL Java_com_bullet_DemoLib_add(JNIEnv * env, jobject obj, jfloat normalizedX, jfloat normalizedY, int heightPixels, int widthPixels)
{
	//	LOGI("%f%f",normalizedX,normalizedY);
	//sphere[x++]=new Sphere(m_dynamicsWorld, 0, 30, 0);
	//LOGI("%d",x);
	//getRayTo(normalizedX,normalizedY);
	shootBox(getRayTo(normalizedX,normalizedY,heightPixels,widthPixels));
}

btVector3 getCameraPosition()
{
	return m_cameraPosition;
}

btVector3	getCameraTargetPosition()
{
	return m_cameraTargetPosition;
}

btRigidBody* localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape)
{


	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic) shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

	box_body.push_back(new btRigidBody(cInfo));
	box_body[box_body.size()-1]->setContactProcessingThreshold(m_defaultContactProcessingThreshold);


	m_dynamicsWorld->addRigidBody(box_body[box_body.size()-1]);

	return box_body[box_body.size()-1];

/*

	btRigidBody* body = new btRigidBody(cInfo);
	body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);
	m_dynamicsWorld->addRigidBody(body);


	return body;

*/

}
const float STEPSIZE = 5;

void updateCamera(){

//	BT_PROFILE("updateCamera");
//	btScalar rele = m_ele * btScalar(0.01745329251994329547);// rads per deg
//	btScalar razi = m_azi * btScalar(0.01745329251994329547);// rads per deg
//
//
//	btQuaternion rot(m_cameraUp,razi);
//
//
//	btVector3 eyePos(0,0,0);
//	eyePos[m_forwardAxis] = -m_cameraDistance;
//
//	LOGI("eyepos1 is %f,%f,%f",eyePos.getX(),eyePos.getY(),eyePos.getZ());
//	btVector3 forward(eyePos[0],eyePos[1],eyePos[2]);
//	if (forward.length2() < SIMD_EPSILON)
//		not executed
//	{
//		forward.setValue(1.f,0.f,0.f);
//	}
//	btVector3 right = m_cameraUp.cross(forward);
//	btQuaternion roll(right,-rele);
//
//	eyePos = btMatrix3x3(rot) * btMatrix3x3(roll) * eyePos;
//
//	m_cameraPosition[0] = eyePos.getX();
//	m_cameraPosition[1] = eyePos.getY();
//	m_cameraPosition[2] = eyePos.getZ();
//
////	LOGI("eyepos2 is %f,%f,%f",eyePos.getX(),eyePos.getY(),eyePos.getZ());
//	m_cameraPosition += m_cameraTargetPosition;
//
//	if (m_glutScreenWidth == 0 && m_glutScreenHeight == 0)
//		return;
//
	btScalar aspect;
	btVector3 extents;

	aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
	extents.setValue(aspect * 1.0f, 1.0f,0);


	if (m_ortho)
	{
//		not executed
		extents *= m_cameraDistance;
		btVector3 lower = m_cameraTargetPosition - extents;
		btVector3 upper = m_cameraTargetPosition + extents;
//		gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
		esOrtho(&projMat,lower.getX(), upper.getX(), lower.getY(), upper.getY(),-1000,1000);
//		glOrthof(lower.getX(), upper.getX(), lower.getY(), upper.getY(),-1000,1000);
		LOGI("ORTHO");
	} else
	{
//		esFrustum(&projMat, -aspect * 1.0f, aspect * 1.0f, 1.0f, -1.0f, 1.0f,  200.0f);
//		esMatrixLookAt(&viewMat,
//				m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2],
//				m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2],
//				m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
		esFrustum(&projMat, -aspect, aspect,  -1.0f, 1.0f, 1.0f, 1000.0f);
		esMatrixLookAt(&viewMat,
						0.0f, 100.0f, 20.0f,
						0.0f, 0.0f, 0.0f,
						0.0f, 1.0f, 0.0f);
	}

}
void stepLeft(int width, int height)
{
	//LOGI("m_azi =%lf", m_azi);
	//LOGI("WIDTH = %d , HEIGHT = %d" , width, height);

	m_glutScreenWidth = width;
	m_glutScreenHeight = height;

	m_azi -= STEPSIZE; if (m_azi < 0) m_azi += 360; updateCamera();

}
void stepRight(int width, int height)
{
	m_glutScreenWidth = width;
	m_glutScreenHeight = height;

	m_azi += STEPSIZE; if (m_azi >= 360) m_azi -= 360; updateCamera();
}
void stepFront(int width, int height)
{
	m_glutScreenWidth = width;
	m_glutScreenHeight = height;

	m_ele += STEPSIZE; if (m_ele >= 360) m_ele -= 360; updateCamera();
}
void stepBack(int width, int height)
{
	m_glutScreenWidth = width;
	m_glutScreenHeight = height;

	m_ele -= STEPSIZE; if (m_ele < 0) m_ele += 360; updateCamera();
}
JNIEXPORT void JNICALL Java_com_bullet_DemoLib_setTextures(JNIEnv * env, jobject obj,jintArray texture)
{
	mTexture = (GLuint *)env->GetIntArrayElements(texture,0);
//	m_texture = (unsigned int)env->GetIntArrayElements(texture,0);//it doesnt work!!!!what the fuck dont try this anymore!!!
//    const char *v = (const char *) m_texture;
//    LOGI("GL %s = %s\n", "m_texture:", v);
//    v = (const char *) mTexture;
//    LOGI("GL %s = %s\n", "mTexture:", v);
}

JNIEXPORT void JNICALL Java_com_bullet_DemoLib_stepLeft(JNIEnv * env, jobject obj, jint width, jint height)
{
	stepLeft(width, height);
}

JNIEXPORT void JNICALL Java_com_bullet_DemoLib_stepFront(JNIEnv * env, jobject obj, jint width, jint height)
{
	stepFront(width, height);
}

JNIEXPORT void JNICALL Java_com_bullet_DemoLib_stepRight(JNIEnv * env, jobject obj, jint width, jint height)
{
	stepRight(width, height);
}

JNIEXPORT void JNICALL Java_com_bullet_DemoLib_stepBack(JNIEnv * env, jobject obj, jint width, jint height)
{
	stepBack(width, height);
}


JNIEXPORT void JNICALL Java_com_bullet_DemoLib_planeRight(JNIEnv * env, jobject obj)
{
	plane_motion_x = plane_motion_x - 1;
	plane_motion_z = plane_motion_z + 1;
}

JNIEXPORT void JNICALL Java_com_bullet_DemoLib_planeLeft(JNIEnv * env, jobject obj)
{
	plane_motion_x = plane_motion_x + 1;
	plane_motion_z = plane_motion_z - 1;

}



btVector3 getRayTo(float x, float y, int heightPixels, int widthPixels)
{
	m_glutScreenWidth = widthPixels;
	m_glutScreenHeight = heightPixels;
	if (m_ortho)
	{

		btScalar aspect;
		btVector3 extents;
		aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
		extents.setValue(aspect * 1.0f, 1.0f,0);

		extents *= m_cameraDistance;
		btVector3 lower = m_cameraTargetPosition - extents;
		btVector3 upper = m_cameraTargetPosition + extents;

		btScalar u = x / btScalar(m_glutScreenWidth);
		btScalar v = (m_glutScreenHeight - y) / btScalar(m_glutScreenHeight);

		btVector3	p(0,0,0);
		p.setValue((1.0f - u) * lower.getX() + u * upper.getX(),(1.0f - v) * lower.getY() + v * upper.getY(),m_cameraTargetPosition.getZ());
		return p;
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = btScalar(2.0) * btAtan(tanFov);

	btVector3	rayFrom = getCameraPosition();
	btVector3 rayForward = (getCameraTargetPosition()-getCameraPosition());
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 vertical = m_cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);


	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	btScalar aspect;

	aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;

	hor*=aspect;


	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/float(m_glutScreenWidth);
	btVector3 dVert = vertical * 1.f/float(m_glutScreenHeight);


	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;

	//rayTo = rayTo / 100.f;

	return rayTo;
}

void shootBox(const btVector3& destination)
{
	float mass = 1.f;
	btTransform startTransform;
	startTransform.setIdentity();
	btVector3 camPos = getCameraPosition();
	startTransform.setOrigin(camPos);



	const btScalar BOX_DIMENSIONS = 3.0f;

	btBoxShape* box = new btBoxShape( btVector3(BOX_DIMENSIONS, 0.1f, BOX_DIMENSIONS*4/3) );
	box->initializePolyhedralFeatures();
	m_shootBoxShape = box;

	btRigidBody* body = localCreateRigidBody(mass, startTransform,m_shootBoxShape);
	body->setLinearFactor(btVector3(1,1,1));
	//body->setRestitution(1);

	btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
	linVel.normalize();
	linVel*=m_ShootBoxInitialSpeed;

	body->getWorldTransform().setOrigin(camPos);
	body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
	body->setLinearVelocity(linVel);
	body->setAngularVelocity(btVector3(0,0,0));
	body->setCcdMotionThreshold(0.5);
	body->setCcdSweptSphereRadius(0.4f);//value should be smaller (embedded) than the half extends of the box (see ::setShootBoxShape)
	LOGI("shootBox uid=%d\n", body->getBroadphaseHandle()->getUid());
	LOGI("camPos=%f,%f,%f\n",camPos.getX(),camPos.getY(),camPos.getZ());
	LOGI("destination=%f,%f,%f\n",destination.getX(),destination.getY(),destination.getZ());

}

void getFluidColors(bool drawFluidsWithMultipleColors, int fluidIndex, btFluidSph* fluid, int particleIndex, float* out_r, float* out_g, float* out_b)
{
	const float COLOR_R = 1.0f;
	const float COLOR_G = 1.0f;
	const float COLOR_B = 1.0f;

	if(!drawFluidsWithMultipleColors)
	{
		float brightness = fluid->getVelocity(particleIndex).length() * 2.0f;
		if(brightness < 0.f)brightness = 0.f;
		if(brightness > 1.f)brightness = 1.f;
		const float MIN_BRIGHTNESS(0.45f);
		brightness = brightness * (1.0f - MIN_BRIGHTNESS) + MIN_BRIGHTNESS;

		*out_r = COLOR_R * brightness;
		*out_g = COLOR_G * brightness;
		*out_b = COLOR_B * brightness;
	}
	else
	{
		*out_r = COLOR_R;
		*out_g = COLOR_G;
		*out_b = COLOR_B;

		if(fluidIndex % 2)
		{
			*out_r = 1.0f - COLOR_R;
			*out_g = 1.0f - COLOR_G;
			*out_b = 1.0f - COLOR_B;
		}
	}
}

//////////////////////Bullet Physics code////////////////////////////////
void init_physics(){

	m_collisionConfiguration = new btFluidRigidCollisionConfiguration();

	//'standard' Bullet configuration
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();
	m_solver = new btSequentialImpulseConstraintSolver();
	m_fluidSolverCPU = new btFluidSphSolverDefault();
	m_dynamicsWorld = new btFluidRigidDynamicsWorld(m_dispatcher, m_broadphase,
			m_solver, m_collisionConfiguration, m_fluidSolverCPU);

	m_fluidWorld = static_cast<btFluidRigidDynamicsWorld*>(m_dynamicsWorld);

	m_fluidWorld->setGravity(btVector3(0.0 ,-9.8, 0.0));
	m_fluidWorld->applyGravity();

	btCollisionShape* groundShape = new btBoxShape( btVector3(100.0, 0.0, 100.0) );
	m_collisionShapes.push_back(groundShape);


	plane = new Plane(m_fluidWorld, groundShape,  0, 0, 0);

	///////////////////// Demo_Drop/////////////////////////////
	const btScalar VOL_BOUND = 52.0f;
	btVector3 volMin(-VOL_BOUND, -10.0f, -VOL_BOUND);
	btVector3 volMax(VOL_BOUND, VOL_BOUND * 2.0f, VOL_BOUND);
	{
		btFluidSph* fluid;

		fluid = new btFluidSph(m_fluidWorld->getGlobalParameters(),
				MIN_FLUID_PARTICLES);
		btFluidSphParametersLocal FL = fluid->getLocalParameters();
		FL.setDefaultParameters();
		FL.m_aabbBoundaryMin = volMin;
		FL.m_aabbBoundaryMax = volMax;
		FL.m_particleRadius = 0.8f;
		FL.m_enableAabbBoundary = 1;
		FL.m_restDensity *= 3.0f;
		FL.m_sphParticleMass *= 2.0f;
		FL.m_stiffness /= 3.0f;

		fluid->setLocalParameters(FL);
		fluid->setGridCellSize(m_fluidWorld->getGlobalParameters() );
		m_fluids.push_back(fluid);

		// Also create an emitter
	//*
		{
			btFluidEmitter* emitter = new btFluidEmitter();
			m_emitter = emitter;

			emitter->m_fluid = fluid;

			const btScalar OFFSET(1.0);
			emitter->m_positions.push_back(btVector3(0, 0, 0));
			emitter->m_positions.push_back(btVector3(0, OFFSET, 0));
			emitter->m_positions.push_back(btVector3(0, -OFFSET, 0));
			emitter->m_positions.push_back(btVector3(OFFSET, 0, 0));
			emitter->m_positions.push_back(btVector3(-OFFSET, 0, 0));

			emitter->m_speed = 1.f;

			emitter->m_center.setValue(12.f, 2.f, 12.f);
			emitter->m_rotation.setEuler(90, -45, 0);
		//
			const btScalar INIT_BOUND = 50.0f;
			btVector3 initMin(-INIT_BOUND, 0.f, 0.f);
			btVector3 initMax(INIT_BOUND, 45.f, INIT_BOUND);
			emitter->addVolume(fluid, initMin, initMax, fluid->getEmitterSpacing(m_fluidWorld->getGlobalParameters()) * 0.87 );
		//
			m_fluidWorld->addSphEmitter(emitter);
		}
		//*/
//*/
		for (int i = 0; i < m_fluids.size(); ++i) {
			const bool ENABLE_CCD = true;
			if (ENABLE_CCD)
				m_fluids[i]->setCcdMotionThreshold(
						m_fluids[i]->getLocalParameters().m_particleRadius);

			m_fluidWorld->addFluidSph(m_fluids[i]);
		}

	}


}

void renderFluids(const float r, const float g, const float b)
{
	bool drawFluidsWithMultipleColors = false;
//	LOGI("size is %d",m_fluids.size());
	for(int i = 0; i < m_fluids.size(); ++i)
	{
		const btFluidSphParametersLocal& FL = m_fluids[i]->getLocalParameters();
		btScalar particleRadius = FL.m_particleRadius;

		//Beer's law constants
		//Controls the darkening of the fluid's color based on its thickness
		//For a constant k, (k > 1) == darkens faster; (k < 1) == darkens slower; (k == 0) == disable
		float _r = r, _g = g, _b = b;

		float absorptionR = 0.;
		float absorptionG = 0.;
		float absorptionB = 0.;

		if(drawFluidsWithMultipleColors)
		{
			absorptionR = 1.0;
			absorptionG = 1.0;
			absorptionB = 1.0;
		}
		//*
		m_screenSpaceRenderer->render(m_fluids[i]->internalGetParticles().m_pos, particleRadius * 1.5f,
				_r, _g, _b, absorptionR, absorptionG, absorptionB, &projMat, &viewMat);
	}
	//*/
}
/*
void setShootBoxShape ()
{
	if (!m_shootBoxShape)
	{
		const btScalar BOX_DIMENSIONS = 3.0f;

		btBoxShape* box = new btBoxShape( btVector3(BOX_DIMENSIONS, 0.1f, BOX_DIMENSIONS*4/3) );
		box->initializePolyhedralFeatures();
		m_shootBoxShape = box;
	}
}*/
/*
void renderShootBox()
{
	//	LOGI("RenderShootBox.");
/*
	for(int i = 0; i < box_body.size(); ++i) {
		glPushMatrix();
		btTransform trans;
		box_body[i]->getMotionState()->getWorldTransform(trans);
		float matrix[16];
		trans.getOpenGLMatrix(matrix);
		glMultMatrixf(matrix);
		glEnableClientState(GL_VERTEX_ARRAY);
		//		glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
		glVertexPointer(3, GL_FLOAT, 0, verticesBox);
		//		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[1]);
		glColor4f(255.0f, 0.0f, 0.0f, 255.0f);
		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[0]);
		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[4]);
		glColor4f(0.0f, 0.0f, 255.0f, 255.0f);
		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[8]);
		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[12]);
		glColor4f(0.0f, 255.0f, 0.0f, 255.0f);
		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[16]);
		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[20]);
		glDisableClientState(GL_VERTEX_ARRAY);
		glPopMatrix();

	}
//
}
*/
const GLfloat gTexCoor[] = { 0, 0, 0, 1, 1, 1, 1, 0 };
void renderShootBox(float x, float y, float z, GLuint program, ESMatrix* pMat, ESMatrix* vMat){
//*
	for(int i = 0; i < box_body.size(); ++i) {
		btTransform trans;

		box_body[i]->getMotionState()->getWorldTransform(trans);
		trans.setRotation(btQuaternion(btVector3(0,0,1),SIMD_PI*0.01*x));
		setupGraphics();

		float matrix[16] ;
		trans.getOpenGLMatrix(matrix);
		ESMatrix mvMat;
		ESMatrix mvpMat;
		esMatrixMultiply(&mvMat, (ESMatrix*)matrix, vMat);
		esMatrixMultiply(&mvpMat, &mvMat, pMat);

		glFrontFace(GL_CW);


//		glEnableVertexAttribArray(0);

		checkGlError("glClear");

		glUseProgram(gProgram);
		glUniformMatrix4fv( glGetUniformLocation(gProgram, "mvpMat"), 1, false, (GLfloat*)&mvpMat );

//		glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, gTriangleVertices);
		glVertexAttribPointer(gvPositionHandle, 2, GL_FLOAT, GL_FALSE, 0, verticesBox);
		glVertexAttribPointer(gvTexCoorHandle, 2, GL_FLOAT, GL_FALSE, 0, gTexCoor);
		glEnableVertexAttribArray(gvPositionHandle);
		glEnableVertexAttribArray(gvTexCoorHandle);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D,mTexture[0]);

		glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
//		glUniform2f( glGetUniformLocation(gProgram, "vTexCoords"), 0.0f, 0.0f);
//		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[8]);
//		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[0]);
//		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[4]);
//		glUniform4f( glGetUniformLocation(program, "_Color"), 0.0f, 0.0f, 255.0f, 255.0f);
//		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[8]);
//		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[12]);
//		glUniform4f( glGetUniformLocation(program, "_Color"), 0.0f, 255.0f, 0.0f, 255.0f);
//		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[16]);
//		glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, &indicesBox[20]);
		glUseProgram(0);
	}
//*/
}

void SwapBuffers()
{
	//glSwapBuffers(glDisplay, glSurface);
}

void remove_physics(){
	if(plane)
		delete plane;
	if(m_shootBoxShape)
		delete m_shootBoxShape;
	if(m_fluidSolverCPU)
		delete m_fluidSolverCPU;
	if(m_emitter)
		delete m_emitter;
	if(m_fluidWorld)
		delete m_fluidWorld;
	if(m_collisionConfiguration)
		delete m_collisionConfiguration;
	if(m_solver)
		delete m_solver;
	if(m_dispatcher)
		delete m_dispatcher;
	if(m_broadphase)
		delete m_broadphase;
	m_fluids.clear();
	m_collisionShapes.clear();
	m_demos.clear();
}

#ifndef BT_NO_PROFILE
void showAllProfileInfo()
{
	static double time_since_reset = 0.f;
	if (!m_idle)
	{
		time_since_reset = CProfileManager::Get_Time_Since_Reset();
	}

	{
		//recompute profiling data, and store profile strings
		m_profileIterator->First();
		solveTreeNode(m_profileIterator,0);
	}
}

#endif //BT_NO_PROFILE

#ifndef BT_NO_PROFILE
void solveTreeNode(CProfileIterator* profileIterator, int spacing)
{

	profileIterator->First();
	if (profileIterator->Is_Done())
		return;

	float accumulated_time=0,parent_time = profileIterator->Is_Root() ? CProfileManager::Get_Time_Since_Reset() : profileIterator->Get_Current_Parent_Total_Time();
	int i;
	int frames_since_reset = CProfileManager::Get_Frame_Count_Since_Reset();
	LOGD("%s,%.3f",	profileIterator->Get_Current_Parent_Name(), parent_time );
	float totalTime = 0.f;


	int numChildren = 0;

	for (i = 0; !profileIterator->Is_Done(); i++,profileIterator->Next())
	{
		numChildren++;
		float current_total_time = profileIterator->Get_Current_Total_Time();
		accumulated_time += current_total_time;
		float fraction = parent_time > SIMD_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
		{
			int i;	for (i=0;i<spacing;i++)	printf(".");
		}
		LOGD("%s,%s,%.2f,%.3f,%d",profileIterator->Get_Current_Parent_Name(),profileIterator->Get_Current_Name(), fraction,(current_total_time / (double)frames_since_reset), profileIterator->Get_Current_Total_Calls());
		totalTime += current_total_time;
		//recurse into children
	}

	if (parent_time < accumulated_time)
	{
		LOGD("what's_wrong\n");
	}
	for (i=0;i<spacing;i++)	printf(".");
	//LOGD("%s (%.3f %%) :: %.3f ms\n", "Unaccounted:",parent_time > SIMD_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);

	for (i=0;i<numChildren;i++)
	{
		profileIterator->Enter_Child(i);
		solveTreeNode(profileIterator,spacing+3);
		profileIterator->Enter_Parent();
	}
}

#endif //BT_NO_PROFILE


int stepCounter = 0;

void draw(const float r, const float g, const float b)
{
#ifndef BT_NO_PROFILE
	CProfileManager::Reset();
#endif //BT_NO_PROFILE

	BT_PROFILE("draw()");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glViewport ( 0, 0, m_glutScreenWidth, m_glutScreenHeight );
	btScalar aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;

	m_fluidWorld->stepSimulation(btScalar(1.0/60.0) ,1,  btScalar(1.0/60.0));
	m_fluidWorld->setFluidSolver(m_fluidSolverCPU);
//	if(m_fluidWorld) m_fluidWorld->debugDrawWorld();		//Optional but useful: debug drawing to detect problems

	updateCamera();
	esMatrixLoadIdentity(&projMat);
	esMatrixLoadIdentity(&viewMat);
	esFrustum(&projMat, -aspect, aspect,  -1.0f, 1.0f, 1.0f, 1000.0f);
	esMatrixLookAt(&viewMat,
					0.0f, 100.0f, 20.0f,
					0.0f, 0.0f, 0.0f,
					0.0f, 1.0f, 0.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDisable(GL_BLEND);
//	plane->draw(plane_motion_x, -10, plane_motion_z, basicShader,&projMat,&viewMat);

	renderShootBox(plane_motion_x, -10, plane_motion_z, basicShader,&projMat,&viewMat);
	renderFluids(r, g, b);
	glUseProgram(0);
}



JNIEXPORT void JNICALL Java_com_bullet_DemoLib_step(JNIEnv * env, jobject obj, const float r, const float g, const float b)
{
	struct timeval start1;
	gettimeofday(&start1, NULL);
	long int totalTime = start1.tv_sec*1000+start1.tv_usec/1000;
//	LOGI("draw time is %ld",totalTime);
	draw(r, g, b);

#ifndef BT_NO_PROFILE
	stepCounter++;
	if(stepCounter > 100){
		showAllProfileInfo();
		stepCounter = 0;
	}
#endif //BT_NO_PROFILE

}

JNIEXPORT void JNICALL Java_com_bullet_DemoLib_blurThick(JNIEnv * env, jobject obj, bool isChecked) {
	m_screenSpaceRenderer->setCheckedBlurThick(isChecked);
}


