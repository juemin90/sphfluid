#include "Plane.h"

short Plane::indices [] = {
		0, 1, 2,
		2, 3, 0
};
float Plane::vertices []= {
		100.0f, 0.0f, 100.0f,
		-100.0f, 0.0f, 100.0f,
		-100.0f, 0.0f, -100.0f,
		100.0f, 0.0f, -100.0f,
};


Plane::Plane(btDiscreteDynamicsWorld* dynamicsWorld,btCollisionShape* shape1, float x, float y ,float z){
	shape = shape1;

	btVector3 localInertia(0,0,0);
	shape->calculateLocalInertia(0,localInertia);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin( btVector3(0,-10,0) );
	groundTransform.setRotation(btQuaternion(btVector3(0,0,1),SIMD_PI*0.045));

	motionState = new btDefaultMotionState(groundTransform);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(0.f,motionState,shape,localInertia);
	body = new btRigidBody(rbInfo);


	this->world = dynamicsWorld;
	this->world->addRigidBody(body);
}

Plane::~Plane(){
	world->removeRigidBody(body);
	delete motionState;
	delete body;
	delete shape;
}

void Plane::draw(float x, float y, float z, GLuint program, ESMatrix* pMat, ESMatrix* vMat){
//*
	btTransform trans;


	//world->removeRigidBody(body);
	motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,y,0)));
	body->setMotionState(motionState);
	//body->setAngularVelocity(btVector3(x,0,z));
	body->getMotionState()->getWorldTransform(trans);
	trans.setRotation(btQuaternion(btVector3(0,0,1),SIMD_PI*0.01*x));

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin( btVector3(0,-10,0) );
	groundTransform.setRotation(btQuaternion(btVector3(0,0,1),SIMD_PI*0.01 * x));
	//delete motionState;
	motionState = new btDefaultMotionState(groundTransform);
	body->setMotionState(motionState);

	float matrix[16] ;
	groundTransform.getOpenGLMatrix(matrix);
	ESMatrix mvMat;
	ESMatrix mvpMat;
	esMatrixMultiply(&mvMat, (ESMatrix*)matrix, vMat);
	esMatrixMultiply(&mvpMat, &mvMat, pMat);

	glUseProgram(program);
	glUniformMatrix4fv( glGetUniformLocation(program, "mvpMat"), 1, false, (GLfloat*)&mvpMat );
	glUniform4f( glGetUniformLocation(program, "_Color"), 0.8f, 0.8f, 0.8f, 0.0f);

	glFrontFace(GL_CW);
	glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, vertices);
	glEnableVertexAttribArray(0);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, &indices[0]);
	glFrontFace(GL_CCW);

	glUseProgram(0);
//*/
}
