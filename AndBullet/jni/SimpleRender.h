/*
 * SimpleRender.h
 *
 *  Created on: 2014-7-1
 *      Author: kumcun
 */

#ifndef SIMPLERENDER_H_
#define SIMPLERENDER_H_

#include "ScreenSpaceFluidRendererGL.h"

class SimpleRender {

private:

public:
	void draw(GLuint program, ESMatrix* pMat, ESMatrix* vMat, GLsizei SCREEN_WIDTH, GLsizei SCREEN_HEIGHT);
};

#endif /* SIMPLERENDER_H_ */
