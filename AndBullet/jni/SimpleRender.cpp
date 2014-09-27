#include "SimpleRender.h"

//From texture.cpp
GLuint create_empty_texture();
GLuint create_water_texture();

void SimpleRender::draw(GLuint program, ESMatrix* pMat, ESMatrix* vMat, GLsizei SCREEN_WIDTH, GLsizei SCREEN_HEIGHT) {
	GLuint framebuffer = create_empty_texture();
	GLuint water_texture = create_water_texture();

	glClear (GL_COLOR_BUFFER_BIT);

	//Draw fuild here
	glBindTexture(GL_TEXTURE_2D, water_texture);
	glBlendFunc(GL_ONE, GL_ONE);//Additive rendering
//	for(std::list<fluid_c>::const_iterator i = fluid.begin(); i != fluid.end(); i++) i->draw();

	//Copy the screen into a texture
	glBindTexture(GL_TEXTURE_2D, framebuffer);
	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0);

	//Draw a transparent black rectangle over the whole screen
	//Instead of just clearing the screen we now leave the original (non-metaball) image slightly visible to produce glow
	glDisable (GL_TEXTURE_2D);
	glColor4f(0.0, 0.0, 0.0, 0.7);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable (GL_TEXTURE_2D);

	//Redraw the copied screen and produce metaballs
	//Metaballs are produced simply by discarding pixels that don't have enough alpha
	glEnable (GL_ALPHA_TEST);
	glAlphaFunc(GL_GREATER, 0.85); //Discard treshold
	glColor4f(1.0, 1.0, 1.0, 1.0);
	glBlendFunc(GL_SRC_ALPHA, GL_ZERO);//Discard current screen pixels of pixels that are redrawn
	draw_fullscreen_rectangle();
	glDisable (GL_ALPHA_TEST);
}
