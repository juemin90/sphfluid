package com.bullet;

import java.io.InputStream;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;


import android.app.AlertDialog;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView.Renderer;
import android.opengl.GLU;
import android.opengl.GLUtils;
import android.util.Log;

public class BulletRenderer implements Renderer{
	private final Context context;
	GL10 gl;
	int width, height;
	boolean init = true;
	public BulletRenderer(Context context) {
        this.context = context;
    }
	public void onDrawFrame(GL10 gl) {
		
        DemoLib.step(1.0f, 1.0f, 1.0f);
    }

    public void onSurfaceChanged(GL10 gl, int width, int height) {
    	this.width = width;
    	this.height = height;
		DemoLib.change(width, height);
    }

    int textureId;
    private int[] TextureString = new int[1];
    
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        this.gl = gl;
        Bitmap bitmap = getBitmap(context,R.drawable.lg);
        if(bitmap != null)
        {
        	gl.glEnable(GLES20.GL_TEXTURE_2D);
            gl.glGenTextures(1, TextureString,0);
            textureId= TextureString[0];
            Log.e("textureId", String.valueOf(textureId));
            gl.glBindTexture(GLES20.GL_TEXTURE_2D, textureId);
            gl.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_NEAREST);
            gl.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
            gl.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, 
            		GLES20.GL_CLAMP_TO_EDGE); 
            gl.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, 
            		GLES20.GL_CLAMP_TO_EDGE); 
            GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0);
            DemoLib.setTextures(TextureString);
            bitmap.recycle();
        }
        DemoLib.create();
    } 
    private Bitmap getBitmap(Context context,int resId)
    {
    	BitmapFactory.Options options = new BitmapFactory.Options();
    	options.inScaled = false;
    	return BitmapFactory.decodeResource(context.getResources(), resId,options);
    
    }
    
	public void handleTouchPress(float normalizedX, float normalizedY, int heightPixels, int widthPixels){
    	DemoLib.add(normalizedX,normalizedY,heightPixels,widthPixels);
    	
    }
	
	public void stepLeft(){
		DemoLib.stepLeft(this.width, this.height);
		
	}
	public void stepFront(){
		DemoLib.stepFront(this.width, this.height);
		
	}
	
	public void stepBack(){
		DemoLib.stepBack(this.width, this.height);
		
	}
	public void stepRight(){
		DemoLib.stepRight(this.width, this.height);
		
	}
	
	public void planeRight(){
		DemoLib.planeRight();
	}

	public void planeLeft(){
		DemoLib.planeLeft();
	}
	public void blurThick(boolean isChecked) {
		DemoLib.blurThick(isChecked);
	}
}
