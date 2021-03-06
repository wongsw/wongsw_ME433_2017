package com.example.songwei.sw_imageprocessing;

// libraries

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.TextView;
import android.widget.SeekBar;

import java.io.IOException;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;
import static android.hardware.Camera.Parameters.FOCUS_MODE_AUTO;

import android.widget.SeekBar.OnSeekBarChangeListener;

public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView fpsTextview;
    SeekBar RbarControl, TbarControl;
    TextView myTitle;

    static long prevtime = 0; // for FPS calculation
    int Rvalue = 0; //The difference between Green and Red pixel values (to find color Grey)
    int Tvalue = 0; //How dark/light you want grey to be



    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off


        myTitle = (TextView) findViewById(R.id.title);
        fpsTextview = (TextView) findViewById(R.id.fpsStatus);
        RbarControl = (SeekBar) findViewById(R.id.seek1);
        TbarControl = (SeekBar) findViewById(R.id.seek2);
        //RbarText = (TextView) findViewById(R.id.bar1Status);
        //TbarText = (TextView) findViewById(R.id.bar2Status);
        setMyControlListener(); // calls function to update progress bar

        // see if the app has permission to use the camera
        ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, 1);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
            mSurfaceHolder = mSurfaceView.getHolder();

            mTextureView = (TextureView) findViewById(R.id.textureview);
            mTextureView.setSurfaceTextureListener(this);

            // set the paintbrush for writing text on the image
            paint1.setColor(0xffff0000); // red
            paint1.setTextSize(24);

            fpsTextview.setText("started camera");
        } else {
            fpsTextview.setText("no camera permissions");
        }

    }

    private void setMyControlListener() {
        RbarControl.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = 0;
                //RbarText.setText("R value is: "+progress);
                Rvalue = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
        TbarControl.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
               //TbarText.setText("T value is: "+progress);
                Tvalue = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        parameters.setAutoExposureLock(true); // keep the white balance constant
        //parameters.setWhiteBalance(FOCUS_MODE_AUTO);
        //parameters.setExposureCompensation();
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode


        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // every time there is a new Camera preview frame
        mTextureView.getBitmap(bmp);

        final Canvas c = mSurfaceHolder.lockCanvas();
        if (c != null) {

            int[] pixels = new int[bmp.getWidth()]; // pixels[] is the RGBA data
            for (int j = 0; j < bmp.getHeight(); j+=10){ // to read each row of the frame
                int startY = j; // which row in the bitmap to analyze to read
                bmp.getPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);

                int COM = 0; // center of masses (where software thinks middle of line is)
                int sum_mr = 0; // the sum of the mass times the radius
                int sum_m = 0; // the sum of the masses
                for (int i = 0; i < bmp.getWidth(); i++) {
                    if (((green(pixels[i]) - red(pixels[i])) > -Rvalue) && ((green(pixels[i]) - red(pixels[i])) < Rvalue) && (green(pixels[i]) > Tvalue)) {
                        pixels[i] = rgb(1, 1, 1); // set the pixel to almost 100% black
                        sum_m = sum_m + green(pixels[i]) + red(pixels[i]) + blue(pixels[i]);
                        sum_mr = sum_mr + (green(pixels[i]) + red(pixels[i]) + blue(pixels[i])) * i;
                    }
                }
                // only use the data if there were a few pixels identified, otherwise you might get a divide by 0 error
                if (sum_m > 5) {
                    COM = sum_mr / sum_m;
                } else {
                    COM = 0;
                }
                // update the row
                bmp.setPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);
                // draw a circle where COM is
                canvas.drawCircle(COM, startY, 3, paint1); // x position, y position, diameter, color
                // write the pos as text
                canvas.drawText("R = " + Rvalue, 10, 30, paint1);
                canvas.drawText("T = " + Tvalue, 10, 60, paint1);
            }
        }
        c.drawBitmap(bmp, 0, 0, null);
        mSurfaceHolder.unlockCanvasAndPost(c);

        // calculate the FPS to see how fast the code is running
        long nowtime = System.currentTimeMillis();
        long diff = nowtime - prevtime;
        fpsTextview.setText("FPS " + 1000 / diff);
        prevtime = nowtime;
    }
}
