package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;


import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;


public class CameraTest {
    private static final String TAG = "BasicImageDetector";

    //  Camera stuff
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;
    private EvictingBlockingQueue<Bitmap> frameQueue;
    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    private android.os.Handler callbackHandler;
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    Telemetry telemetry;
    public Bitmap bmp;

    public CameraTest(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        telemetry.addLine("Initting BasicImageDetector");
        telemetry.update();
        callbackHandler = CallbackLooper.getDefault().getHandler();
        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        initializeFrameQueue(2);
        telemetry.setAutoClear(false);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        openCamera();
        if (camera == null) {
            telemetry.addLine("Camera is null!>");
            telemetry.update();
        }

        startCamera();
        if (cameraCaptureSession == null)  {
            telemetry.addLine("CamCaptureSession is null!>");
            telemetry.update();
        }
    }

    public void grabPicture() {

        try {

            telemetry.addLine("Starting");
            telemetry.update();
            bmp = null;
            telemetry.addLine("Polling camera");
            telemetry.update();
            while(bmp == null) {

                bmp = frameQueue.poll();


            }
            telemetry.addLine("Polling done");
            telemetry.update();
            if(bmp == null) {

                telemetry.addLine("We did not get a bitmap!!!!");
                telemetry.update();
            }

        } finally {
            //telemetry.addLine("Closing camera.");
            //telemetry.update();
            closeCamera();
        }


    }

/*
    public POSITION detectPosition() {
        grabPicture();

        for(int rowIdx = 0; rowIdx < rowStrideMax; rowIdx++) {
            // Consider every stride of pixels
            for(int colIdx = 0; colIdx <  colStrideMax; colIdx++) {
                //  Get the average of the stride
                //for(int yStride = 0; yStride < colStrideMax; yStride++) {
                int x = colIdx * stride;
                int y = rowIdx * stride;

                bmp.getPixels(pixelBuf, 0,stride, x, y, stride, stride);

                float avgGreen = getAvgGreen(pixelBuf);

                if(avgGreen > colorThresh) {
                    //telemetry.addLine("Setting green pixels at " + x + ", y" + y);

                    // telemetry.update();
                    bmp.setPixels(greenBuf, 0, stride, x, y, stride, stride);
                    greenBins[colIdx]++;
                } else{

                    //bmp.setPixels(blackBuf, 0, stride, x, y, stride, stride);
                }
            }
        }
        //int avgCol = colCount / colStrideMax;
        int bestBin = 0;
        int bestBinCount = 0;
        for(int i = 0; i < greenBins.length; i++) {
            if(greenBins[i] > bestBinCount) {
                bestBinCount = greenBins[i];
                bestBin = i;
            }
        }
        // Draw a white diagnostic line
        for(int i = 0; i < bmp.getHeight(); i++) {
            bmp.setPixel(bestBin*stride, i, Color.WHITE);
        }
        // Discard the best b in if the # of matching green cells is less than avgMin.
        if(greenBins[bestBin] < avgMin) {
            bestBin = 0;
        }

        POSITION pos = POSITION.LEFT;
        telemetry.addLine("DetectedLinePos: " + bestBin*stride);
        if(bestBin*stride > middleThresh) {
            if(bestBin*stride > rightThresh) {
                pos = POSITION.RIGHT;
            } else {
                pos = POSITION.MIDDLE;
            }
        } else {
            pos = POSITION.LEFT;
        }
        // telemetry.addLine("Best bin: " + bestBin + " value: " + greenBins[bestBin]);
        // telemetry.addLine("Position " + pos);
        //  telemetry.update();
        saveBitmap(bmp);
        return pos;
    }
    */


    public void saveBitmap(Bitmap bitmap) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.png", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                  telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()");
            error("exception saving %s", file.getName());
        }
    }

    private void initializeFrameQueue(int capacity) {

        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                //   frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }



    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera() {

        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }


    private void openCamera() {
        telemetry.addLine("Opening camrea.");
        telemetry.update();
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        //
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);

        //  if (camera == null) {
        //      error("camera not found or permission to use not granted: %s", cameraName);
        // }
        telemetry.addLine("Camera now open..");
        telemetry.update();
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent


        final int imageFormat = ImageFormat.YUY2;


        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);


        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {

            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(CameraCaptureSession session) {
                    try {

                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame(CameraCaptureSession session,CameraCaptureRequest request, CameraFrame cameraFrame) {

                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted(CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException |RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            error("exception starting camera");
            synchronizer.finish(null);
        }


        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }


        cameraCaptureSession = synchronizer.getValue();
    }


    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }
    private void error(String format, Object...args) {
        telemetry.log().add(format, args);
        telemetry.update();
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

    /*
    public float getAvgGreen(int[] pixels) {
        float cumulGreen = 0;
        float avg = 0;
        for (int i = 0; i < pixels.length; i++) {
            int color = pixels[i];
            int A = (color >> 24) & 0xff; // or color >>> 24
            int R = (color >> 16) & 0xff;
            int G = (color >> 8) & 0xff;
            int B = (color) & 0xff;
            float[] hsv = new float[3];
            Color.RGBToHSV(R, G, B, hsv);
            if(R > redThresh || B > blueThresh) {
                G = 0;
            }
            cumulGreen += G;
        }
        return cumulGreen/pixels.length;
    }
*/


}
