
package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import static org.opencv.imgproc.Imgproc.drawContours;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */
@Disabled
enum TargetColor { NULL, GREEN, PINK, CYAN }


public class ComputerVision
{
    OpenCvCamera phoneCam;
    WebcamName camName;
    int contourSize = 0;
    int convexHullCount = 0;
    public TargetColor color = TargetColor.NULL;
    int greenCount = 0;
    int pinkCount = 0;
    int cyanCount = 0;

    public void init(HardwareMap h){
        camName = h.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = h.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", h.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(camName,cameraMonitorViewId);

    }

    public void stop() {
        phoneCam.stopRecordingPipeline();
        phoneCam.stopStreaming();

    }

    public void reset() {
        greenCount = 0;
        pinkCount = 0;
        cyanCount = 0;
    }
    public void start(OpMode opmode, Telemetry telemetry)
    {
        // phoneCam.setPipeline(new CupPipeline());
        OpenCvCamera.AsyncCameraOpenListener listener = new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
               //phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                phoneCam.setPipeline(new SamplePipeline());

                phoneCam.startStreaming(640,480);

            }

            @Override
            public void onError(int errorCode) {

            }
        };
        phoneCam.openCameraDeviceAsync(listener);

    }

    public TargetColor getColor() {
        if(greenCount > cyanCount && greenCount > pinkCount) {
            return TargetColor.GREEN;
        }
        if(cyanCount > greenCount && cyanCount > pinkCount) {
            return TargetColor.CYAN;
        }
        if (pinkCount > greenCount && pinkCount > cyanCount){
            return TargetColor.PINK;
        }
        return TargetColor.NULL;
    }


    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;
        GreenPipeline greenPipe = new GreenPipeline();
        PinkPipeline pinkPipe = new PinkPipeline();
        CyanPipeline cyanPipe = new CyanPipeline();

        @Override
        public Mat processFrame(Mat input)
        {
            Mat output = input;
            Imgproc.cvtColor(input, output, Imgproc.COLOR_BGR2RGB);

            greenPipe.process(input);
            if(greenPipe.convexHullsOutput() != null) {
                if (greenPipe.convexHullsOutput().size() > 0) {
                    color = TargetColor.GREEN;
                    greenCount = greenCount + 1;
                }
            }



            cyanPipe.process(input);
            if(cyanPipe.convexHullsOutput() != null) {
                if (cyanPipe.convexHullsOutput().size() > 0) {
                    color = TargetColor.CYAN;
                    cyanCount = cyanCount + 1;
                }
            }

            pinkPipe.process(input);
            if(pinkPipe.convexHullsOutput() != null) {
                if (pinkPipe.convexHullsOutput().size() > 0) {
                    color = TargetColor.PINK;
                    pinkCount += 1;

                }
            }

            return output;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                phoneCam.pauseViewport();
            }
            else
            {
                phoneCam.resumeViewport();
            }
        }
    }

}
