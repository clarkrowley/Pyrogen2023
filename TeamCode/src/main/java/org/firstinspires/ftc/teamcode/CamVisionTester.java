
package org.firstinspires.ftc.teamcode;
import static org.opencv.imgproc.Imgproc.drawContours;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Timer;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */
@TeleOp(name="CamVisionTester", group="Robot")
@Disabled
public class CamVisionTester extends LinearOpMode
{
    TargetColor lastColor = TargetColor.NULL;
    ComputerVision computerVision = new ComputerVision();
    @Override
    public void runOpMode() {
        computerVision.init(hardwareMap);
        waitForStart();
        computerVision.start(this,telemetry);

        double start = getRuntime();
        double t = getRuntime();

        while(!isStopRequested()) {
            if(getRuntime() - t > 1) {
                if (computerVision.color != lastColor) {
                    telemetry.speak(computerVision.color.toString());
                    lastColor = computerVision.getColor();
                }
                t = getRuntime();
            }
            telemetry.addLine("Color was: " + computerVision.color.toString());
            telemetry.update();
        }
    }



}
