package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="AutoApril", group="Linear Opmode")
public class AutoApril extends LinearOpMode {
    RobotHardware robot;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of Sleeve
    int leftTagID = 15;
    int centerTagID = 17;
    int rightTagID = 19;
    boolean seeLeftTag = false;
    boolean seeCenterTag = false;
    boolean seeRightTag = false;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                seeLeftTag = false;
                seeCenterTag = false;
                seeRightTag = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == leftTagID )
                    {
                        tagOfInterest = tag;
                        seeLeftTag = true;
                        break;
                    }
                    if(tag.id == centerTagID )
                    {
                        tagOfInterest = tag;
                        seeCenterTag = true;
                        break;
                    }
                    if(tag.id == rightTagID )
                    {
                        tagOfInterest = tag;
                        seeRightTag = true;
                        break;
                    }
                }

                if(seeLeftTag)
                {
                    telemetry.addLine("Left Tag is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else if (seeCenterTag) {
                    telemetry.addLine("Center Tag is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else if (seeRightTag) {
                    telemetry.addLine("Right Tag is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see a tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen  tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        robot.gripper.setPosition(robot.gripperClosePos);
        sleep(500);
        robot.elevator.setTargetPosition(150);
        robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elevator.setPower(1.);
        sleep(500);

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == centerTagID)
        {
            autoCenter();
        }
        else if (tagOfInterest.id == leftTagID) {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */
            autoLeft();
        } else if (tagOfInterest.id == rightTagID) {
            autoRight();
        }
    }

    void autoCenter() {
        telemetry.speak("Green");
        double feet = 3.5;
        // Drive forward
        double distanceTicks = RobotHardware.ticksPerRev / RobotHardware.FeetPerRev * feet;
        robot.driveToPosition(distanceTicks,this,.2,0.5);
    }
    void autoLeft() {
        telemetry.speak("Cyan");
        double feet;
        // Drive forward
        feet = -2;
        double distanceTicks = RobotHardware.ticksPerRev / RobotHardware.FeetPerRev * feet;
        robot.strafeToPosition(distanceTicks,this,.2,0.5);
        feet = 4;
        distanceTicks = RobotHardware.ticksPerRev / RobotHardware.FeetPerRev * feet;
        robot.driveToPosition(distanceTicks,this,.2,0.5);
    }
    void autoRight() {
        telemetry.speak("Pink");
        double feet;
        // Drive forward
        feet = 2;
        double distanceTicks = RobotHardware.ticksPerRev / RobotHardware.FeetPerRev * feet;
        robot.strafeToPosition(distanceTicks,this,.2,0.5);
        feet = 3.5;
        distanceTicks = RobotHardware.ticksPerRev / RobotHardware.FeetPerRev * feet;
        robot.driveToPosition(distanceTicks,this,.2,0.5);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}