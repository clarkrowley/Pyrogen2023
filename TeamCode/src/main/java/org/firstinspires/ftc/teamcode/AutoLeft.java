/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoLeft extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();
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

    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    //Tag IDs of Sleeve
    int leftTagID = 15;
    int centerTagID = 17;
    int rightTagID = 19;
    boolean seeLeftTag = false;
    boolean seeCenterTag = false;
    boolean seeRightTag = false;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.gripper.setPosition(robot.gripperClosePos);
        double m_turn;
        double m_drive;

        if (leftside()) {
            m_turn = -48.5;
            m_drive = 55.5;
        } else {
            m_turn = 50.;
            m_drive = 52;
        }
        TrajectorySequence leftTrajectory1 = drive.trajectorySequenceBuilder(
                new Pose2d(0., 0, Math.toRadians(0)))
                //.forward(4)
                //.turn(Math.toRadians(-90))
                //.strafeLeft(47.7) //was 47.5
                .forward(m_drive)
                .back(2)
                .turn(Math.toRadians(m_turn))
                .build();

        //TrajectorySequence leftTrajectory2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
        //        .forward(9)
        //        .build();
        if (leftside()) {
            m_drive = 7.5;
        } else {
            m_drive = 12.;
        }
        Trajectory leftTrajectory2 = drive.trajectoryBuilder(leftTrajectory1.end()).forward(m_drive).build();

        if (leftside()) {
            m_turn = -42.5;
        } else {
            m_turn = 40.;
        }
        TrajectorySequence leftTrajectory3 = drive.trajectorySequenceBuilder(leftTrajectory2.end()).back(10.).turn(Math.toRadians(m_turn)).build();

        TrajectorySequence leftTrajectoryLeft;
        TrajectorySequence leftTrajectoryRight;

        if (leftside()) {
             leftTrajectoryLeft = drive.trajectorySequenceBuilder(leftTrajectory3.end()).strafeRight(2).back(24).build();
             leftTrajectoryRight = drive.trajectorySequenceBuilder(leftTrajectory3.end()).strafeRight(2).forward(27).build();
        } else {
             leftTrajectoryLeft = drive.trajectorySequenceBuilder(leftTrajectory3.end()).strafeLeft(1).forward(27).build();
             leftTrajectoryRight = drive.trajectorySequenceBuilder(leftTrajectory3.end()).strafeLeft(1).back(22).build();
        }

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

        //waitForStart();
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
        robot.elevator.setTargetPosition(robot.elevatorBottomPosition + 100);
        robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elevator.setPower(1.);

        drive.followTrajectorySequence(leftTrajectory1);

        robot.elevator.setTargetPosition(robot.elevatorTopPosition);
        sleep(3000);

        //  drive.followTrajectorySequence(leftTrajectory2);
        drive.followTrajectory(leftTrajectory2);

        //  Lower cone
        robot.elevator.setTargetPosition(robot.elevatorTopPosition - 500);
        sleep(1500);

        robot.gripper.setPosition(robot.gripperOpenPos);
        sleep(500);
        robot.elevator.setTargetPosition(robot.elevatorBottomPosition);

        drive.followTrajectorySequence(leftTrajectory3);


        if(tagOfInterest == null || tagOfInterest.id == centerTagID)
        {

        }
        else if (tagOfInterest.id == leftTagID) {

            drive.followTrajectorySequence(leftTrajectoryLeft);

        } else if (tagOfInterest.id == rightTagID) {

            drive.followTrajectorySequence(leftTrajectoryRight);

        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    boolean leftside() {
        return true;
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
