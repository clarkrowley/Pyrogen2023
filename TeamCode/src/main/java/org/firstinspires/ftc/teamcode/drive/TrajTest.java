package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class TrajTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive  drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(
                                          new Pose2d(0., 0, Math.toRadians(0)))
                .forward(4)
                .turn(Math.toRadians(90))
                .strafeRight(45.5)
                .forward(26)
                .back(50)
                .turn(Math.toRadians(45))
                .forward(7)
                .back(7)
                .turn(Math.toRadians(45))
                .strafeRight(46)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(myTrajectory);
    }
}
