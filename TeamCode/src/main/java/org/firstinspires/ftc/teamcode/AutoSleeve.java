
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="AutoSleeve", group="Linear Opmode")
public class AutoSleeve extends LinearOpMode {
    RobotHardware robot;
    ComputerVision vision;


    @Override
    public void runOpMode() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        vision = new ComputerVision();
        vision.init(hardwareMap);
        vision.start(this,telemetry);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // 304.8mm = 1 ft
        waitForStart();
        vision.reset();
        telemetry.addData("Vision:", "Starting camera capture");
        telemetry.update();

        TargetColor color = TargetColor.NULL; // = vision.getColor();

    /*
        double ticksPerRev = 8192;
        double FeetPerRev =  54. * Math.PI / mmPerFoot;
        double feet = 3.;
        double distanceTicks = ticksPerRev / FeetPerRev * feet;
        feet=-2.;
        distanceTicks = ticksPerRev / FeetPerRev * feet;
        robot.strafeToPosition(distanceTicks,this,.2,1.0);
        sleep(1000);
        feet=3.;
        distanceTicks = ticksPerRev / FeetPerRev * feet;
        robot.driveToPosition(distanceTicks,this,.2,1.0);
        sleep(250);

     */

        double startTime = getRuntime();
        int count = 0;
        while(opModeIsActive() && getRuntime() < startTime + 10) {

            telemetry.addData("Vision:", "Cyan: " + vision.cyanCount);
            telemetry.addData("Vision:", "Pink: " + vision.pinkCount);
            telemetry.addData("Vision:", "Green:" + vision.greenCount);

            telemetry.update();
            color = vision.getColor();
            count++;

        }

        robot.gripper.setPosition(robot.gripperClosePos);
        sleep(500);
        robot.elevator.setTargetPosition(150);
        robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elevator.setPower(1.);
        sleep(500);

        switch(color) {
            case PINK:
                autoPink();
                break;
            case GREEN:
                autoGreen();
                break;
            default:
                autoCyan();
                break;
        }

        vision.phoneCam.stopRecordingPipeline();
        vision.stop();
    }

    void autoGreen() {
        telemetry.speak("Green");
        double feet = 3.5;
        // Drive forward
        double distanceTicks = RobotHardware.ticksPerRev / RobotHardware.FeetPerRev * feet;
        robot.driveToPosition(distanceTicks,this,.2,0.5);
    }
    void autoCyan() {
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
    void autoPink() {
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
}
