package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import java.util.Locale;

public class PyroGyro {
    RobotHardware robot ;

    public float target = 0;
    Telemetry telemetry = null;
    Orientation angles;
    Acceleration gravity;
    double zAccumulated;  //Total rotation left/right
    private double TURNSPEED = 0.7;
    double rotate = 0.35;

    /* Constructor */
    public PyroGyro(RobotHardware robot){
        this.robot = robot;
    }

    /* Initialize standard Hardware interfaces */
    public void init(Telemetry telemetry) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        robot.imu.initialize(parameters);
        target = getHeadingFloat();
        this.telemetry = telemetry;

        telemetry.addData("Calibration state", robot.imu.isGyroCalibrated());
        telemetry.update();
    }


    public double getHeading() {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // return Math.round(angles.firstAngle);
        return angles.firstAngle;
    }

    public float getHeadingFloat() {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // return Math.round(angles.firstAngle);
        return angles.firstAngle;
    }


    public double getError(double target) {
        double robotError = target - (double) getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    //private ElapsedTime runtime = new ElapsedTime();

    //This function turns a number of degrees compared to where the robot was when the program started.
    //Positive numbers turn left.
    public void turnAbsolute(int target, ElapsedTime runtime, double timelimit,Telemetry telemetry) {
        robot.left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        zAccumulated = getHeading();

        while (runtime.seconds() < timelimit && Math.abs(zAccumulated - target) > 3 ) {
            rotate = TURNSPEED * Math.max(0.1, Math.min(Math.abs(zAccumulated - target) / 20., 1.)) ;

            if (zAccumulated > target) {  // we need to turn right
                robot.left_front.setPower(rotate);
                robot.right_front.setPower(-rotate);
                robot.left_back.setPower(rotate);
                robot.right_back.setPower(-rotate);
            }

            if (zAccumulated <= target) {  // we need to turn left
                robot.left_front.setPower(-rotate);
                robot.right_front.setPower(rotate);
                robot.left_back.setPower(-rotate);
                robot.right_back.setPower(rotate);
            }

            zAccumulated = getHeading();
            telemetry.addData("current heading", "%03f", (float)zAccumulated);
            telemetry.update();
        }

        robot.left_front.setPower(0);
        robot.right_front.setPower(0);
        robot.left_back.setPower(0);
        robot.right_back.setPower(0);
    }

    public double getHeadingRadians() {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (double) angles.firstAngle;
    }


    void turnToAngle(double angle, double minSpeed, double maxSpeed, int maxTime, OpMode opMode) {
        double ANGLE_ERROR_THRESHOLD = 1;
        robot.resetEncoders();
        ElapsedTime runTime = new ElapsedTime();

        double angleError = getError(angle);
        double angleOutput = maxSpeed;
        double angularVelocity = 100;
        //angleOutput = PyroUtil.clamp(angleError, -0.5, 0.5);
        while ((Math.abs(angleError) > ANGLE_ERROR_THRESHOLD || angularVelocity > 0.001) && runTime.seconds() < maxTime) {
            angleError =  getError(angle);
            if(Math.abs(angleError) < 90) {
                angleOutput = (maxSpeed + minSpeed) / 2;
            } else if(Math.abs(angleError) < 45) {
                angleOutput = (maxSpeed * minSpeed) / 4;
            } else if(Math.abs(angleError) < 15) {
                angleOutput = minSpeed;
            }
            angleOutput = Range.clip(angleOutput, minSpeed, maxSpeed);
            /*
            //  angleOutput
            if (angleError >= 0) {
                robot.rotate(angleOutput * -1);
            } else {
                robot.rotate(angleOutput );
            }
            */
            robot.runDrivePolarJoy(0,0,angleOutput);

            opMode.telemetry.addData("Time:",runTime.time());
            opMode.telemetry.addData("Error:" , angleError);
            opMode.telemetry.addData("Output:" , angleOutput);
            opMode.telemetry.addData("Turning to angle:",angle);
            opMode.telemetry.addData("Heading",(float)getHeading());
            opMode.telemetry.addData("Angular velocity",angularVelocity);
            opMode. telemetry.update();

        }

        robot.stop();
    }




}
