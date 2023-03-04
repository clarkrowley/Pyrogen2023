/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



public class RobotHardware {

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor right_back   = null;
    public DcMotor right_front = null;
    public DcMotor left_back = null;
    public DcMotor left_front = null;
    public DcMotor left_vertical = null;
    public DcMotor right_vertical = null;
    public DcMotor horizontal = null;

    private double MAXSPEED = 1;
    public Servo gripper = null;
    private double gripper_start = 0.387;
    public DcMotor elevator = null;
    public BNO055IMU imu = null;

    public double gripperOpenPos = 0.387;
    public double gripperClosePos = 0.17;
    public boolean gripperClosed = true;
    public Boolean gripperTogglePressed = false;

    public double elevatorUpPower = 1;
    public double elevatorDownPower = -.5;
    public int elevatorTargetPosition = 0;
    public int elevatorTopPosition = 4200;
    public int elevatorBottomPosition = 10;

    // Odometry
    public static double mmPerFoot = 304.8;
    public static double ticksPerRev = 8192;
    public static double FeetPerRev =  54. * Math.PI / mmPerFoot;

    public void init(HardwareMap hardwareMap)    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        right_back  = hardwareMap.get(DcMotor.class, "right_back");
        left_back =   hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_front =  hardwareMap.get(DcMotor.class, "left_front");

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        //elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorTargetPosition = 0;

        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(gripper_start);
        left_vertical = hardwareMap.get(DcMotor.class, "left_vertical");
        right_vertical =   hardwareMap.get(DcMotor.class, "right_vertical");
        horizontal = hardwareMap.get(DcMotor.class, "horizontal");

        //  back right was correct, reverse the rest
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public void runDrivePolar(double speed, double angle, double rotate) {
        double goAngle = angle - Math.PI / 4.;
        double v1 = speed * Math.cos(goAngle) + rotate;
        double v2 = speed * Math.sin(goAngle) - rotate;
        double v3 = speed * Math.sin(goAngle) + rotate;
        double v4 = speed * Math.cos(goAngle) - rotate;
        left_front.setPower(v1);
        right_front.setPower(v2);
        left_back.setPower(v3);
        right_back.setPower(v4);
    }

    public void runDrivePolarJoy(double leftX, double leftY, double rightX) {
        double speed = Math.hypot(leftX, leftY);
        double angle = Math.atan2(-leftY, leftX);
        double rotate = rightX;
        runDrivePolar(speed * MAXSPEED, angle, rotate);
    }

    public void runDriveTankJoy(double leftY, double rightY){
        left_front.setPower(leftY);
        right_front.setPower(rightY);
        left_back.setPower(leftY);
        right_back.setPower(rightY);
    }

    public void toggleGripper(Gamepad gamepad, Telemetry telem){
        if (!gamepad.right_bumper && gripperTogglePressed) {
            gripperTogglePressed = false;
            gripperClosed = ! gripperClosed;
        } else {
            if (gamepad.right_bumper) {
                gripperTogglePressed = true;
            }
        }
        if (gripperClosed) {
            gripper.setPosition(gripperClosePos);
        } else {
            gripper.setPosition(gripperOpenPos);
        }
        telem.addData("gripperClosed:",gripperClosed);
        telem.update();
    }

    public void moveGripper(double leftY, Telemetry telem){
        double val = remap(-1*leftY,-1,0,1,1);
        telem.addData("val:",val);
        telem.update();
        gripper.setPosition(val);
    }

    public void runStraightSlow(boolean forward, double seconds,
                                LinearOpMode opmode, ElapsedTime runtime) {
        opmode = opmode;
        double speed = 0.;
        double rotate = 0.;
        double angle = Math.PI / 2.;
        if (!forward) {
            angle = -Math.PI / 2.;
        }
        noencoder();
        speed = 0.2;
        runtime.reset();
        while (opmode.opModeIsActive() &&
                (runtime.seconds() < seconds)) {
            runDrivePolar(speed, angle, rotate);
        }
        stop();
    }

    public void rotate(double speed) {
        speed *= -1;
        double v1 = speed * +1;
        double v2 = speed * -1;
        double v3 = speed * +1;
        double v4 = speed * -1;
        left_front.setPower(v1);
        right_front.setPower(v2);
        left_back.setPower(v3);
        right_back.setPower(v4);
    }

    public void strafeToPosition(double targetPos, LinearOpMode opMode, double minSpeed, double maxSpeed) {
        resetEncoders();
        double errorThresh = 1024;
        double pos = 0;
        double error = 9999;
        double p = 0;
        targetPos = targetPos * 1.45;
        while (opMode.opModeIsActive() && Math.abs(error) > errorThresh) {
            pos = horizontal.getCurrentPosition();
            error = targetPos - pos;
            p = error / 8192;
            if (p < 0) {
                p = PyroUtil.clamp(p, -1 * maxSpeed, -1 * minSpeed);
                runDrivePolar(Math.abs(p),Math.PI,0);
            } else if (p > 0) {
                p = PyroUtil.clamp(p, minSpeed, maxSpeed);
                runDrivePolar(p,0,0);
            }
            //runDrivePolarJoy(0., -1.*p, 0.);

            opMode.telemetry.addData("Speed: " , p);
            opMode.telemetry.addData("LF: " , left_front.getCurrentPosition());
            opMode.telemetry.addData("RF: " , right_front.getCurrentPosition());
            opMode.telemetry.addData("LV: " , left_vertical.getCurrentPosition());
            opMode.telemetry.addData("RV: " , right_vertical.getCurrentPosition());
            opMode.telemetry.addData("targetPos",targetPos);
            opMode.telemetry.addData("pos",pos);
            opMode.telemetry.addData("error " , error);
            opMode.telemetry.update();
        }
        stop();
    }

    public void driveToPosition(double targetPos, LinearOpMode opMode, double minSpeed, double maxSpeed) {
        resetEncoders();
        double errorThresh = 1024;
        double pos = 0;
        double error = 9999;
        double p = 0;
        while (opMode.opModeIsActive() && Math.abs(error) > errorThresh) {
            pos = -1.* (left_vertical.getCurrentPosition() + right_vertical.getCurrentPosition()) /2;
            error = targetPos - pos;
            p = error / 8192;
            if (p < 0) {
                p = PyroUtil.clamp(-1 * maxSpeed, -1 * minSpeed, p);
            } else if (p > 0) {
                p = PyroUtil.clamp(minSpeed, maxSpeed, p);
            }
            //runDrivePolarJoy(0., -1.*p, 0.);
            runDrivePolar(p,Math.PI/2.,0);

            opMode.telemetry.addData("Speed: " , p);
            opMode.telemetry.addData("LF: " , left_front.getCurrentPosition());
            opMode.telemetry.addData("RF: " , right_front.getCurrentPosition());
            opMode.telemetry.addData("LV: " , left_vertical.getCurrentPosition());
            opMode.telemetry.addData("RV: " , right_vertical.getCurrentPosition());
            opMode.telemetry.addData("targetPos",targetPos);
            opMode.telemetry.addData("pos",pos);
            opMode.telemetry.addData("error " , error);
            opMode.telemetry.update();
        }
        stop();
    }

    public void noencoder() {
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void resetEncoders() {
        left_vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stop() {
        left_front.setPower(0.);
        right_front.setPower(0.);
        left_back.setPower(0.);
        right_back.setPower(0.);
    }

    public void runElevatorWithEncoder(boolean up, float down, Telemetry telem) {
        int d = elevatorTargetPosition;
        if (up) {
            d = d + 15;
            d = Math.min(d,elevatorTopPosition);
        }
        if (down > .5) {
            d = d - 15;
            d = Math.max(d, elevatorBottomPosition);
        }
        elevator.setTargetPosition(d);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(1.);
        elevatorTargetPosition = d;
        telem.addData("elev target:",d);
        d = elevator.getCurrentPosition();
        telem.addData("elev position:",d);
        return;
    }
    public void resetElevatorEncoder(){
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runElevator(boolean up, float down, Telemetry telem) {
        if (up) {
            elevator.setPower(elevatorUpPower);
            return;
        }
        if (down > .5) {
            elevator.setPower(elevatorDownPower);
            return;
        }
        elevator.setPower(0.);
        return;
    }

    //public void runElevator(double inputValue, Telemetry telem) {elevator.setPower(inputValue);}

    double remap(double val, double l1, double l2, double h1, double h2) {
        return l2 + (val - l1) * (h2 - l2) / (h1 - l1);
    }

}






