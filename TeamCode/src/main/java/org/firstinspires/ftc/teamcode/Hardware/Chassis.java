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

package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

/**
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 */

public class Chassis {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally - idk abt that one)
    public DcMotor fLMotor, fRMotor, bLMotor, bRMotor;

    private List<DcMotor> motors;

    public TouchSensor RSensor;
    public TouchSensor LSensor;

    private IMU imu;

    private OpMode myOpMode;

    static final double FEET_PER_METER = 3.28084;

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public Chassis(OpMode opMode, boolean auton) {

        myOpMode = opMode;

        fLMotor = myOpMode.hardwareMap.get(DcMotor.class, "fLMotor");
        fRMotor = myOpMode.hardwareMap.get(DcMotor.class, "fRMotor");
        bLMotor = myOpMode.hardwareMap.get(DcMotor.class, "bLMotor");
        bRMotor = myOpMode.hardwareMap.get(DcMotor.class, "bRMotor");

        motors = Arrays.asList(fLMotor, bLMotor, bRMotor, fRMotor);

        fRMotor.setDirection(DcMotor.Direction.REVERSE);
        bRMotor.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (auton) {
            RSensor = myOpMode.hardwareMap.get(TouchSensor.class, "RSensor");
            LSensor = myOpMode.hardwareMap.get(TouchSensor.class, "LSensor");

            IMU.Parameters parameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );
            imu = myOpMode.hardwareMap.get(IMU.class, "imu");
            imu.initialize(parameters);
        }
    }

    public void mecanumMove(double left_stick_x, double left_stick_y, double right_stick_x, double speedMultiplier)
    {
        //variables
        setDriveModeEncoder();
        double r = Math.hypot(left_stick_x, left_stick_y);
        double robotAngle = Math.atan2(left_stick_y, -left_stick_x) - Math.PI / 4;
        double rightX = right_stick_x;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        setDrivePower(v1 * speedMultiplier, v2 * speedMultiplier, v3 * speedMultiplier, v4 * speedMultiplier);

        myOpMode.telemetry.update();
    }

    // Pass the requested wheel motor powers to the appropriate hardware drive motors.
    public void setDrivePower(double frontLeft, double frontRight, double backLeft, double backRight) {
        // Output the values to the motor drives.
        fLMotor.setPower(frontLeft);
        fRMotor.setPower(frontRight);
        bLMotor.setPower(backLeft);
        bRMotor.setPower(backRight);
    }
    public boolean hitWall(){
        return RSensor.isPressed() || LSensor.isPressed();
    }
    public void stopAndResetDriveEncoders() {
        fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setDriveModeEncoder() {
        fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setDriveModeRTP() {
        fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean motorsBusy() {
        return fLMotor.isBusy() || fRMotor.isBusy() || bRMotor.isBusy() || bLMotor.isBusy();
    }
}
