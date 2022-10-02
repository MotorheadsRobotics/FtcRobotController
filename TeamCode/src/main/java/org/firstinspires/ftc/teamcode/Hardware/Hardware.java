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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 */

public class Hardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    private ElapsedTime runtime = new ElapsedTime();

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally - idk abt that one)
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;

    public DcMotor upMotorL;
    public DcMotor upMotorR;
    public DcMotor horMotor;

    public Servo claw;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double      COUNTS_PER_INCH = 8080;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        fLMotor = myOpMode.hardwareMap.get(DcMotor.class, "fLMotor");
        fRMotor = myOpMode.hardwareMap.get(DcMotor.class, "fRMotor");
        bLMotor = myOpMode.hardwareMap.get(DcMotor.class, "bLMotor");
        bRMotor = myOpMode.hardwareMap.get(DcMotor.class, "bRMotor");

        fLMotor.setDirection(DcMotor.Direction.FORWARD);
        fRMotor.setDirection(DcMotor.Direction.REVERSE);
        bLMotor.setDirection(DcMotor.Direction.FORWARD);
        bRMotor.setDirection(DcMotor.Direction.REVERSE);

        fLMotor.setPower(0);
        bLMotor.setPower(0);
        fRMotor.setPower(0);
        bRMotor.setPower(0);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
         fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /** Non-drive Motors **/
        upMotorL = myOpMode.hardwareMap.get(DcMotor.class, "upMotorL");
        upMotorR = myOpMode.hardwareMap.get(DcMotor.class, "upMotorR");
        horMotor = myOpMode.hardwareMap.get(DcMotor.class, "horMotor");

        claw = myOpMode.hardwareMap.get(Servo.class, "claw");

        upMotorL.setDirection(DcMotor.Direction.REVERSE);
        upMotorR.setDirection(DcMotor.Direction.FORWARD);
        horMotor.setDirection(DcMotor.Direction.FORWARD);

        upMotorL.setPower(0);
        upMotorR.setPower(0);
        horMotor.setPower(0);

        claw.setPosition(0);

        upMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    public void mecanumMove(double left_stick_x, double left_stick_y, double right_stick_x)
    {
        //variables
        double r = Math.hypot(-left_stick_x, left_stick_y);
        double robotAngle = Math.atan2(left_stick_y, -left_stick_x) - Math.PI / 4;
        double rightX = -right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        setDrivePower(v1, v2, v3, v4);

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
    public void stopAndResetDriveEncoders() {
        fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
