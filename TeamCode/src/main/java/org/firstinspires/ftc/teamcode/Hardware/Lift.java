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

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auton.AprilTagDetectionPipeline;
import org.opencv.core.Mat;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 */

public class Lift {

    /* Declare OpMode members. */
    private OpMode myOpMode;   // gain access to methods in the calling OpMode.
    private ElapsedTime runtime = new ElapsedTime();

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally - idk abt that one)
    private DcMotor upMotorL;
    private DcMotor upMotorR;

    private Servo claw;
    private Servo flipL;
    private Servo flipR;
    public static double FLIP_CONSTANT = 0.9;
    public Servo rotate;
    public static double ROTATE_CONSTANT = 0.84;
    public static int minHeightForFlip = 2053;
    public static double LIFTMOTORPOWER = 1.0;

    public static int groundInch = 0;
    public static int lowInch = 16;
    public static int midInch = 25;
    public static int highInch = 35;
    public static int maxHeightInch = 40;
    public static int liftCountsPerInch = 83;
    public static int[] heightsCounts = new int[] {groundInch * liftCountsPerInch, highInch * liftCountsPerInch, lowInch * liftCountsPerInch, midInch * liftCountsPerInch};
    public static String[] heightNames = new String[] {"Floor", "High Terminal", "Low Terminal", "Medium Terminal"};
    public static int maxHeightCounts = maxHeightInch * liftCountsPerInch;

    public TouchSensor upLSensor;
    public TouchSensor upRSensor;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Lift(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init(boolean calibrate)    {
        upMotorL = myOpMode.hardwareMap.get(DcMotor.class, "upMotorL");
        upMotorR = myOpMode.hardwareMap.get(DcMotor.class, "upMotorR");

        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        flipL = myOpMode.hardwareMap.get(Servo.class, "flipL");
        flipR = myOpMode.hardwareMap.get(Servo.class, "flipR");
        rotate = myOpMode.hardwareMap.get(Servo.class, "rotate");

        upLSensor = myOpMode.hardwareMap.get(TouchSensor.class, "upLSensor");
        upRSensor = myOpMode.hardwareMap.get(TouchSensor.class, "upRSensor");

        upMotorL.setDirection(DcMotor.Direction.FORWARD);
        upMotorR.setDirection(DcMotor.Direction.REVERSE);

        upMotorL.setPower(0);
        upMotorR.setPower(0);

        claw.setPosition(1);
        flipL.setPosition(1);
        flipR.setPosition(0);
        rotate.setPosition(1);

        upMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        upMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

        if(calibrate)
            calibrateLift();
    }

    public void calibrateLift(){
        myOpMode.telemetry.addData(">", "Lift Calibrating");
        upMotorL.setPower(-0.2);
        upMotorR.setPower(-0.2);

        while(!upLSensor.isPressed() && !upRSensor.isPressed()) {}
        upMotorL.setPower(0);
        upMotorR.setPower(0);

        if (!upLSensor.isPressed() || !upRSensor.isPressed()) {
            myOpMode.telemetry.addData(">", "Lift Calibration Failed, Check Lift Alignment");
        }
        else {
            myOpMode.telemetry.addData(">", "Lift Calibration Success.");
        }
        myOpMode.telemetry.update();

        stopAndResetLiftEncoders();
        upMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the lift height to the given count number (floor = 0, ground = 660, low = 4950, middle = 7590, high = 10560)
     * @param counts represents how high the lift should go, 330 counts = 1 inch
     * @param liftPower how fast the lift should
     */
    public void setLift(int counts, double liftPower) {
        upMotorL.setTargetPosition(counts);
        upMotorR.setTargetPosition(counts);

        try {
            upMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            upMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        catch(Exception e){
            myOpMode.telemetry.addData("Motors failing", "use manual adjustments I guess");
            myOpMode.telemetry.update();
        }

        upMotorL.setPower(liftPower);
        upMotorR.setPower(liftPower);
    }

    public void setLift(int counts, double liftPower, double timeoutS) {
        runtime.reset();

        upMotorL.setTargetPosition(counts);
        upMotorR.setTargetPosition(counts);

        upMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        upMotorL.setPower(liftPower);
        upMotorR.setPower(liftPower);

        while(upMotorL.isBusy() && upMotorR.isBusy() && runtime.seconds() < timeoutS){
            // stay in the code
        }
    }

    public void downDropUp(int height){
//        setDrivePower(0,0,0,0);
        setLift(height - 373,LIFTMOTORPOWER,1);
        claw.setPosition(0); // open
        try {
            sleep(150);
        } catch (InterruptedException e) {
            myOpMode.telemetry.addData("Try-Catch Failed", "Oops");
            myOpMode.telemetry.update();
        }
        setLift(height + 51,LIFTMOTORPOWER,1);
    }

    public void stopAndResetLiftEncoders() {
        upMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Flips claw to a specific point
     * @param pos pos = 0 represents initialization state, pos = 1 represents flipped state
     */
    public void flipToPosition(double pos) {
        flipL.setPosition(1 - pos);
        flipR.setPosition(pos);
    }

    public void closeClaw(){
        claw.setPosition(0);
    }
    public void openClaw(){
        claw.setPosition(1);
    }
}
