/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.Auton.AprilTagImageRecognition.FEET_PER_METER;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


public abstract class AutonDriving extends LinearOpMode {
    // Initialize Hardware
    Hardware robot = new Hardware(this);
    ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;       // from GoBuilda
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;         // Gearing up (more speed, less torque) --> ratio < 1.0
    static final double     WHEEL_DIAMETER_INCHES   = 3.77952756 ;  // 96mm
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     COUNTS_PER_DEGREE       = 1;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    static final double     HEADING_THRESHOLD      = 1;

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;


    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  fLSpeed       = 0;
    private double  fRSpeed       = 0;
    private double  bLSpeed       = 0;
    private double  bRSpeed       = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    /**
     * Method to drive at any given angle for a certain number of degrees. Maintains heading.
     * @param speed desired magnitude speed for the robot
     * @param direction forwards = 0, right = 90, backwards = 180, left = 270
     * @param inches you know what this means
     * @param timeoutS how many seconds until the robot gives up on this task
     */
    public void encoderDrive(double speed, double direction, double inches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        robot.stopAndResetDriveEncoders();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                robot.fLMotor.getCurrentPosition(),
                robot.fRMotor.getCurrentPosition(),
                robot.bLMotor.getCurrentPosition(),
                robot.bRMotor.getCurrentPosition());
        telemetry.update();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            /** Direction: forwards = 0, right = 90, back = 180, left = 270 **/
            // direction -= 90; direction *= -1; direction *= Math.PI/180;
            direction = (90 - direction) * Math.PI / 180 - Math.PI / 4;
            double v1 = inches * Math.cos(direction);
            double v2 = inches * Math.sin(direction);
            double v3 = inches * Math.sin(direction);
            double v4 = inches * Math.cos(direction);

            newFrontLeftTarget = robot.fLMotor.getCurrentPosition() + (int)(v1 * COUNTS_PER_INCH);
            newFrontRightTarget = robot.fRMotor.getCurrentPosition() + (int)(v2 * COUNTS_PER_INCH);
            newBackLeftTarget = robot.bLMotor.getCurrentPosition() + (int)(v3 * COUNTS_PER_INCH);
            newBackRightTarget = robot.bRMotor.getCurrentPosition() + (int)(v4 * COUNTS_PER_INCH);

            robot.fLMotor.setTargetPosition(newFrontLeftTarget);
            robot.fRMotor.setTargetPosition(newFrontRightTarget);
            robot.bLMotor.setTargetPosition(newBackLeftTarget);
            robot.bRMotor.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            double forwardSlashSpeed = Math.abs(speed) * Math.cos(direction);
            double backwardsSlashSpeed = Math.abs(speed) * Math.sin(direction);
            robot.fLMotor.setPower(forwardSlashSpeed);
            robot.fRMotor.setPower(backwardsSlashSpeed);
            robot.bLMotor.setPower(backwardsSlashSpeed);
            robot.bRMotor.setPower(forwardSlashSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.fLMotor.isBusy() || robot.fRMotor.isBusy() || robot.bLMotor.isBusy() || robot.bRMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }

    public AprilTagDetection getTag(AprilTagDetectionPipeline pipeline) {
        AprilTagDetection tagOfInterest = null;
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 1 || tag.id == 2 || tag.id == 3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
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
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
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
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        return tagOfInterest;
    }

    public void squarePathSignal(AprilTagDetection tagOfInterest){
        if(tagOfInterest.id == 1){
            encoderDrive(0.5, 90, 45,2);
            encoderDrive(0.5, 270, 4, 2);
            encoderDrive(0.5, 0,34,3);
        }
        else if(tagOfInterest.id == 2){
            encoderDrive(0.5, 90,
                    50,2);
        }
        else if(tagOfInterest.id == 3){
            encoderDrive(0.5, 90, 50,2);
            encoderDrive(0.5, 270, 4, 2);
            encoderDrive(0.5, 180,34,3);
        }
    }
    /**
     *
     * @param speed desired speed at which wheels turn
     * @param leftInches how many inches the left-side wheels turn
     * @param rightInches same for right-side wheels
     * @param timeoutS seconds until robot gives up on life
     */
    public void encoderDriveSimple(double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS) {
        int newfLeftTarget;
        int newfRightTarget;
        int newbLeftTarget;
        int newbRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfLeftTarget = robot.fLMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfRightTarget = robot.fRMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newbLeftTarget = robot.bLMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbRightTarget = robot.bRMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.fLMotor.setTargetPosition(newfLeftTarget);
            robot.fRMotor.setTargetPosition(newfRightTarget);
            robot.bLMotor.setTargetPosition(newbLeftTarget);
            robot.bRMotor.setTargetPosition(newbRightTarget);

            // Turn On RUN_TO_POSITION
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.fRMotor.setPower(Math.abs(speed));
            robot.fLMotor.setPower(Math.abs(speed));
            robot.bRMotor.setPower(Math.abs(speed));
            robot.bLMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.fRMotor.isBusy() && robot.bLMotor.isBusy() && robot.fLMotor.isBusy() && robot.bRMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newfLeftTarget,  newfRightTarget, newbLeftTarget, newbRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    /**
     * gyroStrafeDrive(Autonomous)
     * @param speed maximum speed of travel, the ideal speed the robot should move at
     * @param distance distance robot needs to move...
     * @param heading in this ideal heading.
     * this function should have the robot strafe given the three parameters above, and should
     * actively maintain its heading in this process.
     */
    public void gyroStrafeDrive(double speed, double heading, double distance) {
        speed = Math.abs(speed); // whether speed is positive or negative, it works.
        double holdHeading = robot.getRawHeading(); // what heading we need to hold to is defined by what direction robot is facing right now.
        encoderDriveNoWaiting(speed, heading, distance); // set encoder drive running
        double fLSpeedCurrent;
        double fRSpeedCurrent;
        double bLSpeedCurrent;
        double bRSpeedCurrent;
        while (opModeIsActive() && robot.bRMotor.isBusy() && robot.bLMotor.isBusy() && robot.fRMotor.isBusy() && robot.fLMotor.isBusy()){
            turnSpeed = getSteeringCorrection(holdHeading,P_DRIVE_GAIN); // get steering correction with respect to our hold heading
            if (distance < 0) {
                turnSpeed *= -1.0; // apparently if your distance is negative you need opposite turning... I'm not really gonna question this.
            }
            fLSpeedCurrent = fLSpeed - turnSpeed; // motor base speeds returned by encoderDrive are modified by turnspeed
            bLSpeedCurrent = bLSpeed - turnSpeed; // new variables avoid changing static base values
            fRSpeedCurrent = fRSpeed + turnSpeed;
            bRSpeedCurrent = bRSpeed + turnSpeed;
            double max = Math.max(Math.max(fLSpeedCurrent, bLSpeedCurrent), Math.max(fRSpeedCurrent, bRSpeedCurrent));
            if (max > 1.0)
            {
                // if highest speed is above 1, scale them all down
                fLSpeedCurrent /= max;
                bLSpeedCurrent /= max;
                fRSpeedCurrent /= max;
                bRSpeedCurrent /= max;
            }
            robot.fLMotor.setPower(fLSpeedCurrent);
            robot.bLMotor.setPower(bLSpeedCurrent);
            robot.fRMotor.setPower(fRSpeedCurrent);
            robot.bRMotor.setPower(bRSpeedCurrent); // variable power.

            telemetry.addData("Heading Target: ", holdHeading);
            telemetry.addData("Current Heading: ", turnSpeed);
            telemetry.update();
        }
    }
    /**
     * setLift (Autonomous), waits for
     * @param counts
     * @param liftPower
     */
    public void setLift(int counts, double liftPower, double timeoutS) {
        runtime.reset();
        robot.upMotorL.setTargetPosition(counts);
        robot.upMotorR.setTargetPosition(counts);

        robot.upMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.upMotorL.setPower(liftPower);
        robot.upMotorR.setPower(liftPower);

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.upMotorL.isBusy() && robot.upMotorR.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to",  " %7d", counts);
            telemetry.addData("Currently at",  " at %7d :%7d",
                    robot.upMotorL.getCurrentPosition(), robot.upMotorR.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     * turnDegrees turns the robot in place counterclockwise by the number of degrees
     * @param speed
     * @param degrees +ve is counterclockwise, -ve is clockwise
     * @param timeoutS amount of time until program gives up
     */
    public void turnDegrees(double speed, double degrees, double timeoutS){
        // 90 degrees = 666 counts
        double oldHeading = robot.getRawHeading();
        degrees *= COUNTS_PER_DEGREE;
        encoderDriveSimple(speed, -degrees, degrees,timeoutS);
        double newHeading = robot.getRawHeading();
        telemetry.addData("Old Heading", oldHeading);
        telemetry.addData("New Heading", newHeading);
        telemetry.update();
        sleep(1000);
    }

    /**
     * Method to drive at any given angle for a certain number of degrees. Maintains heading. Does not stop movement.
     * @param speed desired magnitude speed for the robot
     * @param direction forwards = 0, right = 90, backwards = 180, left = 270
     * @param inches you know what this means
     */
    public void encoderDriveNoWaiting(double speed, double direction, double inches) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        robot.stopAndResetDriveEncoders();
        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                robot.fLMotor.getCurrentPosition(),
                robot.fRMotor.getCurrentPosition(),
                robot.bLMotor.getCurrentPosition(),
                robot.bRMotor.getCurrentPosition());
//        telemetry.update();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            /** Direction: forwards = 0, right = 90, back = 180, left = 270 **/
            // direction -= 90; direction *= -1; direction *= Math.PI/180;
            direction = (90 - direction) * Math.PI / 180 - Math.PI / 4;
            double v1 = inches * Math.cos(direction);
            double v2 = inches * Math.sin(direction);
            double v3 = inches * Math.sin(direction);
            double v4 = inches * Math.cos(direction);

            newFrontLeftTarget = -robot.fLMotor.getCurrentPosition() + (int)(v1 * COUNTS_PER_INCH);
            newFrontRightTarget = -robot.fRMotor.getCurrentPosition() + (int)(v2 * COUNTS_PER_INCH);
            newBackLeftTarget = -robot.bLMotor.getCurrentPosition() + (int)(v3 * COUNTS_PER_INCH);
            newBackRightTarget = -robot.bRMotor.getCurrentPosition() + (int)(v4 * COUNTS_PER_INCH);

            robot.fLMotor.setTargetPosition(newFrontLeftTarget);
            robot.fRMotor.setTargetPosition(newFrontRightTarget);
            robot.bLMotor.setTargetPosition(newBackLeftTarget);
            robot.bRMotor.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.fLMotor.setTargetPosition(newFrontLeftTarget);
            robot.fRMotor.setTargetPosition(newFrontRightTarget);
            robot.bLMotor.setTargetPosition(newBackLeftTarget);
            robot.bRMotor.setTargetPosition(newBackRightTarget);

            // reset the timeout time and start motion.
            runtime.reset();
            double forwardSlashSpeed = Math.abs(speed) * Math.cos(direction);
            double backwardsSlashSpeed = Math.abs(speed) * Math.sin(direction);
            fLSpeed = forwardSlashSpeed;
            bRSpeed = fLSpeed;
            fRSpeed = backwardsSlashSpeed;
            bLSpeed = fRSpeed;
            robot.fLMotor.setPower(forwardSlashSpeed);
            robot.fRMotor.setPower(backwardsSlashSpeed);
            robot.bLMotor.setPower(backwardsSlashSpeed);
            robot.bRMotor.setPower(forwardSlashSpeed);
        }
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
//            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            turnRobot(turnSpeed);

            // Display drive status for the driver.
            sendTelemetry();
        }

        // Stop all motion;
        stopAllMotion();
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = robot.getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        //return 1 - 2 / (Math.exp(headingError / 30) + 1); this is what you had
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param turn  clockwise turning motor speed.
     */
    public void turnRobot(double turn) {
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = -turn;
        rightSpeed = turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        robot.fLMotor.setPower(leftSpeed);
        robot.bLMotor.setPower(leftSpeed);
        robot.fRMotor.setPower(rightSpeed);
        robot.bRMotor.setPower(rightSpeed);
    }
    public void stopAllMotion(){
        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);
    }

    /**
     *  Display the various control parameters while driving
     */
    public void sendTelemetry() {
        telemetry.addData("Motion", "Turning");

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }



    @SuppressLint("DefaultLocale")
    public void tagToTelemetry(AprilTagDetection detection)
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