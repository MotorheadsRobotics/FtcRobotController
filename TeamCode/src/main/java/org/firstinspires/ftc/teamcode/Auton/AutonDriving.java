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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")

public class AutonDriving extends LinearOpMode {
    // Initialize Hardware
    Hardware robot = new Hardware(this);
    ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;       // from GoBuilda
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;         // Gearing up (more speed, less torque) --> ratio < 1.0
    static final double     WHEEL_DIAMETER_INCHES   = 3.77952756 ;  // 96mm
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive(DRIVE_SPEED,  0,  20, 5.0);
        telemetry.addData("Path", "Part 1/3 Done");
        telemetry.update();
        sleep(1000);
        encoderDrive(TURN_SPEED,   90, 20, 5.0);
        telemetry.addData("Path", "Part 2/3 Done");
        telemetry.update();
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 180, 20, 5.0);
        telemetry.addData("Path", "Part 3/3 Done");
        telemetry.update();
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 270, 20, 5.0);
        telemetry.addData("Path", "Part 3/3 Done");
        telemetry.update();
        sleep(1000);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

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
}
