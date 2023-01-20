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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name="Encoder Test", group="Robot")

public class EncoderCountTesting extends AutonDriving {
    // Initialize Hardware
    Hardware robot = new Hardware(this);
    ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;       // from GoBilda
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;         // Gearing up (more speed, less torque) --> ratio < 1.0
    static final double     WHEEL_DIAMETER_INCHES   = 3.77952756 ;  // 96mm
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
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
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    @Override
    public void runOpMode() {
        robot.init();
        robot.initGyro();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // turnToHeading(90,0.05,0.2); // rotate left 90 degrees
//        encoderDrive(1,0,1,3);
        turnDegrees(1,666,10);
        telemetry.addData("Path", "Done");
        telemetry.update();
        sleep(50);
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed * maxTurnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            turnRobot(turnSpeed);

            // Display drive status for the driver.
            sendTelemetry();
        }

        // Stop all motion;
        stopAllMotion();
    }
    public void turnToHeading2(double heading, double minPower, double maxPower){
        robotHeading = getRawHeading() - headingOffset;
        headingError = heading - robotHeading;
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        final double originalError = headingError;

        while(opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)){
            robotHeading = getRawHeading() - headingOffset;
            headingError = heading - robotHeading;
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            double adjuster = Math.abs(headingError / 180);
            double adjustedPower = Range.clip(adjuster, minPower, maxPower);

            if(headingError > 0){
                turnRobot(-adjustedPower);
            }
            else if(headingError < 0){
                turnRobot(adjustedPower);
            }
            telemetry.addData("heading", robotHeading);
            telemetry.addData("raw heading", getRawHeading());
            telemetry.addData("error", headingError);
            telemetry.addData("adjuster", adjuster);
            telemetry.addData("adjustedPower", adjustedPower);
            telemetry.update();
        }
        stopAllMotion();
    }

    /**
     * turns at 10% speed (or less) until reaching desired target
     * @param heading +ve turns the robot counterclockwise, -ve rotates clockwise
     */
    public void turnToHeading3(double heading){
        double currentHeading = getRawHeading();
        double errorABS = Math.abs(heading - currentHeading);
        while(errorABS > 1){
            double error = (heading - currentHeading) / 1800;
            while(error > 180) error -=360;
            while(error <= -180) error +=360;
            double speed = Range.clip(error, -0.1, 0.1);
            if(speed < 0.02 && speed > 0){
                speed = 0.02;
            }
            if(speed > -0.02 && speed < 0){
                speed = -0.02;
            }
            turnRobot(-speed);
            telemetry.addData("speed", speed);
            telemetry.addData("heading", heading);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("abs error", errorABS);
            telemetry.update();
            currentHeading = getRawHeading();
            errorABS = Math.abs(heading - currentHeading);
        }
        stopAllMotion();
    }

}