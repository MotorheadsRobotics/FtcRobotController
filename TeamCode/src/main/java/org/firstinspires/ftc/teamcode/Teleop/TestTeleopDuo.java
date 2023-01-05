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

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

/**
 *  In OnBot Java, add a new OpMode, drawing from this Sample; select TeleOp.
 *  Also add another new file named RobotHardware.java, drawing from the Sample with that name; select Not an OpMode.
 */

@TeleOp(name="TestTeleopDuo", group="Robot")
public class TestTeleopDuo extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    Hardware robot = new Hardware(this);

//  goal heights (in) {0, 2, 15, 23, 32}

    public int currentPreset = 0;
    public int currentStackPreset = 0;
    public static int countsPerInch = 330;
    public String mode = "PRESET";
    private static final double LIFTMOTORPOWER = 1.0;
    public ElapsedTime runtime = new ElapsedTime();
    public static double FLIPDELAY = 1000; // milliseconds

    @Override
    public void runOpMode() {
        double clawPosition = 1;
        double flipPosition = 0;
        double time = 0;
        double rotatePosition = 1;
        double verticalMotorPower = 1.0;
        boolean isRotated = true;
        boolean isFlipped = true;
        boolean wasPressed = false;
        double speedMultiplier = 1;
        int liftPosL = 0, liftPosR = 0;
        boolean needsToChange = true;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // boolean slow = false;
        robot.upMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.upMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.upMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.upMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int offsetCounts = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Increase speed mapped to left bumper

            if (gamepad1.left_bumper) {
                speedMultiplier /= 2;
                while(gamepad1.left_bumper) {}
                if (speedMultiplier < 0.0625) {
                    speedMultiplier = 0.0625;
                }
            }

            // Increase speed mapped to right bumper
            if (gamepad1.right_bumper) {
                speedMultiplier *= 2;
                while(gamepad1.left_bumper) {}
                if (speedMultiplier > 1) {
                    speedMultiplier = 1;
                }
            }

            // Drive robot via mecanum
            robot.mecanumMove(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, speedMultiplier);

            // Speed Multiplier Telemetry
            telemetry.addData("Speed Multiplier: ", speedMultiplier);

            // Claw mapped to a
            if (gamepad2.a) {
                clawPosition = 1 - clawPosition;
                while(gamepad2.a) {}
            }
            robot.claw.setPosition(clawPosition);

            // Flipping mapped to y
            if (gamepad2.y) { // flipPosition = 0 means default state
                flipPosition = 1 - flipPosition;
                while(gamepad2.y) {}
                time = runtime.milliseconds();
                isRotated = false;
                isFlipped = false;
            }

            if(isFlipped){
                robot.flipL.setPosition(Hardware.FLIP_CONSTANT * (1 - flipPosition));
                robot.flipR.setPosition(Hardware.FLIP_CONSTANT * flipPosition);
            }
            else if(!isFlipped && robot.upMotorL.getCurrentPosition() + robot.upMotorR.getCurrentPosition() <= 2 * Hardware.minHeightForFlip){
                robot.flipL.setPosition(Hardware.FLIP_CONSTANT * flipPosition);
                robot.flipR.setPosition(Hardware.FLIP_CONSTANT * (1 - flipPosition));
            }
            else if(!isFlipped && robot.upMotorL.getCurrentPosition() + robot.upMotorR.getCurrentPosition() > 2 * Hardware.minHeightForFlip){
                robot.claw.setPosition(1);
                robot.flipL.setPosition(Hardware.FLIP_CONSTANT * (1 - flipPosition));
                robot.flipR.setPosition(Hardware.FLIP_CONSTANT * flipPosition);
                isFlipped = true;
            }

            //auto rotate after flip
            if(!isRotated && Math.abs(runtime.milliseconds() - time - FLIPDELAY) <= 175) {
                rotatePosition = 1 - rotatePosition;
                isRotated = true;
            }

//            //gamepad x flip
//            if (gamepad2.x) {
//                rotatePosition = 1 - rotatePosition;
//                while(gamepad2.x) {}
//            }
            robot.rotate.setPosition(Hardware.ROTATE_CONSTANT * rotatePosition);

            // Vertical Slides
            /*if (gamepad2.left_trigger > 0.3) {
                robot.upMotorL.setPower(verticalMotorPower);
                robot.upMotorR.setPower(verticalMotorPower);
            } else if (gamepad2.right_trigger > 0.3) {
                robot.upMotorL.setPower(-verticalMotorPower);
                robot.upMotorR.setPower(-verticalMotorPower);
            } else {
                robot.upMotorL.setPower(0);
                robot.upMotorR.setPower(0);
            }*/
            telemetry.addData("leftLiftPos", robot.upMotorL.getCurrentPosition());
            telemetry.addData("rightLiftPos", robot.upMotorR.getCurrentPosition());

            // Offset Calculations using bumpers
            if(gamepad2.left_bumper){
                offsetCounts -= 33;
            }
            else if(gamepad2.right_bumper){
                offsetCounts += 33;
            }

            // dpad setting presets
            if(gamepad2.dpad_up){
                if(currentPreset < 5) {
                    currentPreset++;
                }
                while(gamepad2.dpad_up) {}
                mode = "PRESET";
            }
            else if(gamepad2.dpad_down){
                if(currentPreset > 0){
                    currentPreset--;
                }
                while(gamepad2.dpad_down) {}
                mode = "PRESET";
            }
//
            // Stack Mode Iteration
            if(gamepad2.dpad_right){
                if(currentStackPreset < 1) {
                    currentStackPreset++;
                }
                while(gamepad2.dpad_right) {}
                mode = "STACK";
            }
            else if(gamepad2.dpad_left){
                if(currentStackPreset > 0){
                    currentStackPreset--;
                }
                mode = "STACK";
                while(gamepad2.dpad_left) {}
            }

            // Check if we should be in manual mode
            if(gamepad2.right_trigger > 0.3 || gamepad2.left_trigger > 0.3) {
                mode = "MANUAL";
            }

            // Move Lifts
            if(mode.equals("MANUAL")){
                telemetry.addData("Manual Mode", true);
                telemetry.addData("Current Preset", Hardware.heightNames[currentPreset]);

                robot.upMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.upMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if(gamepad2.left_trigger > 0.3 && robot.upMotorL.getCurrentPosition() > (offsetCounts) && robot.upMotorR.getCurrentPosition() > (offsetCounts)){
                    robot.upMotorL.setPower(-0.5);
                    robot.upMotorR.setPower(-0.5);
                    needsToChange = true;
                }
                else if(gamepad2.right_trigger > 0.3 && robot.upMotorL.getCurrentPosition() < (Hardware.maxHeight + offsetCounts) && robot.upMotorL.getCurrentPosition() < (Hardware.maxHeight + offsetCounts)){
                    robot.upMotorL.setPower(LIFTMOTORPOWER);
                    robot.upMotorR.setPower(LIFTMOTORPOWER);
                    needsToChange = true;
                }
                else{
                    // record last known position
                    if(needsToChange) {
                        liftPosL = robot.upMotorL.getCurrentPosition();
                        liftPosR = robot.upMotorL.getCurrentPosition();
                        needsToChange = false;
                        robot.setLift((liftPosL + liftPosR)/2, LIFTMOTORPOWER);
                    }
                    // set positions there

                }
            }
            else if(mode.equals("PRESET")) { // Preset Mode 1
                telemetry.addData("Preset Mode", true);
                telemetry.addData("Current Preset", Hardware.heightNames[currentPreset]);
                robot.setLift(Hardware.heightsCounts[currentPreset], LIFTMOTORPOWER);
            }
            else { // Preset Mode 2: Stacks
                telemetry.addData("Stack Mode", true);
                telemetry.addData("Current Preset", Hardware.stackHeightNames[currentStackPreset]);
                robot.setLift(Hardware.stackHeights[currentStackPreset], LIFTMOTORPOWER);
            }
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(75);
        }
    }
}
