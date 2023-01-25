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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.AutonDriving;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

/**
 *  In OnBot Java, add a new OpMode, drawing from this Sample; select TeleOp.
 *  Also add another new file named RobotHardware.java, drawing from the Sample with that name; select Not an OpMode.
 */

@TeleOp(name="Duo No Calibration", group="Robot")
public class DuoNoCalibration extends AutonDriving {
    Hardware robot = new Hardware(this);

//  goal heights (in) {0, 2, 15, 23, 32}

    public int currentPreset = 0;
    public static int countsPerInch = 89; // 330
    private static final double LIFTMOTORPOWER = 0.8;
    public ElapsedTime runtime = new ElapsedTime();
    public static double FLIPDELAY = 1100; // milliseconds

    public static int[] heightsCounts = new int[] {0, 119, 237, 356, 475, 2937, 1335, 2047};
    public static int maxHeight = 3204;
    public static String[] heightNames = new String[] {"Floor", "Cone 2", "Cone 3", "Cone 4", "Cone 5", "High Terminal", "Low Terminal", "Medium Terminal"};

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
        boolean lock = false;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.initWithoutCalibration();
        robot.initGyro();

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

            if (gamepad1.left_trigger > 0.3) {
                speedMultiplier /= 2;
                while(gamepad1.left_trigger > 0.3) {}
                if (speedMultiplier < 0.0625) {
                    speedMultiplier = 0.0625;
                }
            }

            // Increase speed mapped to right bumper
            if (gamepad1.right_trigger > 0.3) {
                speedMultiplier *= 2;
                while(gamepad1.right_trigger > 0.3) {}
                if (speedMultiplier > 1) {
                    speedMultiplier = 1;
                }
            }

            if(gamepad2.left_stick_button && gamepad2.right_stick_button){
                lock = !lock;
            }
            // Drive robot via mecanum
            if(!lock)
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
                isRotated = false;
                isFlipped = false;
            }

            if(isFlipped){
                robot.flipL.setPosition(Hardware.FLIP_CONSTANT * (1 - flipPosition));
                robot.flipR.setPosition(Hardware.FLIP_CONSTANT * flipPosition);
            }
            else if(robot.upMotorL.getCurrentPosition() + robot.upMotorR.getCurrentPosition() <= 2 * Hardware.minHeightForFlip){
                robot.flipL.setPosition(Hardware.FLIP_CONSTANT * flipPosition);
                robot.flipR.setPosition(Hardware.FLIP_CONSTANT * (1 - flipPosition));
            }
            else if(robot.upMotorL.getCurrentPosition() + robot.upMotorR.getCurrentPosition() > 2 * Hardware.minHeightForFlip){
                robot.claw.setPosition(1);
                robot.flipL.setPosition(Hardware.FLIP_CONSTANT * (1 - flipPosition));
                robot.flipR.setPosition(Hardware.FLIP_CONSTANT * flipPosition);
                time = runtime.milliseconds();
                isFlipped = true;
            }

            //auto rotate after flip
            if(!isRotated && Math.abs(runtime.milliseconds() - time - FLIPDELAY) <= 175) {
                rotatePosition = 1 - rotatePosition;
                isRotated = true;
            }

            if(gamepad2.x){
                rotatePosition = 1 - rotatePosition;
                while(gamepad2.x){}
            }
            robot.rotate.setPosition(Hardware.ROTATE_CONSTANT * rotatePosition);

            // Lifts
            telemetry.addData("leftLiftPos", robot.upMotorL.getCurrentPosition());
            telemetry.addData("rightLiftPos", robot.upMotorR.getCurrentPosition());

            // Offset Calculations using bumpers
            if(gamepad2.left_trigger > 0.3){
                offsetCounts -= 45;
            }
            else if(gamepad2.right_trigger > 0.3){
                offsetCounts += 45;
            }

            // dpad setting presets
            if(gamepad2.left_bumper){
                offsetCounts = 0;
                currentPreset = 5;
                while(gamepad2.left_bumper) {}
            }
            else if(gamepad2.dpad_up){
                offsetCounts = 0;
                if(currentPreset < 5) {
                    currentPreset++;
                }
                while(gamepad2.dpad_up) {}
            }
            else if(gamepad2.dpad_down){
                offsetCounts = 0;
                if(currentPreset > 0 && currentPreset < 6){
                    currentPreset--;
                }
                else if(currentPreset == 6 || currentPreset == 7){
                    currentPreset = 4;
                }
                while(gamepad2.dpad_down) {}
            }
            else if(gamepad2.dpad_right){ // right is medium
                offsetCounts = 0;
                currentPreset = 7;
                while(gamepad2.dpad_right) {}
            }
            else if(gamepad2.dpad_left){ // left is low
                offsetCounts = 0;
                if(currentPreset > 0){
                    currentPreset = 6;
                }
                while(gamepad2.dpad_left) {}
            }

            // Move Lifts
            telemetry.addData("Preset Mode", true);
            telemetry.addData("Current Preset", heightNames[currentPreset]);

//            telemetry.addLine(String.format("%d", getRawHeading()));
            robot.setLift(heightsCounts[currentPreset] + offsetCounts, LIFTMOTORPOWER);

            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(75);
        }
    }
}
