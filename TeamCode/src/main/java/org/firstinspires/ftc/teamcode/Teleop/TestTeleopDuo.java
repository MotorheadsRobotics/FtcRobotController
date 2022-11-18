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

    public static int[] heights = new int[] {0, 2, 15, 23, 32};
    public static int[] heightsCounts = new int[] {0, 660, 4950, 7590, 10560};
    public static int maxHeight = 11880;
    public static int[] stackHeights = new int[] {440, 880, 1320, 1760};
    public static String[] stackHeightNames = new String[] {"Cone 2", "Cone 3", "Cone 4", "Cone 5"};
    public static String[] heightNames = new String[] {"Floor", "Ground Terminal", "Low Terminal", "Medium Terminal", "High Terminal"};
    public int currentPreset = 0;
    public int currentStackPreset = 0;
    public static int countsPerInch = 330;
    public String mode = "MANUAL";
    private static double LIFTMOTORPOWER = 1.0;

    @Override
    public void runOpMode() {
        double army          = 0;
        double handOffset   = 0;
        double servoPosition = 0;
        double verticalMotorPower = 1.0;
        double horizontalMotorPower = 1.0;

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
            // Drive robot via mecanum
            // if (slow) {
            //     //slow mode
            //     robot.mecanumMove(0.1 * gamepad1.left_stick_x, 0.1 * gamepad1.left_stick_y, 0.1 * gamepad1.right_stick_x);
            // }
            // else {
                //fast mode
                robot.mecanumMove(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            // }
            
            //slow mode boolean change
            // if (gamepad1.a) {
            //     slow = !slow;
            //     while (gamepad1.a) {
            //     }
            // }
            
            //lift presets
            if (gamepad2.b) {
                
            }
            
            // Test telemetry / encoders
            telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
            telemetry.update();
            // Servo mapped to a
            if (gamepad2.a) {
                servoPosition = 1 - servoPosition;
                while(gamepad2.a) {
                }
            }
            robot.claw.setPosition(servoPosition);

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
                if(currentPreset < 4) {
                    currentPreset++;
                }
                mode = "PRESET";
            }
            else if(gamepad2.dpad_down){
                if(currentPreset > 0){
                    currentPreset--;
                }
                mode = "PRESET";
            }

            // Stack Mode Iteration
            if(gamepad2.dpad_right){
                if(currentStackPreset < 3) {
                    currentStackPreset++;
                }
                mode = "STACK";
            }
            else if(gamepad2.dpad_left){
                if(currentStackPreset > 0){
                    currentStackPreset--;
                }
                mode = "STACK";
            }

            // Check if we should be in manual mode
            if(gamepad2.right_trigger > 0.3 || gamepad2.left_trigger > 0.3) {
                mode = "MANUAL";
            }

            // Move Lifts
            if(mode.equals("MANUAL")){
                telemetry.addData("Manual Mode", true);
                telemetry.addData("Current Preset", heightNames[currentPreset]);

                robot.upMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.upMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if(gamepad2.left_trigger > 0.3 && robot.upMotorL.getCurrentPosition() > (0 + offsetCounts) && robot.upMotorR.getCurrentPosition() > (0 + offsetCounts)){
                    robot.upMotorL.setPower(-LIFTMOTORPOWER);
                    robot.upMotorR.setPower(-LIFTMOTORPOWER);
                }
                else if(gamepad2.right_trigger > 0.3 && robot.upMotorL.getCurrentPosition() < (maxHeight + offsetCounts) && robot.upMotorL.getCurrentPosition() < (maxHeight + offsetCounts)){
                    robot.upMotorL.setPower(LIFTMOTORPOWER);
                    robot.upMotorR.setPower(LIFTMOTORPOWER);
                }
                else{
                    robot.upMotorL.setPower(0);
                    robot.upMotorR.setPower(0);
                }
            }
            else if(mode.equals("PRESET")) { // Preset Mode 1
                telemetry.addData("Preset Mode", true);
                telemetry.addData("Current Preset", heightNames[currentPreset]);
                robot.upMotorL.setTargetPosition(heightsCounts[currentPreset] + offsetCounts);
                robot.upMotorR.setTargetPosition(heightsCounts[currentPreset] + offsetCounts);

                robot.upMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.upMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.upMotorL.setPower(LIFTMOTORPOWER);
                robot.upMotorR.setPower(LIFTMOTORPOWER);
            }
            else { // Preset Mode 2: Stacks
                telemetry.addData("Stack Mode", true);
                telemetry.addData("Current Preset", stackHeightNames[currentStackPreset]);
                robot.upMotorL.setTargetPosition(stackHeights[currentStackPreset] + offsetCounts);
                robot.upMotorR.setTargetPosition(stackHeights[currentStackPreset] + offsetCounts);

                robot.upMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.upMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.upMotorL.setPower(LIFTMOTORPOWER);
                robot.upMotorR.setPower(LIFTMOTORPOWER);
            }
            telemetry.update();
//
//            // Horizontal Slides
//            if (gamepad1.y) {
//                robot.horMotor.setPower(horizontalMotorPower);
//            } else if (gamepad1.x) {
//                robot.horMotor.setPower(-horizontalMotorPower);
//            } else {
//                robot.horMotor.setPower(0);
//            }
//            // Send telemetry messages to explain controls
//            telemetry.addData("Lift Up/Down", "Triggers");
//            telemetry.addData("Lift Adjustment", "Bumpers");
//            telemetry.addData("Lift Terminal Presets", "Dpad Up/Down");
//            telemetry.addData("Lift Stack Presets", "Dpad Left/Right");
//            telemetry.addData("Claw", "A button");
//            telemetry.addData("-", "-------");
//            telemetry.update();


            // Pace this loop so hands move at a reasonable speed.
            sleep(75);
        }
    }
}
