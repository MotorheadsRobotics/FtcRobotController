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

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

/**
 *  In OnBot Java, add a new OpMode, drawing from this Sample; select TeleOp.
 *  Also add another new file named RobotHardware.java, drawing from the Sample with that name; select Not an OpMode.
 */

@TeleOp(name="TestTeleopSolo", group="Robot")
public class TestTeleopSolo extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    Hardware robot = new Hardware(this);

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Drive robot via mecanum
            robot.mecanumMove(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            // Test telemetry / encoders
            telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
            telemetry.update();
            // Servo mapped to a
            if (gamepad1.a) {
                servoPosition = 1 - servoPosition;
                while (gamepad1.a) {
                }
            }
            robot.claw.setPosition(servoPosition);

            // Vertical Slides
            if (gamepad1.left_trigger > 0.3) {
                robot.upMotorL.setPower(verticalMotorPower);
                robot.upMotorR.setPower(verticalMotorPower);
            } else if (gamepad1.right_trigger > 0.3) {
                robot.upMotorL.setPower(-verticalMotorPower);
                robot.upMotorR.setPower(-verticalMotorPower);
            } else {
                robot.upMotorL.setPower(0);
                robot.upMotorR.setPower(0);
            }
//
//            // Horizontal Slides
//            if (gamepad1.y) {
//                robot.horMotor.setPower(horizontalMotorPower);
//            } else if (gamepad1.x) {
//                robot.horMotor.setPower(-horizontalMotorPower);
//            } else {
//                robot.horMotor.setPower(0);
//            }
//            // Send telemetry messages to explain controls and show robot status
//            telemetry.addData("Arm Up/Down", "Trigger Buttons");
//            telemetry.addData("Arm Forward/Backward", "Y / X Buttons");
//            telemetry.addData("-", "-------");
//            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
