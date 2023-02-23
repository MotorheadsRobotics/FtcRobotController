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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Lift;

/**
 *  In OnBot Java, add a new OpMode, drawing from this Sample; select TeleOp.
 *  Also add another new file named RobotHardware.java, drawing from the Sample with that name; select Not an OpMode.
 */

@TeleOp(name="Lift Test", group="Robot")
public class TestLift extends LinearOpMode {
    Lift lift = new Lift(this);
    private ElapsedTime runtime = new ElapsedTime();
    private static final double LIFTMOTORPOWER = 0.8;
    private static final double FLIPDELAY = 1100;

    @Override
    public void runOpMode() {
        double clawPosition = 1;
        double flipPosition = 0;
        double time = 0;
        double rotatePosition = 1;
        boolean isRotated = true;
        boolean isFlipped = true;
        double speedMultiplier = 1;
        boolean lock = false;

        lift.init(true);
        waitForStart();

        lift.stopAndResetLiftEncoders();
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int position = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad2.a) {
                clawPosition = 1 - clawPosition;
                while(gamepad2.a) {}
            }

            if(clawPosition == 1){
                lift.closeClaw();
            }
            else{
                lift.openClaw();
            }

            // Flipping mapped to y
            if (gamepad2.y) { // flipPosition = 0 means default state
                flipPosition = 1 - flipPosition;
                while(gamepad2.y) {}
                isRotated = false;
                isFlipped = false;
            }

            if(isFlipped){
                lift.flipToPosition(flipPosition);
            }
            else if(!lift.canFlip()){
                lift.flipToPosition(1 - flipPosition);
            }
            else if(lift.canFlip()){
                lift.closeClaw();
                lift.flipToPosition(flipPosition);
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
            lift.setRotate(1 - rotatePosition);

            // Lifts
            telemetry.addData("leftLiftPos", lift.getCurrentLiftHeights()[0]);
            telemetry.addData("rightLiftPos", lift.getCurrentLiftHeights()[1]);
            telemetry.addData("position", position);

            // Adjust Lift
            if(gamepad2.left_trigger > 0.3){
                position -= 1;
            }
            else if(gamepad2.right_trigger > 0.3){
                position += 1;
            }
            else if(gamepad2.dpad_up){
                position += 50;
            }
            else if(gamepad2.dpad_down){
                position -= 50;
            }
            else if(gamepad2.dpad_left){
                position -= 10;
            }
            else if(gamepad2.dpad_right){
                position += 10;
            }

            if(position < 0)
                position = 0;

            lift.setLift(position, LIFTMOTORPOWER);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(25);
        }
    }
}