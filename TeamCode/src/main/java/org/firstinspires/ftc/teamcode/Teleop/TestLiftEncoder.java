package org.firstinspires.ftc.teamcode.Teleop;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestLiftEncoder", group="Robot")

public class TestLiftEncoder extends LinearOpMode {
    Hardware robot = new Hardware(this);
    
    public static int[] heights = new int[] {0, 2, 10, 20, 30};
    public int currentPreset = 0;
    public static int countsPerInch = 330;
    public boolean liftPresets = false;
    public boolean newMode = false;
    
    @Override
    public void runOpMode () {
        double verticalMotorPower = 0.1;
        robot.init();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("leftLiftPos", robot.upMotorL.getCurrentPosition());
            telemetry.addData("rightLiftPos", robot.upMotorR.getCurrentPosition());
            telemetry.update();

            //switching lift modes based on trigger vs bumper toggle
            if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
                liftPresets = false;
            }
            else if (gamepad2.left_bumper || gamepad2.right_bumper) {
                if (!liftPresets) {
                    newMode = true;
                }
                liftPresets = true;
            }

            if (newMode) {
                robot.upMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.upMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                int target = 0;
                if (gamepad2.right_bumper) {
                    for (int i = 0; i < heights.length; i++) {
                        int current = heights[i] * countsPerInch;
                        //only use 1 motor as they should theoretically be the same position
                        if (current < robot.upMotorL.getCurrentPosition()) {
                            target = current;
                            currentPreset = i;
                        } else {
                            break;
                        }
                    }
                    while (gamepad2.right_bumper){

                    }
                }
                else if (gamepad2.left_bumper) {
                    for (int i = 0; i < heights.length; i++) {
                        int current = heights[i] * countsPerInch;
                        //only use 1 motor as they should theoretically be the same position
                        if (current > robot.upMotorL.getCurrentPosition()) {
                            target = current;
                            currentPreset = i;
                            break;
                        }
                    }
                    while (gamepad2.left_bumper){

                    }
                }
                robot.upMotorL.setTargetPosition(target);
                robot.upMotorR.setTargetPosition(target);
            }

            if (!liftPresets) {
                robot.upMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.upMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (gamepad2.left_trigger > 0.3) {
                    robot.upMotorL.setPower(verticalMotorPower);
                    robot.upMotorR.setPower(verticalMotorPower);
                } else if (gamepad2.right_trigger > 0.3) {
                    robot.upMotorL.setPower(-verticalMotorPower);
                    robot.upMotorR.setPower(-verticalMotorPower);
                } else {
                    robot.upMotorL.setPower(0);
                    robot.upMotorR.setPower(0);
                }
            }
            else {
                if (gamepad2.left_bumper && currentPreset < heights.length - 1) {
                    currentPreset++;
                    while (gamepad2.left_bumper) {

                    }
                }
                else if (gamepad2.right_bumper && currentPreset > 0) {
                    currentPreset--;
                    while (gamepad2.right_bumper) {

                    }
                }

                int target = heights[currentPreset] * countsPerInch;
                robot.upMotorR.setTargetPosition(target);
                robot.upMotorL.setTargetPosition(target);
            }
        }
    }
}