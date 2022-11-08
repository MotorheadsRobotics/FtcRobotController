package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name="TestLiftEncoder", group="Robot")

public class TestLiftEncoderKevin extends LinearOpMode {
    Hardware robot = new Hardware(this);
    
    public static int[] heights = new int[] {0, 2, 10, 20, 30};
    public static int[] heightsCounts = new int[] {0, 660, 3300, 6600, 9900};
    public static String[] heightNames = new String[] {"Floor", "Ground Terminal", "Low Terminal", "Medium Terminal", "High Terminal"};
    public int currentPreset = 0;
    public static int countsPerInch = 330;
    public boolean manualMode = true;
    private static double LIFTMOTORPOWER = 1.0;

    @Override
    public void runOpMode () {
        robot.init();
        waitForStart();
        int offsetCounts = 0;

        while(opModeIsActive()) {
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
            if(gamepad2.dpad_up && currentPreset < 4){
                currentPreset++;
                manualMode = false;
            }
            else if(gamepad2.dpad_down && currentPreset > 0){
                currentPreset--;
                manualMode = false;
            }

            // Check if we should be in manual mode
            if(gamepad2.right_trigger > 0.3 || gamepad2.left_trigger > 0.3) {
                manualMode = true;
            }

            // Move Lifts
            if(manualMode){
                telemetry.addData("Manual Mode", true);
                if(gamepad2.right_trigger > 0.3){
                    robot.upMotorL.setPower(LIFTMOTORPOWER);
                    robot.upMotorR.setPower(LIFTMOTORPOWER);
                }
                else if(gamepad2.left_trigger > 0.3){
                    robot.upMotorL.setPower(-LIFTMOTORPOWER);
                    robot.upMotorR.setPower(-LIFTMOTORPOWER);
                }
                else{
                    robot.upMotorR.setPower(0);
                }
            }
            else { // Preset Mode
                telemetry.addData("Manual Mode", true);
                telemetry.addData("Current Preset", heightNames[currentPreset]);
                robot.upMotorL.setTargetPosition(heightsCounts[currentPreset] + offsetCounts);
                robot.upMotorR.setTargetPosition(heightsCounts[currentPreset] + offsetCounts);

                robot.upMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.upMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.upMotorL.setPower(LIFTMOTORPOWER);
                robot.upMotorR.setPower(LIFTMOTORPOWER);
                // might need to code in the run_to_position function because i dont know if DcMotor.RunMode.RUN_TO_POSITION only finishes exits the while loop once done.
            }
            telemetry.update();
            sleep(50);
        }
    }
}