package org.firstinspires.ftc.teamcode.Teleop;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestLiftEncoder", group="Robot")

public class TestLiftEncoder extends LinearOpMode {
    Hardware robot = new Hardware(this);
    
    public static double[] heights = new double[] {0, 2, 10, 20, 30};
    public int currentPreset = 0;
    public static int countsPerInch = 330;
    
    @Override
    public void runOpMode () {
        double verticalMotorPower = 0.1;
        robot.init();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("leftLiftPos", robot.upMotorL.getCurrentPosition());
            telemetry.addData("rightLiftPos", robot.upMotorR.getCurrentPosition());
            telemetry.update();
            
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
            
            if (gamepad2.left_bumper) {
                
            } else if 
            
        }
    }
}