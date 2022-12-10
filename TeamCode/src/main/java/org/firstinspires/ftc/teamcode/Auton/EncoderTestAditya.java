package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name = "EncoderTestAditya", group = "Robot")
public class EncoderTestAditya extends LinearOpMode {
    Hardware robot = new Hardware(this);
    ElapsedTime runtime = new ElapsedTime();
    public DcMotor[] motors = new DcMotor[] {robot.bRMotor, robot.bLMotor, robot.fRMotor, robot.fLMotor};
    @Override
    public void runOpMode(){
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (DcMotor motor : motors) {
            motor.setTargetPosition(motor.getCurrentPosition() + 1000);
        }
        for (DcMotor motor : motors) {
            motor.setPower(0.5);
        }
    }
}
