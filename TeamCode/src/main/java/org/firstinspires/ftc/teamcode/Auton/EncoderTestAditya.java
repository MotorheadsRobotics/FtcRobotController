package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name = "EncoderTestAditya", group = "Robot")
public class EncoderTestAditya extends IterativeAuton {
    @Override
    public void start() {
        runtime.reset();
        run();
    }

    public void run() {
        robot.stopAndResetDriveEncoders();
        robot.setDriveModeEncoder();
        robot.fRMotor.setTargetPosition(1000);
        robot.fLMotor.setTargetPosition(1000);
        robot.bRMotor.setTargetPosition(1000);
        robot.bLMotor.setTargetPosition(1000);
        robot.setDriveModeRTP();
        robot.setDrivePower(0.2, 0.2, 0.2, 0.2);
    }

}
