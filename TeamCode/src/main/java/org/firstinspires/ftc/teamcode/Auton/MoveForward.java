package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveForward", group = "Robot")
public class MoveForward extends IterativeAuton {
    @Override
    public void start() {
        runtime.reset();
        run();
        robot.flipToPosition(0.5);
    }

    public void run() {
        robot.stopAndResetDriveEncoders();
        int fRorig = robot.fRMotor.getCurrentPosition();
        int fLorig = robot.fLMotor.getCurrentPosition();
        int bRorig = robot.bRMotor.getCurrentPosition();
        int bLorig = robot.bLMotor.getCurrentPosition();

        robot.setDriveModeEncoder();
        robot.fRMotor.setTargetPosition(-1000);
        robot.fLMotor.setTargetPosition(-1000);
        robot.bRMotor.setTargetPosition(-1000);
        robot.bLMotor.setTargetPosition(-1000);
        robot.setDriveModeRTP();
        robot.setDrivePower(0.8, 0.8, 0.8, 0.8);

        while (true) {
            telemetry.addData("FR", fRorig);
            telemetry.addData("FL", fLorig);
            telemetry.addData("BR", bRorig);
            telemetry.addData("BL", bLorig);
            telemetry.addData("FRNew", robot.fRMotor.getCurrentPosition());
            telemetry.addData("FLNew", robot.fLMotor.getCurrentPosition());
            telemetry.addData("BRNew", robot.bRMotor.getCurrentPosition());
            telemetry.addData("BLNew", robot.bLMotor.getCurrentPosition());
            telemetry.update();
        }
    }

}
