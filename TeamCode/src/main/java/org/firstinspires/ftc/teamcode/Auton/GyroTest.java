package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous
public class GyroTest extends AutonDriving{

    @Override
    public void runOpMode() {
        robot.init();
        robot.initGyro();
        waitForStart();
        gyroStrafeDrive(0.5, 0, 30);
        telemetry.addData("Heading", robot.getRawHeading());
        telemetry.update();
    }
}
