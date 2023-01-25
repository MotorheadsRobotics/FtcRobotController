package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous
public class GyroTest extends AutonDriving{

    @Override
    public void runOpMode() {
        robot.init();
        robot.initGyro();
        gyroStrafeDrive(0.5, 90, 30);
        telemetry.addData("Heading", robot.getRawHeading());
        telemetry.update();
    }
}
