package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous
public class GyroTest extends RobotAutoDriveByGyro_Linear{

    @Override
    public void runOpMode() {
            turnToHeading(0.3, 90);
            telemetry.addLine(String.format("%d", getRawHeading()));
            telemetry.update();
    }
}
