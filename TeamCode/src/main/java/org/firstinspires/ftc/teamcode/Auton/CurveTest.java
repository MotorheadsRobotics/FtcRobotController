package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class CurveTest extends AdiRunner{
    @Override
    public void runOpMode() {
        robot.init(false);
        waitForStart();
        simpleCurveDrive(0.5, 50, 1.2, 3);
    }
}
