package org.firstinspires.ftc.teamcode.Auton;

public class CurveTest extends AdiRunner{
    @Override
    public void runOpMode() {
        robot.init(false);
        runtime.reset();
        simpleCurveDrive(0.5, 50, 1.2);
    }
}
