package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class turnTest extends AdiRunner{
    @Override
    public void runOpMode() {
        robot.init(false);
        robot.initGyro();
        runtime.reset();
        turnToIMU(90, 0.5, robot);
    }
}
