package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Move Forward Test")
public class moveForward extends AutonDriving {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(true);
        waitForStart();
        encoderDrive(0.9, 0, 100, 10);
    }
}
