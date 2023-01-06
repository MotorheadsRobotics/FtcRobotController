package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveToSquare", group = "Robot")
public class MoveToTargetSquare extends AutonRightStackTestUnderLinearAuton{
    @Override
    public void runOpMode() {
        robot.init();
        robot.initAprilTagDetection();
        tagOfInterest = getTag();
        basicPathWithCamera(tagOfInterest);
    }
}
