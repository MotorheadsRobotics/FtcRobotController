package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveToSquare", group = "Robot")
public class MoveToTargetSquare extends AutonRightStack {
    @Override
    public void runOpMode() {
        robot.init(false);
        tagOfInterest = getTag(robot.initAprilTagDetection());
        robot.flipToPosition(0.5);
        if (tagOfInterest != null) {
            squarePathSignal(tagOfInterest);
        }
        robot.flipToPosition(0);
    }
}
