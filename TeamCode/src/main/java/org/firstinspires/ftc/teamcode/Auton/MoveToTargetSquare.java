package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveToSquare", group = "Robot")
public class MoveToTargetSquare extends AutonRightStack {
    @Override
    public void runOpMode() {
        robot.init();
        tagOfInterest = getTag(robot.initAprilTagDetection());
        if (tagOfInterest != null) {
            squarePathSignal(tagOfInterest);
        }
    }
}
