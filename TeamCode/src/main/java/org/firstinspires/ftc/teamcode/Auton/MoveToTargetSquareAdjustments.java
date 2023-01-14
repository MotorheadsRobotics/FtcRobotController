package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveToSquareSketch", group = "Robot")
public class MoveToTargetSquareAdjustments extends AutonRightStack {
    @Override
    public void runOpMode() {
        robot.init();
        tagOfInterest = getTag(robot.initAprilTagDetection());
        if (tagOfInterest != null) {
            squarePathSignal(tagOfInterest);
        }
    }
}
