package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Camera;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "MoveToSquare", group = "Robot")
public class MoveToTargetSquare extends AutonDriving {
    AprilTagDetection tagOfInterest;
    @Override
    public void runOpMode() {
        Camera cam = new Camera(this);
        Lift lift = new Lift(this, true);
        tagOfInterest = getTag(cam.initAprilTagDetection());
        lift.flipToPosition(0.5);
        if (tagOfInterest != null) {
            squarePathSignal(tagOfInterest);
        }
        lift.flipToPosition(0);
    }
}
