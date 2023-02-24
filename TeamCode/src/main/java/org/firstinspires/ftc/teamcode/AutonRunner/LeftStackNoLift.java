package org.firstinspires.ftc.teamcode.AutonRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="Left Stack NO LIFT", group="Robot")
public class LeftStackNoLift extends AutonomousDriving {
    AprilTagDetection tagOfInterest = null;
    //TODO: change lift presets to what they actually are.
    double adjustment = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        lift.init(true);
        lift.flipToPosition(0.5);
        tagOfInterest = getTag(tagDetector.initAprilTagDetection());

        robot.setPoseEstimate(new Pose2d(-36,-65.25,0));

        track1 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                .strafeLeft(47.25)
                .splineToSplineHeading(new Pose2d(-30.8,-6.8,Math.toRadians(225)), Math.toRadians(45))
                //TODO: make robot not run into pole
                .lineTo(new Vector2d(-28.2,-4.2))
                .build();


         trackCreator trackMod = new trackCreator() {
             @Override
             public void track2Mod(double cone) {
                 // it's possible we may need to hardcode a start point instead of getting the current estimate.
                 // don't know which would cause less drift
                 robot.setPoseEstimate(robot.getPoseEstimate().plus(new Pose2d(0,-adjustment)));
                 track2 = robot.trajectoryBuilder(robot.getPoseEstimate())
                         .addTemporalMarker(0.1, () -> lift.closeClaw())
                         //TODO: Make robot not run into wall
                         .splineTo(new Vector2d(-64.0, -13.50), Math.toRadians(180)) // theoretically this point should be (-63.0, -12) but variations idk
                         .build();
             }
             @Override
             public void track3Update() {
                 track3 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                         //TODO: copy from track 1 to not have it run into pole
                         .splineTo(new Vector2d(-28.2, -4.2), Math.toRadians(45))
                         .build();
             }
         };

        waitForStart();

        if(isStopRequested()) return;

        robot.followTrajectory(track1);
        telemetry.addData("Path: ", "Track 1 Completed - Preloaded Cone");
        telemetry.update();
        lift.downDrop();

        for (int i = 4; i >= 1; i--) {
            // Go towards cone stack
            trackMod.track2Mod(cones[i]);
            robot.followTrajectory(track2);
            sleep(100);
            telemetry.addData("Path: ", "Track 2 Completed - (" + (5 - i) + "/5)");
            telemetry.update();

            // Go back to high goal
            trackMod.track3Update();
            robot.followTrajectory(track3);
            telemetry.addData("Path: ", "Track 3 Completed - (" + (5 - i) + "/5)");
            telemetry.update();
        }

        // Park in designated spot
        Trajectory track4 = null;
        if(tagOfInterest == null){tagOfInterest = new AprilTagDetection(); tagOfInterest.id = 2;}
        int loopIterations = 4;
        switch (tagOfInterest.id) {
            case 1: // Park Left
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate().plus(new Pose2d(0,adjustment * loopIterations)), false)
                        .splineTo(new Vector2d(-36,-28),Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-60,-36),Math.toRadians(180))
                        .build();
                break;
            case 3: // Park Right
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate().plus(new Pose2d(0,adjustment * loopIterations)), false)
                        .splineTo(new Vector2d(-36,-28),Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-12,-36),Math.toRadians(0))
                        .build();
                break;
            default: // Park Middle / or guess middle if no tag found
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate().plus(new Pose2d(0,adjustment * loopIterations)), false)
                        .splineTo(new Vector2d(-36,-36),Math.toRadians(270))
                        .build();
        }
        robot.followTrajectory(track4);
        telemetry.addData("Path: ", "Track 4 Completed - Park");
        telemetry.update();

        sleep(10000);
    }
}
