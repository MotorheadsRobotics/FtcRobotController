package org.firstinspires.ftc.teamcode.AutonRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="RoadRunner Test Right Stack", group="Robot")
public class RightStackRunner extends AutonomousDriving {
    AprilTagDetection tagOfInterest = null;
    //TODO: change lift presets to what they actually are.


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        lift.init(true);
        lift.flipToPosition(0.5);
        tagOfInterest = getTag(tagDetector.initAprilTagDetection());

        robot.setPoseEstimate(new Pose2d(36,-65.75,0));

        Trajectory track1 = robot.trajectoryBuilder(new Pose2d(36, -65.75,0), true)
                .addDisplacementMarker(() -> lift.setLift(Lift.highInch * Lift.liftCountsPerInch, Lift.LIFTMOTORPOWER))
                .addTemporalMarker(0.5, () -> lift.flipToPosition(1))
                .addTemporalMarker(1.5, () -> lift.setRotate(1))
                .strafeLeft(47.75)
                .splineToSplineHeading(new Pose2d(30.8,-6.8,Math.toRadians(225)), Math.toRadians(45))
                //TODO: make robot not run into pole
                .lineTo(new Vector2d(27.0,-3.0))
                .build();


        trackCreator trackMod = new LeftStackRunner.trackCreator() {
            @Override
            public void track2Mod(double cone) {
                //it's possible we may need to hardcode a start point instead of getting the current estimate.
                // don't know which would cause less drift
                track2 = robot.trajectoryBuilder(robot.getPoseEstimate())
                        .addDisplacementMarker(() -> lift.setLift((int)cone, Lift.LIFTMOTORPOWER))
                        .addTemporalMarker(0.15, () -> lift.flipToPosition(0))
                        .addTemporalMarker(0.3, () -> {
                            lift.setRotate(0);
                            lift.openClaw();
                        })
                        .addTemporalMarker(0.1, () -> lift.closeClaw())
                        //TODO: Make robot not run into wall
                        .splineTo(new Vector2d(61, -12), Math.toRadians(180))
                        .build();
            }
            @Override
            public void track3Update() {
                track3 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                        .addTemporalMarker(0.5, () -> lift.flipToPosition(1))
                        .addTemporalMarker(0.7, () -> lift.setRotate(1))
                        .addDisplacementMarker(() -> lift.setLift(Lift.highInch * Lift.liftCountsPerInch, Lift.LIFTMOTORPOWER))
                        //TODO: copy from track 1 to not have it run into pole
                        .splineTo(new Vector2d(26.8, -2.8), Math.toRadians(45))
                        .build();
            }
        };

        waitForStart();

        if(isStopRequested()) return;

        robot.followTrajectory(track1);
        telemetry.addData("Path: ", "Track 1 Completed");
        telemetry.update();
        lift.downDrop();
        for (int i = 4; i >= 0; i--) {
            trackMod.track2Mod(cones[i]);
            robot.followTrajectory(track2);
            lift.closeClaw();
            sleep(100);
            trackMod.track3Update();
            robot.followTrajectory(track3);
            lift.downDrop();
        }
        Trajectory track4 = null;
        switch (tagOfInterest.id) {
            case 1:
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                        .splineTo(new Vector2d(36,-28),Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-60,-36),Math.toRadians(180))
                        .build();
                break;
            case 3:
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                        .splineTo(new Vector2d(36,-28),Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-12,-36),Math.toRadians(0))
                        .build();
                break;
            default:
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                        .splineTo(new Vector2d(36,-36),Math.toRadians(270))
                        .build();
        }
        robot.followTrajectory(track4);
        sleep(10000);
    }
}
