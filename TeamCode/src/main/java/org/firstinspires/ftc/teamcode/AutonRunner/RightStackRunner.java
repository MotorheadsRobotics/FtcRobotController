package org.firstinspires.ftc.teamcode.AutonRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Camera;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="RoadRunner Test Right Stack", group="Robot")
public class RightStackRunner extends AutonomousDriving {
    AprilTagDetection tagOfInterest = null;
    double adjustment = 0.0;

    double leftWallPoint = -64.5;
    //TODO: change lift presets to what they actually are.


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(this, true);
        Camera tagDetector = new Camera(this);
        lift.flipToPosition(0.5);
        tagOfInterest = getTag(tagDetector.initAprilTagDetection());

        robot.setPoseEstimate(new Pose2d(36,-65.25,0));
        Vector2d coneSpot = new Vector2d(26.-5);

        Trajectory track1 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                .addDisplacementMarker(() -> lift.setLift(Lift.highInch * Lift.liftCountsPerInch, Lift.LIFTMOTORPOWER))
                .addTemporalMarker(0.5, () -> lift.flipToPosition(1))
                .addTemporalMarker(1.5, () -> lift.setRotate(1))
                .strafeLeft(47.75)
                .splineToSplineHeading(new Pose2d(30.8,-6.8,Math.toRadians(-45)), Math.toRadians(135))
                //TODO: make robot not run into pole
//                .lineTo(coneSpot)
                .splineTo(coneSpot, Math.PI + Math.atan((0 - coneSpot.getY())/(24-coneSpot.getX())))
                .build();


        trackCreator trackMod = new LeftStackRunner.trackCreator() {
            @Override
            public void track2Mod(double cone) {
                //it's possible we may need to hardcode a start point instead of getting the current estimate.
                // don't know which would cause less drift
                robot.setPoseEstimate(robot.getPoseEstimate().plus(new Pose2d(0,-adjustment)));
                track2 = robot.trajectoryBuilder(robot.getPoseEstimate())
                        .addDisplacementMarker(() -> lift.setLift((int)cone, Lift.LIFTMOTORPOWER))
                        .addTemporalMarker(0.05, () -> lift.flipToPosition(0))
                        .addTemporalMarker(0.22, lift::closeClaw)
                        .addTemporalMarker(0.3, () -> {
                            lift.setRotate(0);
                            lift.openClaw();
                        })
                        .addTemporalMarker(0.2, lift::closeClaw)
                        //TODO: Make robot not run into wall
                        .splineTo(new Vector2d(leftWallPoint, -12), Math.toRadians(0)) // theoretically this point should be (-63.5, -12) but variations idk
                        .build();
            }
            @Override
            public void track3Update(int offset) {
                track3 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                        .addTemporalMarker(0.5, () -> lift.flipToPosition(1))
                        .addTemporalMarker(1.5, () -> lift.setRotate(1))
                        .addDisplacementMarker(() -> lift.setLift(Lift.highInch * Lift.liftCountsPerInch, Lift.LIFTMOTORPOWER))
                        //TODO: copy from track 1 to not have it run into pole
                        .splineTo(new Vector2d(26.7, -2.9), Math.toRadians(135))
                        .build();
            }
        };

        waitForStart();

        if(isStopRequested()) return;

        robot.followTrajectory(track1, this);
        telemetry.addData("Path: ", "Track 1 Completed");
        telemetry.update();
        lift.downDrop();
        for (int i = 4; i >= 1; i--) {
            trackMod.track2Mod(cones[i]);
            robot.followTrajectory(track2, this);
            lift.closeClaw();
            sleep(300);
            if (robot.hitWall()){
                robot.setPoseEstimate(new Pose2d(63, -12, Math.toRadians(0)));
            }
            int offset = 166;
            trackMod.track3Update(offset);
            robot.followTrajectory(track3, this);
            lift.downDrop();
        }
        Trajectory track4;
        if(tagOfInterest == null) {
            tagOfInterest = new AprilTagDetection();
            tagOfInterest.id = -1;
        }
        switch (tagOfInterest.id) {
            case 1:
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                        .splineTo(new Vector2d(36,-26),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-60,-36),Math.toRadians(180))
                        .build();
                break;
            case 3:
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                        .splineTo(new Vector2d(36,-26),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-12,-36),Math.toRadians(0))
                        .build();
                break;
            default:
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                        .splineTo(new Vector2d(36,-36),Math.toRadians(90))
                        .build();
        }
        lift.setLift(0, Lift.LIFTMOTORPOWER);
        robot.followTrajectory(track4, this);
        sleep(10000);
    }
}
