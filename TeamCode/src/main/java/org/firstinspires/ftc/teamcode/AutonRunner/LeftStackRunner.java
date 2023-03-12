package org.firstinspires.ftc.teamcode.AutonRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Camera;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="RoadRunner Test Left Stack", group="Robot")
public class LeftStackRunner extends AutonomousDriving {
    AprilTagDetection tagOfInterest = null;
    //TODO: change lift presets to what they actually are.
    double y_adjustment = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(this, true);
        Camera tagDetector = new Camera(this);
        lift.flipToPosition(0.5);
        tagOfInterest = getTag(tagDetector.initAprilTagDetection());

        robot.setPoseEstimate(new Pose2d(-36,-65.25,0));

        track1 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                .addDisplacementMarker(() -> lift.setLift(Lift.highInch * Lift.liftCountsPerInch, Lift.LIFTMOTORPOWER))
                .addTemporalMarker(0.5, () -> lift.flipToPosition(1))
                .addTemporalMarker(1.5, () -> lift.setRotate(1))
                .strafeLeft(42.5)
                .splineToSplineHeading(new Pose2d(-30.8,-6.8,Math.toRadians(225)), Math.toRadians(45))
                //TODO: make robot not run into pole
                .splineTo(new Vector2d(-27,-3.6), Math.toRadians(40))
                .build();


         trackCreator trackMod = new trackCreator() {
             @Override
             public void track2Mod(double cone) {
                 // it's possible we may need to hardcode a start point instead of getting the current estimate.
                 // don't know which would cause less drift
                 double num = -63.5;
                 track2 = robot.trajectoryBuilder(robot.getPoseEstimate())
                         .addDisplacementMarker(() -> lift.setLift((int)cone, Lift.LIFTMOTORPOWER))
                         .addTemporalMarker(0.05, () -> lift.flipToPosition(0))
                         .addTemporalMarker(0.3, () -> {
                             lift.setRotate(0);
                             lift.openClaw();
                         })
                         .addTemporalMarker(0.15, lift::closeClaw)
                         //TODO: Make robot not run into wall
                         .splineTo(new Vector2d(num, -14), Math.toRadians(180)) // theoretically this point should be (-63.5, -12) but variations idk
                         .build();
             }
             @Override
             public void track3Update(int offset) {
                 robot.setPoseEstimate(robot.getPoseEstimate().plus(new Pose2d(0, -y_adjustment)));
                 track3 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                         .addTemporalMarker(0.5, () -> lift.flipToPosition(1))
                         .addTemporalMarker(1.5, () -> lift.setRotate(1))
                         .addDisplacementMarker(() -> {
                             lift.closeClaw();
                             lift.setLift(Lift.highInch * Lift.liftCountsPerInch + offset, Lift.LIFTMOTORPOWER);
                         })
                         //TODO: copy from track 1 to not have it run into pole
                         .splineTo(new Vector2d(-27.6, -4), Math.toRadians(40)) // further away from the cone
                         .build();
             }
         };

        waitForStart();

        if(isStopRequested()) return;

        robot.followTrajectory(track1, this);
        telemetry.addData("Path: ", "Track 1 Completed - Preloaded Cone");
        telemetry.update();
        lift.downDrop(Lift.heightsCounts[1] - 100, 400);

        for (int i = 3; i >= 1 && opModeIsActive(); i--) {
            // Go towards cone stack
            trackMod.track2Mod(cones[i]);
            robot.followTrajectory(track2, this);
            lift.closeClaw();
            sleep(300);
            telemetry.addData("Path: ", "Track 2 Completed - (" + (4 - i) + "/4)");
            telemetry.update();

            // Go back to high goal
            int offset = 166;
            if (robot.hitWall()){
                robot.setPoseEstimate(new Pose2d(-62.5, -14, Math.toRadians(180)));
            }
            trackMod.track3Update(offset);
            robot.followTrajectory(track3, this);
            lift.downDrop();
            telemetry.addData("Path: ", "Track 3 Completed - (" + (4 - i) + "/4)");
            telemetry.update();
        }

        // Park in designated spot
        Trajectory track4;
        if(tagOfInterest == null) {
            tagOfInterest = new AprilTagDetection();
            tagOfInterest.id = 0;
        }
        switch (tagOfInterest.id) {
            case 1: // Park Left
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate())
                        .addDisplacementMarker(() -> lift.setLift(0, Lift.LIFTMOTORPOWER / 2))
                        .addTemporalMarker(0.05, () -> lift.flipToPosition(0))
                        .addTemporalMarker(0.22, lift::closeClaw)
                        .addTemporalMarker(0.3, () -> lift.setRotate(0))
                        .addTemporalMarker(0.4, lift::openClaw)
                        //TODO: Make robot not run into wall
                        .splineTo(new Vector2d(-65, -16), Math.toRadians(180)) // theoretically this point should be (-63.5, -12) but variations idk
                        .build();
                break;
            case 3: // Park Right
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                        .addDisplacementMarker(() -> lift.setLift(0, Lift.LIFTMOTORPOWER / 2))
                        .addTemporalMarker(0.05, () -> lift.flipToPosition(0))
                        .addTemporalMarker(0.22, lift::closeClaw)
                        .addTemporalMarker(0.3, () -> lift.setRotate(0))
                        .splineTo(new Vector2d(-36,-24),Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-8,-36),Math.toRadians(0))
                        .build();
                break;
            default: // Park Middle / or guess middle if no tag found
                track4 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                        .addDisplacementMarker(() -> lift.setLift(0, Lift.LIFTMOTORPOWER / 2))
                        .addTemporalMarker(0.05, () -> lift.flipToPosition(0))
                        .addTemporalMarker(0.22, lift::closeClaw)
                        .addTemporalMarker(0.3, () -> lift.setRotate(0))
                        .splineTo(new Vector2d(-36,-36),Math.toRadians(270))
                        .build();
        }
        lift.setLift(0, Lift.LIFTMOTORPOWER);
        telemetry.addLine("Starting Track 4");
        robot.followTrajectory(track4, this);
        telemetry.addData("Path: ", "Track 4 Completed - Park");
        telemetry.update();
        while(lift.isBusy() && opModeIsActive());
    }
}
