package org.firstinspires.ftc.teamcode.AutonRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="RoadRunner Test Left Stack", group="Robot")
public class LeftStackRunner extends AutonomousDriving {
    AprilTagDetection tagOfInterest = null;
    //TODO: change lift presets to what they actually are.
    int cone1 = 0, cone2 = 42, cone3 = 83, cone4 = 125, cone5 = 8 * Lift.liftCountsPerInch;
    int[] cones = new int[] {cone1, cone2, cone3, cone4, cone5};
    Trajectory track2;
    interface trackCreator {
        void track2Mod(int cone);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        lift.init(true);
        lift.flipToPosition(0.5);
        tagOfInterest = getTag(tagDetector.initAprilTagDetection());

        robot.setPoseEstimate(new Pose2d(-36,-65.75,0));

        Trajectory track1 = robot.trajectoryBuilder(new Pose2d(-36, -65.75,0), true)
                .addDisplacementMarker(() -> lift.setLift(Lift.highInch * Lift.liftCountsPerInch, Lift.LIFTMOTORPOWER))
                .addTemporalMarker(0.5, () -> lift.flipToPosition(1))
                .addTemporalMarker(1.5, () -> lift.setRotate(1))
                .strafeLeft(47.75)
                .splineToSplineHeading(new Pose2d(-30.8,-6.8,Math.toRadians(225)), Math.toRadians(45))
                //TODO: make robot not run into pole
                .lineTo(new Vector2d(-26.4,-2.4))
                .build();


         trackCreator track2Mod = new trackCreator() {
             @Override
             public void track2Mod(int cone) {
                 track2 = robot.trajectoryBuilder(robot.getPoseEstimate())
                         .addDisplacementMarker(() -> lift.setLift(cone, Lift.LIFTMOTORPOWER))
                         .addTemporalMarker(0.15, () -> lift.flipToPosition(0))
                         .addTemporalMarker(0.3, () -> {
                             lift.setRotate(0);
                             lift.openClaw();
                         })
                         .addTemporalMarker(0.1, () -> lift.closeClaw())
                         //TODO: Make robot not run into wall
                         .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                         .build();
             }
         };



        Trajectory track3 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                .addDisplacementMarker(() -> lift.setLift(Lift.highInch * Lift.liftCountsPerInch, Lift.LIFTMOTORPOWER))
                //TODO: copy from track 1 to not have it run into pole
                .splineTo(new Vector2d(-26.8,-2.8), Math.toRadians(45))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        robot.followTrajectory(track1);
        telemetry.addData("Path: ", "Track 1 Completed");
        telemetry.update();
        lift.downDrop();
        track2Mod.track2Mod(cone5);
        robot.followTrajectory(track2);
        telemetry.addData("Path: ", "Track 2 Completed");
        telemetry.update();
        for (int i = 4; i >= 0; i--) {
            lift.closeClaw();
            sleep(100);
            robot.followTrajectory(track3);
            track2Mod.track2Mod(cones[i]);
            robot.followTrajectory(track2);
        }
        sleep(10000);
    }
}
