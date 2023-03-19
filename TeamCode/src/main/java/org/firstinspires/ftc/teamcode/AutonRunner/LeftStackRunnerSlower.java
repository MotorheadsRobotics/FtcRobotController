package org.firstinspires.ftc.teamcode.AutonRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Camera;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="Inefficient Runner", group="Robot")
public class LeftStackRunnerSlower extends AutonomousDriving {
    AprilTagDetection tagOfInterest = null;

    int cone1 = 0, cone2 = 42, cone3 = 83, cone4 = 125, cone5 = 166;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(this, true);
        Camera tagDetector = new Camera(this);
        lift.flipToPosition(0.5);
        tagOfInterest = getTag(tagDetector.initAprilTagDetection());

        robot.setPoseEstimate(new Pose2d(-36,-65.75,0));

        Trajectory track1 = robot.trajectoryBuilder(new Pose2d(-36, -65.75,0), true)
                .addDisplacementMarker(() -> lift.setLift(Lift.highInch * Lift.liftCountsPerInch, Lift.LIFTMOTORPOWER))
                .addTemporalMarker(0, 0.5, () -> lift.flipToPosition(1))
                .addTemporalMarker(0, 1.5, () -> lift.setRotate(1))
                .strafeLeft(47.75)
                .build();

        Trajectory midtrack = robot.trajectoryBuilder(track1.end(), true)
                .splineToSplineHeading(new Pose2d(-30.8,-6.8,Math.toRadians(225)), Math.toRadians(45))
                .lineTo(new Vector2d(-26.8,-2.8))
                .build();


        Trajectory track2 = robot.trajectoryBuilder(track1.end())
                .addDisplacementMarker(() -> {
                    lift.setLift(cone5, Lift.LIFTMOTORPOWER);
                })
                .addTemporalMarker(0, 0.2, () -> {
                    lift.closeClaw();
                })
                .splineTo(new Vector2d(-63.5,-12), Math.toRadians(180))
                .build();

        Trajectory track3 = robot.trajectoryBuilder(track2.end(), true)
                .splineTo(new Vector2d(-26.8,-2.8), Math.toRadians(45))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        robot.followTrajectory(track1, this);
        robot.followTrajectory(midtrack, this);
        lift.downDrop(Lift.highInch * Lift.liftCountsPerInch);
        telemetry.addData("Path: ", "Track 1 Completed");
        telemetry.update();
//        sleep(3000);
//        robot.followTrajectory(track2);
//        telemetry.addData("Path: ", "Track 2 Completed");
//        telemetry.update();
//        lift.closeClaw();
//        sleep(100);
//        lift.setLift(Lift.highInch * Lift.liftCountsPerInch, Lift.LIFTMOTORPOWER);
//        sleep(200);
//        robot.followTrajectory(track3);
//        lift.downDropUp(Lift.highInch * Lift.liftCountsPerInch);
        // ...

        sleep(10000);
    }
}
