package org.firstinspires.ftc.teamcode.AutonRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name="RoadRunner Test 1", group="Robot")
public class LeftStackRunner extends AutonomousDriving {
    @Override
    public void runOpMode() throws InterruptedException {
        Trajectory track1 = robot.trajectoryBuilder(new Pose2d(), true)
                .strafeLeft(41.75)
                .splineToSplineHeading(new Pose2d(-26.8,-2.8,Math.toRadians(225)), Math.toRadians(45))
                .addTemporalMarker(0,() -> {
                    //
                    telemetry.addData("let's a go", "here");
                    telemetry.update();
                })
                .build();

        waitForStart();

        if(isStopRequested()) return;

        robot.followTrajectory(track1);
    }
}
