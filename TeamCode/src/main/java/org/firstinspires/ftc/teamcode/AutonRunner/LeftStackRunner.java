package org.firstinspires.ftc.teamcode.AutonRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auton.AutonDriving;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name="RoadRunner Test 1", group="Robot")
public class LeftStackRunner extends AutonomousDriving {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(new Pose2d(-36,-65.75,0));

        Trajectory track1 = robot.trajectoryBuilder(new Pose2d(-36, -65.75,0), true)
                .addDisplacementMarker(() -> {
                    // Lift to high preset
                })
                .strafeLeft(47.75)
                .splineToSplineHeading(new Pose2d(-26.8,-2.8,Math.toRadians(225)), Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    // downDropUp
                })
                .build();

        waitForStart();

        if(isStopRequested()) return;

        robot.followTrajectory(track1);

        sleep(10000);
    }
}
