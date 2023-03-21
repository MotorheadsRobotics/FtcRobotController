package org.firstinspires.ftc.teamcode.AutonRunner;

import static org.firstinspires.ftc.teamcode.Auton.AprilTagImageRecognition.FEET_PER_METER;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


public abstract class AutonomousDriving extends LinearOpMode {
    Trajectory track2;
    Trajectory track1;
    Trajectory track3;
    interface trackCreator {
        void track2Mod(double cone);
        void track3Update(int offset);
    }

    public AprilTagDetection getTag(AprilTagDetectionPipeline pipeline) {
        AprilTagDetection tagOfInterest = null;
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 1 || tag.id == 2 || tag.id == 3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        return tagOfInterest;
    }

    @SuppressLint("DefaultLocale")
    public void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    double cone1 = 1.7, cone2 = 2.7 * Lift.liftCountsPerInch, cone3 = 3.7 * Lift.liftCountsPerInch, cone4 = 4.7 * Lift.liftCountsPerInch, cone5 = 5.7 * Lift.liftCountsPerInch;
    double[] cones = new double[]{cone1, cone2, cone3, cone4, cone5};
}


