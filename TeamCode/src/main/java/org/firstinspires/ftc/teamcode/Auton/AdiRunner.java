package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public abstract class AdiRunner extends AutonDriving{
    public void curveDrive (double fastSpeed, double endRelHead , double smallLength) {

    }
    public void simpleCurveDrive (double speed, double inches, double multipliertoLeft){
        encoderDriveSimple(0, inches, inches * multipliertoLeft, 0);
        double Rspeed = speed * multipliertoLeft;
        if (Rspeed > 1) {
            speed /= Rspeed;
            Rspeed = 1;
            robot.fRMotor.setPower(Rspeed);
            robot.bRMotor.setPower(Rspeed);
            robot.bLMotor.setPower(speed);
            robot.fLMotor.setPower(speed);
        }

    }
    public void turnToIMU (double relHead, double speed, Hardware robot1) {
        double targetHead = relHead + robot1.getRawHeading();
        double currentSpeed = speed;
        while (robot1.getRawHeading() != targetHead) {
            currentSpeed = getSteeringCorrection(relHead, P_TURN_GAIN);
            robot1.fLMotor.setPower(currentSpeed);
            robot1.bLMotor.setPower(currentSpeed);
            robot1.fRMotor.setPower(-currentSpeed);
            robot1.bRMotor.setPower(-currentSpeed);
        }
    }
}