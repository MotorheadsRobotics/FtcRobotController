package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.teamcode.Hardware.Chassis;

public abstract class AdiRunner extends AutonDriving{
    public void curveDrive (double fastSpeed, double endRelHead , double smallLength) {

    }
    public void simpleCurveDrive (double speed, double inches, double multipliertoLeft, double timeoutS){
        encoderDriveSimple(0, inches, inches * multipliertoLeft, 0);
        double Rspeed = speed * multipliertoLeft;
        if (Rspeed > 1) {
            speed /= Rspeed;
            Rspeed = 1;
        }
        double startTime = runtime.seconds();
        robot.setDrivePower(speed, Rspeed, speed, Rspeed);
        while (robot.isBusy() && timeoutS < runtime.seconds() - startTime){}
        robot.setDrivePower(0, 0, 0, 0);
    }
    public void turnToIMU (double relHead, double speed, Chassis robot1) {
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