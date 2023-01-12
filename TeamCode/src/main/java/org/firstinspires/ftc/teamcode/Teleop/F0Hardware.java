package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

public class F0Hardware {
    public CRServoImplEx left, right;
    public Servo steer;
    public RevIMU imu;


    public void init(HardwareMap h) {
        left = h.get(CRServoImplEx.class, "left");
        right = h.get(CRServoImplEx.class, "right");

        left.setPwmRange(new PwmControl.PwmRange(1000,2000));
        right.setPwmRange(new PwmControl.PwmRange(1000,2000));

        left.setPwmEnable();
        right.setPwmEnable();

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPower(0);
        right.setPower(0);

        // Servo setup

        steer = h.get(Servo.class, "steer");


        // IMU

        imu = new RevIMU(h, "imu");
        imu.init();
    }
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }
    public void steer(double pos) {
        if(pos > 1) {
            pos = 1;
        }
        if(pos < -1) {
            pos = -1;
        }
        InterpLUT steerLUT = new InterpLUT();
        steerLUT.add(-1.01,0.35);
        steerLUT.add(0,0.525);
        steerLUT.add(1.01, 0.7);
        steerLUT.createLUT();
        steer.setPosition(steerLUT.get(-1*pos));
    }
}
