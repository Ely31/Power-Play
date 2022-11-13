package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class BasicLift {
   DcMotor leftSpool;
   DcMotor rightSpool;

    public BasicLift(HardwareMap hwmap){
        // Hardwaremap stuff
        leftSpool = hwmap.get(DcMotor.class, "leftSpool");
        rightSpool = hwmap.get(DcMotor.class, "rightSpool");

        rightSpool.setDirection(DcMotorSimple.Direction.REVERSE);
        }

    // Methods
    public void setLiftPower(double power){
        leftSpool.setPower(power);
        rightSpool.setPower(power);
    }
}
