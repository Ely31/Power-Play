package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.actuators.LinearActuator;

@Config
public class InternalPIDLift {
    public DcMotor left;
    public DcMotor right;

    final double TICKS_PER_INCH  = 24.469;

    // Measurements are in inches
    public static double maxHeight = 30;
    public static double minHeight = 0;

    public static double retractedPos = 0;
    public static double groundPos = 0;
    public static double lowPos = 0;
    public static double mediumPos = 10;
    public static double highPos = 20;

    public static PIDCoefficients coeffs = new PIDCoefficients(0,0,0);
    PIDFController leftController = new PIDFController(coeffs);
    PIDFController rightController = new PIDFController(coeffs);
    final double f = 0.15;

    public InternalPIDLift(HardwareMap hwmap){
        left = hwmap.get(DcMotor.class, "leftSpool");
        right = hwmap.get(DcMotor.class, "rightSpool");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setCoefficients(PIDCoefficients coeffs){
        leftController = new PIDFController(coeffs);
        rightController = new PIDFController(coeffs);
        InternalPIDLift.coeffs = coeffs;
    }

    // Methods
    public void setHeight(double height){
        leftController.setTargetPosition(height);
        rightController.setTargetPosition(height);
    }
    public double getLeftHeight(){
        return left.getCurrentPosition() / TICKS_PER_INCH;
    }
    public double getRightHeight(){
        return right.getCurrentPosition() / TICKS_PER_INCH;
    }
    public double getAverageHeight(){
        return (getLeftHeight() + getRightHeight()) / 2;
    }

    public void retract(){
        setHeight(retractedPos);
    }
    public void goToLow(){
        setHeight(lowPos);
    }
    public void goToMedium(){
        setHeight(mediumPos);
    }
    public void goToHigh(){
        setHeight(highPos);
    }
    public void goToJunction(int junction){
        switch (junction){
            case 0:
                retract();
                break;
            case 1:
                goToLow();
                break;
            case 2:
                goToMedium();
                break;
            case 3:
                goToHigh();
                break;
        }
    }

    public void update(){
        left.setPower(f + leftController.update(getLeftHeight()));
        right.setPower(f + rightController.update(getRightHeight()));
    }
    public void disalayDebug(Telemetry telemetry){
        telemetry.addData("error", leftController.getLastError());
        telemetry.addData("current position", getLeftHeight());
        telemetry.addData("target position", leftController.getTargetPosition());
        telemetry.addData("left power", left.getPower());
        telemetry.addData("right power", right.getPower());
        telemetry.addData("F",f);
    }
}
