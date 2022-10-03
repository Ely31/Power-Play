package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@TeleOp
public class ProtoBotTeleop extends LinearOpMode {
    // Pre init
    TeleMecDrive drive;
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx intake;

    @Override
    public void runOpMode(){
        // Init
        PhotonCore.enable();
        telemetry.setMsTransmissionInterval(100);
        drive = new TeleMecDrive(hardwareMap, 0.4);

        intake = hardwareMap.get(DcMotorEx.class, "penis");

        waitForStart();
        timer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
            timeUtil.update(timer.milliseconds());
            timeUtil.updateGamepads(gamepad1, gamepad2);

            drive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);
            if (gamepad1.back) drive.resetHeading();


            /*
            // Configure driving parameters with buttons
            if (gamepad1.a) drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (gamepad1.b) drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            if (gamepad1.x) drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad1.y) drive.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
             */

            if (gamepad1.a){
                intake.setPower(1);
            }
            if(gamepad1.b){
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }

            telemetry.addData("maxRPMFraction", drive.lfMaxRPMFraction);
            telemetry.addData("Runmode", drive.runMode);
            telemetry.addData("ZeroPowerBehavior", drive.zeroPowerBehavior);
            telemetry.addData("avg loop time (ms)", timeUtil.getAverageLoopTime());
            telemetry.addData("period", timeUtil.getPeriod());
            telemetry.addData("time", timer.seconds());
            telemetry.update();
        }
    }
}