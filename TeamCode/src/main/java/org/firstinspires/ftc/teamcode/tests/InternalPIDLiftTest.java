package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.InternalPIDLift;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Disabled
@TeleOp(name="",group="test")
public class InternalPIDLiftTest extends LinearOpMode {
    // Pre-init
    InternalPIDLift lift;
    @Override
    public void runOpMode() {
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new InternalPIDLift(hardwareMap);
        lift.setCoefficients(Lift.coeffs);
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            if(gamepad1.dpad_left) lift.retract();
            else if (gamepad1.dpad_up) lift.goToMedium();
            else if (gamepad1.dpad_right) lift.goToHigh();

            lift.update();
            lift.disalayDebug(telemetry);
            telemetry.addData("left power", lift.left.getPower());
            telemetry.update();
        }
    }
}
