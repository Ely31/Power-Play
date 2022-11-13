package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.BasicLift;

@Config
@TeleOp(name="",group="")
public class BasicLiftTest extends LinearOpMode {
    // Pre-init
    BasicLift lift;
    public static double multiplier = 0.1;

    @Override
    public void runOpMode() {
        // Init
        lift = new BasicLift(hardwareMap);
    
        waitForStart();
        // Pre-run
        while (opModeIsActive()) {
            // TeleOp loop

            lift.setLiftPower(gamepad1.left_stick_x * multiplier);

            telemetry.addData("lift power", gamepad1.left_stick_x * multiplier);
            telemetry.update();
        }
    }
}
