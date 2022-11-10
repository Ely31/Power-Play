package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.BasicLift;

@TeleOp(name="",group="")
public class BasicLiftTest extends LinearOpMode {
    // Pre-init

    BasicLift lift;
    
    @Override
    public void runOpMode() {
        // Init

        lift = new BasicLift(hardwareMap);

    
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop

            lift.setLiftPower(gamepad1.left_stick_x);

        }
    }
}
