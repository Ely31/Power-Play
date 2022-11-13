package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Lift;

@TeleOp(name="",group="")
public class LiftTest extends LinearOpMode {
    // Pre-init
    Lift lift;
    @Override
    public void runOpMode() {
        // Init
        lift = new Lift(hardwareMap);
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop


            lift.update();
        }
    }
}
