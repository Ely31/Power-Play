package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp(name="",group="")
public class ArmTest extends LinearOpMode {
    // Pre-init
    Arm arm;
    @Override
    public void runOpMode() {
        // Init
    arm = new Arm(hardwareMap);
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop

            if (gamepad1.x) {
                arm.closeClaw();
            } else {
                arm.openClaw();
            }

            arm.setPivotPos(gamepad1.right_trigger+.2);


            if (gamepad1.circle) {
                arm.setEndPos(1);
            } else {
                arm.setEndPos(0);
            }
        }
    }
}
