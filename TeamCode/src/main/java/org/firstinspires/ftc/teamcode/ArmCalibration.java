package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp(name="",group="test")
public class ArmCalibration extends LinearOpMode {
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
            arm.setPivotPos(0.5);
        }
    }
}
