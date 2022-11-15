package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Config
@TeleOp
public class Teleop extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime timer = new ElapsedTime();
    // Hardware
    TeleMecDrive drive;
    Arm arm;
    Lift lift;

    // Other variables
    boolean clawState = false;
    boolean prevClawInput = false;

    boolean prevCyclingModeInput = false;

    boolean extended = false;
    boolean prevExtendedInput = false;

    int activeJunction = 2; // 0,1,2,3 is ground, low, medium, and high respectively

    public static boolean debug = true;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.15);
        arm = new Arm(hardwareMap);
        lift  = new Lift(hardwareMap);

        waitForStart();
        timer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
            timeUtil.update(timer.milliseconds());
            timeUtil.updateGamepads(gamepad1, gamepad2);
            // Drive
            drive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);
            if (gamepad1.share) drive.resetHeading();

            // CLAW CONTROL
            // Rising edge detector controlling a toggle
            if (gamepad1.left_bumper && !prevClawInput){
                clawState = !clawState;
            }
            prevClawInput = gamepad1.left_bumper;

            if (clawState) arm.closeClaw();
            else arm.openClaw();


            // ARM AND LIFT CONTROL
            // Rising edge detector controlling a toggle for cycling mode (sameside and passthrough)
            if (gamepad2.share && !prevCyclingModeInput){
                arm.setMode(!arm.getMode());
            }
            prevCyclingModeInput = gamepad2.share;

            // Rising edge detector controlling a toggle for the extended state
            if ((gamepad1.left_trigger > 0) && !prevExtendedInput){
                extended = !extended;
            }
            prevExtendedInput = (gamepad1.left_trigger > 0);

            // Switch active junction using the D-Pad
            if (gamepad2.dpad_down) activeJunction = 0;
            if (gamepad2.dpad_left) activeJunction = 1;
            if (gamepad2.dpad_up) activeJunction = 2;
            if (gamepad2.dpad_right) activeJunction = 3;

            // Do stuff with those variables we just changed
            if (extended){
                lift.goToJunction(activeJunction);
                arm.goToScore(); // The action of this method depends on the value of "mode" in the arm class
            } else {
                lift.retract();
                arm.goToGrab(); // Similar behavior to "goToScore"
            }
            // Make the lift move
            lift.update();

            // Print telemetry if we want to
            if (debug) {
                telemetry.addData("clawState", clawState);
                telemetry.addData("avg loop time (ms)", timeUtil.getAverageLoopTime());
                telemetry.addData("period", timeUtil.getPeriod());
                telemetry.addData("time", timer.seconds());
                telemetry.update();
            }
        }
    }
}