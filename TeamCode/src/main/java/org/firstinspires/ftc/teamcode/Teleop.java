package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Config
@TeleOp
public class Teleop extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    // Hardware
    TeleMecDrive drive;
    double drivingSpeedMultiplier;
    Arm arm;
    Lift lift;

    // Other variables
    boolean clawState = false;
    boolean prevClawInput = false;

    boolean prevCyclingModeInput = false;

    boolean extended = false;
    boolean prevExtendedInput = false;

    int activeJunction = 2; // 0,1,2,3 is ground, low, medium, and high respectively
    public static double posEditStep = 0.15;
    public static double retractedPosEditStep = 0.07;
    // Telemetry options
    public static boolean debug = true;
    public static boolean instructionsOn = false;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.2);
        arm = new Arm(hardwareMap);
        lift  = new Lift(hardwareMap);

        waitForStart();
        matchTimer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
            timeUtil.update(matchTimer.milliseconds());
            timeUtil.updateGamepads(gamepad1, gamepad2);

            // Relate the max speed of the bot to the height of the lift to prevent tipping
            drivingSpeedMultiplier = 1 - (lift.getHeight() * 0.035);
            // Drive the bot
            drive.driveFieldCentric(
                    gamepad1.left_stick_x * drivingSpeedMultiplier,
                    gamepad1.left_stick_y * drivingSpeedMultiplier,
                    gamepad1.right_stick_x * drivingSpeedMultiplier,
                    gamepad1.right_trigger);
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
            // Switch active junction using the four buttons on gamepad one
            if (gamepad1.cross) activeJunction = 0;
            if (gamepad1.square) activeJunction = 1;
            if (gamepad1.triangle) activeJunction = 2;
            if (gamepad1.circle) activeJunction = 3;

            // Edit the current level with the dpad on gamepad two
            if (gamepad2.dpad_up) lift.editCurrentPos(activeJunction, posEditStep);
            if (gamepad2.dpad_down) lift.editCurrentPos(activeJunction, -posEditStep);

            // Edit retracted pos for grabbing off the stack (this may be a scuffed way of doing it but comp is in two days)
            if (gamepad2.triangle) lift.editRetractedPos(retractedPosEditStep);
            if (gamepad2.cross) lift.editRetractedPos(-retractedPosEditStep);
            if (arm.getMode()) lift.resetRetractedPos();

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

            // Do stuff with all those variables we just changed or tuned
            if (extended){
                lift.goToJunction(activeJunction);
                // Have a special case for the gronud junction
                if (activeJunction == 0){
                    arm.goToScoreGround();
                } else arm.goToScore(); // The action of this method depends on the value of "mode" in the arm class

            } else {
                lift.retract();
                arm.goToGrab(); // Similar behavior to "goToScore"
            }
            // Make the lift move
            lift.update();


            // Print stuff to telemetry if we want to
            if (debug) {
                telemetry.addData("clawState", clawState);
                telemetry.addData("extended", extended);
                lift.disalayDebug(telemetry);
                arm.displayDebug(telemetry);
                telemetry.addData("avg loop time (ms)", timeUtil.getAverageLoopTime());
                telemetry.addData("period", timeUtil.getPeriod());
                telemetry.addData("time", matchTimer.seconds());
            }
            // Someone should be able to learn how to drive without looking at the source code
            if (instructionsOn) {
                telemetry.addLine("Gamepad 1 controls:");
                telemetry.addLine("Driving: Left stick is translation, right stick x is rotation. Use the right trigger to slow down.");
                telemetry.addLine("calibrate feild-centric with the share button after you point the bot with the claw facing towards you");
                telemetry.addLine("Lift and arm: toggle the claw open and closed with the left bumper, and toggle extend/retract with the left trigger");
                telemetry.addLine("Switch levels with buttons cross, square, triangle, and circle.");
                telemetry.addLine("Ground level is cross, levels get higher as you travel clockwise along the four buttons");
                telemetry.addLine();
                telemetry.addLine("Gamepad 2:");
                telemetry.addLine("make fine adjustments to the current height with dpad up and down.");
                telemetry.addLine("The height tweaks made are saved and will persist until the opmode is stopped.");
            }
            telemetry.update();
        }
    }
}