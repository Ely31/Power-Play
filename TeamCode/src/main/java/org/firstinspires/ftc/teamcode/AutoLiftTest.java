package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.AutoScoringMech;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Autonomous(name="",group="")
public class AutoLiftTest extends LinearOpMode {
    // Pre-init
    AutoScoringMech scoringMech;
    @Override
    public void runOpMode() {
        // Init
        scoringMech = new AutoScoringMech(hardwareMap);
        ElapsedTime repeatTimer = new ElapsedTime();
        waitForStart();
    
        // Pre-run
        repeatTimer.reset();
        while (opModeIsActive()) {
            // Autonomous instructions
            scoringMech.scoreAsync(Lift.highPos);
            scoringMech.updateLift();
            if (repeatTimer.seconds() > 4){
                scoringMech.resetScoringState();
                repeatTimer.reset();
            }
        }
    }
}
