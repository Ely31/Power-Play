package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class ColorTelemetryMessing extends LinearOpMode {
    // Pre-init
    public static String text = "<font color=#04f739> Test: </font>";
    public static String text2 = "<font color =#00ebfc> █ </font>";
    @Override
    public void runOpMode() {
        // Init
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();
        while (opModeIsActive()) {
            // TeleOp loop

            telemetry.addLine(text + text2);
            telemetry.addLine(createPixel("e89d12",0));

            telemetry.update();
        }
    }
    public String createPixel(String color, int weight){
        String glyph = "n";
        switch (weight){
            case 0:
                glyph = "░";
                break;
            case 1:
                glyph = "▒";
                break;
            case 2:
                glyph = "▓";
                break;
            case 3:
                glyph = "█";
                break;
        }
        return "<font color =#" + color + ">" + glyph + "</font>";
    }
}
