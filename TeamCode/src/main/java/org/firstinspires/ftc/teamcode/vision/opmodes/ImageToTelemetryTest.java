package org.firstinspires.ftc.teamcode.vision.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.vision.workspace.ImageToTelemetryPipeline;

@TeleOp(group = "test")
public class ImageToTelemetryTest extends LinearOpMode {
    // Pre-init
    Camera camera;
    ImageToTelemetryPipeline pipeline = new ImageToTelemetryPipeline();
    ElapsedTime processImageThrottle = new ElapsedTime();

    int refreshRate = 600; // In milliseconds

    @Override
    public void runOpMode() {
        // Init
        camera = new Camera(hardwareMap, pipeline);
        // Set up telemetry right
        telemetry.setMsTransmissionInterval(refreshRate);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();
        // Pre-run
        processImageThrottle.reset();
        while (opModeIsActive()) {
            // Autonomous instructions
            if (processImageThrottle.milliseconds() > refreshRate) {
                // Write all the rows of pixels out in telemetry
                for (int i = 0; i < pipeline.imageHeight; i++) {
                    telemetry.addLine(pipeline.rowToDisplayPixels(i));
                }
                telemetry.update();
                // Reset the timer so it loops
                processImageThrottle.reset();
            }
        }
    }
}
