package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;

@TeleOp(group = "test")
public class CaptureImageOpmode extends LinearOpMode {

    Camera camera;
    CaptureImagePipeline pipeline = new CaptureImagePipeline();

    boolean prevInput = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(100);
        camera = new Camera(hardwareMap, pipeline);
    waitForStart();

    while (opModeIsActive()){

        if (gamepad1.a && !prevInput){
            pipeline.saveMatToDisk();
            telemetry.addLine("image saved (in theory)");
        }
        prevInput = gamepad1.a;

        telemetry.addLine("press A to take an image");
        telemetry.addLine("previous images may be overwritten if you aren't careful");
        telemetry.update();
        }
    }
}
