package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;

@TeleOp(group = "test")
public class SignalPipelineTest extends LinearOpMode {
    Camera camera;
    SignalPipeline pipeline = new SignalPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(100);

        camera = new Camera(hardwareMap, pipeline);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("hue", pipeline.getHue());
            telemetry.addData("park pos", pipeline.getParkPos());
            telemetry.update();
        }
    }
}
