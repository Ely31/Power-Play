package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;

@TeleOp(group = "test")
public class SignalPipelineTest extends LinearOpMode {
    Camera camera = new Camera();
    SignalPipeline pipeline = new SignalPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(100);

        camera.init(hardwareMap);
        camera.webcam.setPipeline(pipeline);


        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("val1", pipeline.getAvgColor1());
            telemetry.update();
        }
    }
}
