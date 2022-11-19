package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Camera;

@Autonomous(group = "test")
public class SignalPipelineTest extends LinearOpMode {
    Camera camera;
    SignalPipeline pipeline = new SignalPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(100);

        camera = new Camera(hardwareMap, pipeline);
        FtcDashboard.getInstance().startCameraStream(camera.webcam, 3);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("hue", pipeline.getHue());
            telemetry.addData("park pos", pipeline.getParkPos());
            telemetry.update();
        }
    }
}
