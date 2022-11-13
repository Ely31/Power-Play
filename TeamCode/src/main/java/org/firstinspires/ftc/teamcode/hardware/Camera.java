package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {

    public OpenCvWebcam webcam;

    public Camera(){} // Blank constructor for backwards compatability
    public Camera(HardwareMap hwmap){
        init(hwmap);
    }
    public Camera(HardwareMap hwmap, OpenCvPipeline pipeline){
        init(hwmap);
        setPipeline(pipeline);
    }

    public void init(HardwareMap hwmap) { // Initialization of camera and pipeline
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwmap.get(WebcamName.class, "Webcam 1"));

        webcam.setMillisecondsPermissionTimeout(1000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
    }

    public void stopStreaming(){
        webcam.stopStreaming();
    }
    public void setPipeline(OpenCvPipeline pipeline){
        webcam.setPipeline(pipeline);
    }
}