package org.firstinspires.ftc.teamcode.vision.workspace;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ImageToTelemetryPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    public ImageToTelemetryPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Mat shrunkMat = new Mat();

    int imageWidth = 30;

    double pixelVal;

    String outPutString;

    @Override
    public Mat processFrame(Mat input) {
        // Scale the image down so we can print each pixel to telemetry and they all fit
        Imgproc.resize(input, shrunkMat, new Size(imageWidth, imageWidth / CameraConstants.aspectRatio), Imgproc.INTER_AREA);

        pixelVal = shrunkMat.get(1,1)[1];

        outPutString = createPixel(String.valueOf(pixelVal));

        telemetry.addLine(outPutString);
        telemetry.update();

        return shrunkMat;
    }

    public String createPixel(String color){
        return "<font color =#" + color + ">█</font>";
    }

}
