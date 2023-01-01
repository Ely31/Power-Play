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

    Mat hlsMat = new Mat();
    Mat shrunkMat = new Mat();

    int imageWidth = 30;

    double pixelVal;

    String outPutString;

    @Override
    public Mat processFrame(Mat input) {
        // Convert to hls because html can do hsl
        Imgproc.cvtColor(input, hlsMat, Imgproc.COLOR_RGB2HLS);
        // Scale the image down so we can print each pixel to telemetry and they all fit
        Imgproc.resize(hlsMat, shrunkMat, new Size(imageWidth, imageWidth / CameraConstants.aspectRatio), Imgproc.INTER_AREA);

        pixelVal = shrunkMat.get(1,1)[1];

        outPutString = createPixel(String.valueOf(pixelVal),3);

        telemetry.addLine(outPutString);
        telemetry.update();

        return shrunkMat;
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
