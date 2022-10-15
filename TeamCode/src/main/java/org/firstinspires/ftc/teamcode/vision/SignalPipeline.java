package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalPipeline extends OpenCvPipeline {

    private Mat focusedArea = new Mat();
    private Mat displayMat = new Mat();

    private Scalar averageColor = new Scalar(255,0,0);

    private final Point topLeft = new Point(150, 100);
    private final Point bottomRight = new Point(170, 120);
    private final Rect focusedAreaRect = new Rect(topLeft, bottomRight);

    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(displayMat);
        // Display focused area
        Imgproc.rectangle(displayMat, topLeft, bottomRight, new Scalar(255,0,0), 2);

        focusedArea = displayMat.submat(focusedAreaRect);

        averageColor = Core.mean(focusedArea);


        return displayMat;
    } // End of processFrame


    public Scalar getAverageColor(){
        return averageColor;
    }
    public int getAvgColor1(){
        return (int) averageColor.val[0];
    }
    public int getAvgColor2(){
        return (int) averageColor.val[1];
    }
    public int getAvgColor3(){
        return (int) averageColor.val[2];
    }
}
