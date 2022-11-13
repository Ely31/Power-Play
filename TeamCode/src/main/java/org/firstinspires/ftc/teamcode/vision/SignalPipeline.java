package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.util.Utility;
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

    public static double perfectMagenta = 150;
    public static double perfectGreen = 60;
    public static double perfectBlue = 90;
    public static double colorTolerance = 10;

    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(displayMat);
        // Display focused area
        Imgproc.rectangle(displayMat, topLeft, bottomRight, new Scalar(255,0,0), 2);

        focusedArea = displayMat.submat(focusedAreaRect);
        Imgproc.cvtColor(focusedArea, focusedArea, Imgproc.COLOR_RGB2HSV);

        averageColor = Core.mean(focusedArea);


        return displayMat;
    } // End of processFrame

    // Methods to return useful information from the pipeline
    public Scalar getAverageColor(){
        return averageColor;
    }
    public double getHue(){
        return averageColor.val[0];
    }

    // This is the only one really used from outside this class
    public int getParkPos(){
        int pos = 1;
        if (Utility.withinErrorOfValue(getHue(), perfectMagenta, colorTolerance)) pos = 1;
        if (Utility.withinErrorOfValue(getHue(), perfectGreen, colorTolerance)) pos = 2;
        if (Utility.withinErrorOfValue(getHue(), perfectBlue, colorTolerance)) pos = 3;
        return pos;
    }
}
