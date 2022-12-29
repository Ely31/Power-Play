package org.firstinspires.ftc.teamcode.vision.workspace;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class JunctionPipeline extends OpenCvPipeline {

    Mat binaryMat = new Mat();
    Mat rgbmat = new Mat();
    Mat displayMat = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();
    private RotatedRect[] rects;
    private double largestRectArea = 0;
    private RotatedRect largestRect = new RotatedRect();
    Point[] largestRectPoints = new Point[4];
    Point highestRectPoint = new Point(CameraConstants.cameraWidth/2.0, CameraConstants.cameraHeight -10);
    Point secondHighestRectPoint = new Point(CameraConstants.cameraWidth/2.0, CameraConstants.cameraHeight -30);
    Point topCenterPoint = new Point();

    // Constant stuff for the cleanMask method
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    VisionUtil vUtil = new VisionUtil(Imgproc.COLOR_RGB2HSV);

    public static Scalar lower = new Scalar(10, 150, 50);
    public static Scalar upper = new Scalar(40, 255, 200);

    // Configuration
    public static boolean openingEnabled = true;

    @Override
    public Mat processFrame(Mat input) {
        // Clear the old contours so they don't stack up
        contours.clear();
        // Reset the largest rect and rect area so it finds the new one
        largestRectArea = 0;
        largestRect = new RotatedRect(new Point(320/2, 10), new Size(10,10), 0);
        // Reset the top two points for the same reason
        resetHighPoints();

        input.copyTo(rgbmat);
        input.copyTo(binaryMat);
        vUtil.prepareFrame(binaryMat);
        // Threshold
        Core.inRange(binaryMat, lower, upper, binaryMat);
        // Opening
        if (openingEnabled) cleanMask(binaryMat, binaryMat);
        displayMat.release();
        // Display what's underneath the thresholded mat
        Core.bitwise_and(rgbmat, rgbmat, displayMat, binaryMat);

        vUtil.getContours(binaryMat, contours);
        contours = vUtil.getFilteredContours(contours, 40, 2000);
        vUtil.drawContours(displayMat, contours, vUtil.red,2);

        // Fit rectangles to those contours we found and draw them all
        rects = new RotatedRect[contours.size()];
        for (int i=0; i < contours.size(); i++){
            rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
            vUtil.drawRotatedRect(rects[i], displayMat, vUtil.blue, 2);
        }

        // Find the largest rect
        for (RotatedRect rect : rects) {
            if (rect.size.height * rect.size.width > largestRectArea) {
                largestRectArea = rect.size.height * rect.size.width;
                largestRect = rect;
            }
        }
        // Draw the largest rect in green
        vUtil.drawRotatedRect(largestRect, displayMat, vUtil.green, 2);

        // Get the verticies of the largest rect
        largestRectPoints = new Point[4];
        largestRect.points(largestRectPoints);
        // Find the highest two
        for (Point largestRectPoint : largestRectPoints) {
            // If higher than the highest, it is the highest
            if (largestRectPoint.y > highestRectPoint.y) {
                highestRectPoint = largestRectPoint;
            }
            // If higher than the second highest but lower than the highest, it is the second highest
            if (largestRectPoint.y < secondHighestRectPoint.y && !(largestRectPoint.y > highestRectPoint.y)) {
                secondHighestRectPoint = largestRectPoint;
            }
        }

        topCenterPoint = calculateMidpoint(highestRectPoint, secondHighestRectPoint);

        Imgproc.circle(displayMat, topCenterPoint, 4, vUtil.white);

        return displayMat;
    }

    // Copied from an old message by NPE on the ftc discord
    void cleanMask(Mat input, Mat output)
    {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void resetHighPoints(){
        highestRectPoint = new Point(CameraConstants.cameraWidth/2.0, CameraConstants.cameraHeight -10);
        secondHighestRectPoint = new Point(CameraConstants.cameraWidth/2.0, CameraConstants.cameraHeight -20);
    }

    Point calculateMidpoint(Point p1, Point p2){
        return (new Point((p1.x + p2.x) /2.0, (p1.y + p2.y) /2.0));
    }
}
