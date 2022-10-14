package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CaptureImagePipeline extends OpenCvPipeline {

    Mat outputMat;
    Mat save;
    int imgIndex = 0;

    String path = "/storage/self/primary/Pictures";

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(outputMat);
        return outputMat;
    }

    public void saveMatToDisk(String name, Mat mat) {
        save = mat.clone();
        Imgproc.cvtColor(mat, save, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite(path + name + imgIndex, save);
        imgIndex ++;
    }
    public void saveMatToDisk( Mat mat) {
        save = mat.clone();
        Imgcodecs.imwrite(path + "capture" + imgIndex, save);
        imgIndex ++;
    }
}
