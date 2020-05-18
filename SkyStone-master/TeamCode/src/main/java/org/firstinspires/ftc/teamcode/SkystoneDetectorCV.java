package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetectorCV extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public enum SkystonePosition
    {
        LEFT,
        RIGHT,
        CENTER,
        NONE
    }
    public SkystonePosition position;

    public SkystoneDetectorCV() {

    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);
/*
        Mat matLeft = workingMatrix.submat(120, 150, 10, 50);
        Mat matCenter = workingMatrix.submat(120, 150, 80, 120);
        Mat matRight = workingMatrix.submat(120, 150, 150, 190);

       phone original
       Mat matLeft = workingMatrix.submat(120, 150, 80, 120);
        Mat matCenter = workingMatrix.submat(120, 150, 150, 190);
        Mat matRight = workingMatrix.submat(120, 150, 220, 260);


        Imgproc.rectangle(workingMatrix, new Rect(80, 120, 40, 30), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(150, 120, 40, 30), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(220, 120, 40, 30), new Scalar(0,255,0));

*/
        Mat matLeft = workingMatrix.submat(85, 115, 45, 75);
        Mat matCenter = workingMatrix.submat(85, 115, 150, 190);
        Mat matRight = workingMatrix.submat(85, 115, 220, 260);


        Imgproc.rectangle(workingMatrix, new Rect(45, 85, 40, 30), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(150, 85, 40, 30), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(220, 85, 40, 30), new Scalar(0,255,0));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if (leftTotal > centerTotal) {
            if (leftTotal > rightTotal) {
                // left is skystone
                position = SkystonePosition.LEFT;
            } else {
                // right is skystone
                position = SkystonePosition.RIGHT;
            }
        } else {
            if (centerTotal > rightTotal) {
                // center is skystone
                position = SkystonePosition.CENTER;
            } else {
                // right is skystone
                position = SkystonePosition.RIGHT;
            }
        }

        return workingMatrix;
    }
}
