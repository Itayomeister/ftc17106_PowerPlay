package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Point;

public class freightDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat(); // Defining new mat

    public enum Location{
        right,
        left,
        middle
    }
    Location location = Location.middle;

    ElapsedTime runtime = new ElapsedTime();
    double time = 0;

    static final Rect RANGE_OF_MOTION = new Rect(new Point(200, 0), new Point(1720, 800));
    final int squareX = 150;
    final int squareY = 150;
    boolean found = false;

    Scalar lowHSV = new Scalar(40, 70, 0);
    Scalar highHSV = new Scalar(90, 255, 255);
    // Creating threshold percentages for clearance
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public freightDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input)  {
        runtime.reset();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat ROI = mat.submat(RANGE_OF_MOTION);
        int i = 0, k = 0;
        for (i = 0; i < RANGE_OF_MOTION.y - squareY && !found; i++){
            for (k = 0; k < RANGE_OF_MOTION.x - squareX && !found; k++){
                Mat miniROI = ROI.submat(new Rect(new Point(k, i), new Point(k + squareX,i + squareY)));
                if(Core.sumElems(miniROI).val[0] > PERCENT_COLOR_THRESHOLD) found = true;
            }
        }

        int x = (k + squareX)/2;
        if (RANGE_OF_MOTION.x/2 - x > 100)
            location = Location.left;
        else if (RANGE_OF_MOTION.x/2 - x < 100)
            location = Location.right;
        else
            location = Location.middle;
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Rect detected = new Rect(new Point(k, i), new Point(k+squareX, i+squareY));
        Imgproc.rectangle(mat, detected, new Scalar(0, 255, 0));

        time = runtime.seconds();
        return mat;
    }

    public double getTime(){ return time; }

}
