package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import org.opencv.core.*;

public class ElementDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat(); // Defining new mat
    // Creating list of possible duck places and a location variable
    public enum Location{
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }
    private Location location;

    // Creating ROIs (regions of interest) for the 3 barcodes
    static final Rect LEFT_ROI = new Rect(new Point(1500, 500), new Point(1750, 700));
    static final Rect MIDDLE_ROI = new Rect(new Point(850, 500), new Point(1150, 700));
    static final Rect RIGHT_ROI = new Rect(new Point(300, 500), new Point(660, 700));
    // Creating threshold percentages for clearance
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    static int elementPos = 3;
    // Telemetry constructor
    public ElementDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input)  {
        // Converting from RGB to HSV (color, intensity, brightness)
        telemetry.addData("In ElementDetector", "class");
        telemetry.update();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        // Creating color spectrum for the DUCK identification
        //Scalar lowHSV = new Scalar(30, 50, 50);
        //Scalar highHSV = new Scalar(125, 100, 100);

        // Blue Identification
        //Scalar lowHSV = new Scalar(120, 20, 20);
        //Scalar highHSV = new Scalar(300, 100, 100);

        // Green Identification
        Scalar lowHSV = new Scalar(40, 70, 0);
        Scalar highHSV = new Scalar(90, 255, 255);

        // Creating the range and a mat for each barcode
        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        // Calculating the percentage of identified ducks in each mat
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        // Releasing
        left.release();
        middle.release();
        right.release();

        // Adding telemetry data
        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        // Creating booleans for team scoring element location
        boolean elementLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean elementMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean elementRight = rightValue > PERCENT_COLOR_THRESHOLD;

        // Deciding in which position the element is. Element not detected => Level 3
        //elementPos = 3;

        if (elementLeft && leftValue > middleValue && leftValue > rightValue)
            elementPos = 1;
        else if (elementMiddle && middleValue > rightValue && middleValue > leftValue)
            elementPos = 2;
        else if (elementRight && rightValue > middleValue && rightValue > leftValue)
            elementPos = 3;

        if (elementRight) {
            // The element is in the right barcode
            location = Location.RIGHT;
            telemetry.addData("Team scoring element location: ", location);
        }
        else if (elementMiddle) {
            // The element is in the middle barcode
            location = Location.MIDDLE;
            telemetry.addData("Team scoring element location: ", location);
        }
        else if (elementLeft) {
            // The element is in the left barcode
            location = Location.LEFT;
            telemetry.addData("Team scoring element location: ", location);
        }
        else {
            // The element has not been found
            location = Location.NOT_FOUND;
            telemetry.addData("Team scoring element location: ", location);
        }

        telemetry.addData("elementpos", elementPos);
        telemetry.update();

        // Return 2 RGB
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        // Creating color schemes for duck view on barcodes
        Scalar colorFound = new Scalar(0, 255, 0);
        Scalar colorBlank = new Scalar(255, 0, 0);

        // Drawing rectangles for each barcode with matching color
        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorFound:colorBlank);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorFound:colorBlank);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorFound:colorBlank);

        // Returning the mat to the main program
        return mat;
    }

    public  Location getLocation(){ return location;}

    public  int getPos() {return elementPos;}

}
