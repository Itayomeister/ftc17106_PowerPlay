package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import org.opencv.core.*;

public class ElementDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat(); // Defining new mat

    // Creating list of possible duck places and a location variable
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }

    private Location location;

    // Creating ROI (region of interest) for the signal cone
    static final Rect SIGNAL_ROI = new Rect(new Point(850, 500), new Point(1150, 700));

    // Creating threshold percentages for clearance
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    static int parkingSpot;

    // Telemetry constructor
    public ElementDetector(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Converting from RGB to HSV (color, intensity, brightness)
        telemetry.addData("In ElementDetector", "class");
        telemetry.update();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // Green Identification
        Scalar greenLowHSV = new Scalar(40, 70, 0);
        Scalar greenHighHSV = new Scalar(90, 255, 255);

        // Red Identification
        Scalar redLowHSV = new Scalar(160, 45, 100);
        Scalar redHighHSV = new Scalar(180, 255, 255);

        // Blue Identification
        Scalar blueLowHSV = new Scalar(100, 100, 100);
        Scalar blueHighHSV = new Scalar(120, 255, 255);

        // Creating the range and a mat for each barcode
        Mat redCheck = mat.submat(SIGNAL_ROI);
        Core.inRange(redCheck, redLowHSV, redHighHSV, redCheck);
        Mat blueCheck = mat.submat(SIGNAL_ROI);
        Core.inRange(blueCheck, blueLowHSV, blueHighHSV, blueCheck);
        Mat greenCheck = mat.submat(SIGNAL_ROI);
        Core.inRange(greenCheck, greenLowHSV, greenHighHSV, greenCheck);

        // Calculating the percentage of identified ducks in each mat
        double blueValue = Core.sumElems(blueCheck).val[0] / SIGNAL_ROI.area() / 255;
        double redValue = Core.sumElems(redCheck).val[0] / SIGNAL_ROI.area() / 255;
        double greenValue = Core.sumElems(greenCheck).val[0] / SIGNAL_ROI.area() / 255;

        // Releasing submats
        redCheck.release();
        blueCheck.release();
        greenCheck.release();

        // Adding telemetry data
        telemetry.addData("Left raw value", (int) Core.sumElems(redCheck).val[0]);
        telemetry.addData("Left percentage", Math.round(redValue * 100) + "%");
        telemetry.addData("Middle raw value", (int) Core.sumElems(blueCheck).val[0]);
        telemetry.addData("Middle percentage", Math.round(blueValue * 100) + "%");
        telemetry.addData("Right raw value", (int) Core.sumElems(greenCheck).val[0]);
        telemetry.addData("Right percentage", Math.round(greenValue * 100) + "%");

        // Creating booleans for team scoring element location
        boolean signalRed = redValue > PERCENT_COLOR_THRESHOLD && redValue > blueValue && redValue > greenValue;
        boolean signalBlue = blueValue > PERCENT_COLOR_THRESHOLD && blueValue > redValue && blueValue > greenValue;
        boolean signalGreen = greenValue > PERCENT_COLOR_THRESHOLD && greenValue > redValue && greenValue > blueValue;

        // Deciding in which rotation the signal is. If signal not detected => parking spot #1
        // parkingPos = 1;
        // Creating color schemes for signal view on application
        Scalar signalColor;
        if (signalGreen) {
            // The green side is facing us
            location = Location.RIGHT;
            signalColor = new Scalar(0, 255, 0);
            parkingSpot = 3;
        } else if (signalBlue) {
            // The blue side is facing us
            location = Location.MIDDLE;
            signalColor = new Scalar(0, 0, 255);
            parkingSpot = 2;
        } else if (signalRed) {
            // The red side is facing us
            location = Location.LEFT;
            signalColor = new Scalar(255, 0, 0);
            parkingSpot = 1;
        } else {
            // The signal has not been found
            location = Location.NOT_FOUND;
            signalColor = new Scalar(0, 0, 0);
            parkingSpot = 1;
        }

        telemetry.addData("Parking spot location: ", location);
        telemetry.addData("Parking spot", parkingSpot);
        telemetry.update();

        // Return original mat to RGB color format
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        // Drawing rectangles for each barcode with matching color
        Imgproc.rectangle(mat, SIGNAL_ROI, signalColor);

        // Returning the mat to the main program
        return mat;
    }

    public Location getLocation() {
        return location;
    }

    public int getParkingSpot() {
        return parkingSpot;
    }

}
