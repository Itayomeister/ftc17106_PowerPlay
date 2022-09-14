package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class OdoMari {
    private final int DIAMETER = 6, CYCLES_PER_ROTATION = 8192;
    private final double DISTANCE_PER_CYCLE = DIAMETER * PI / CYCLES_PER_ROTATION; // cm per rotation cycle

    private final float HORIZONTAL_ENCODER_POSITION = 11.5f; // vertical distance from the center of the robot of the horizontal encoder, in cm
    private final float VERTICAL_ENCODER_POSITION = 12.5f; // horizontal distance from the center of the robot of the vertical encoders, in cm

    private final float gyroCalibration;
    private final DcMotor horizontalEncoder, verticalEncoder1, verticalEncoder2;
    private final BNO055IMU imu;

    public static volatile double x = 0, y = 0, angle = 0, X = 0, Y = 0;
    public volatile boolean isActive = true;
    private double prevH = 0, prevV1 = 0, prevV2 = 0;
    private double v1 = 0, v2 = 0, h = 0, theta = 0;

    // Needs to make sure the encoders are in the right ports
    public OdoMari(DcMotor verticalEncoder1, DcMotor horizontalEncoder, DcMotor verticalEncoder2, BNO055IMU imu) {

        this.horizontalEncoder = horizontalEncoder;
        this.verticalEncoder1 = verticalEncoder1;
        this.verticalEncoder2 = verticalEncoder2;

        this.imu = imu;

        gyroCalibration = getGyro();

        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalEncoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalEncoder1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalEncoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalEncoder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        new Thread(this::run).start();
    }

    private void run() {
        while (isActive) {
            calculateLocation();
        }
    }

    public void start() {
        prevH = horizontalEncoder.getCurrentPosition();
        prevV1 = -verticalEncoder1.getCurrentPosition();
        prevV2 = verticalEncoder2.getCurrentPosition();
        angle = getGyro();
        if (!isActive) {
            isActive = true;
            new Thread(this::run).start();
        }
    }

    public void resetAt(double x, double y) {
        OdoMari.x = x;
        OdoMari.y = y;
        X = x;
        Y = y;
    }

    public void calculateLocation() {
        // Cycle in teleop and see if there is a built up mistake and if it is the same every time
        // to determine which positions to send the robot off to
        // d stands for delta
        double  dv1, dv2, dh, dx, dy; // basic trigonomath
        double  dtheta, R; // advanced circle and trigo math

        // saving each odometer last position to calculate the delta
        prevH = h;
        prevV1 = v1;
        prevV2 = v2;

        // setting the new values for each encoder and the angle using the gyro
        v1 = -verticalEncoder1.getCurrentPosition() * DISTANCE_PER_CYCLE;
        v2 = verticalEncoder2.getCurrentPosition() * DISTANCE_PER_CYCLE;
        h = horizontalEncoder.getCurrentPosition() * DISTANCE_PER_CYCLE;
        angle = toRadians(getGyro());

        // calculating the change (delta) in each odometer's values
        dv1 = v1 - prevV1;
        dv2 = v2 - prevV2;
        dh = h - prevH;

        // calculating the change in the movement for each axis
        dy = (dv1 + dv2) / 2d;
        dx = dh - HORIZONTAL_ENCODER_POSITION * (dv2 - dv1) / VERTICAL_ENCODER_POSITION;

        // calculating the absolute position on the field
        x += dx * cos(angle) + dy * sin(angle);
        y += dy * cos(angle) - dx * sin(angle);


        // calculations based on radius
        dtheta = (dv1 - dv2) / (2 * VERTICAL_ENCODER_POSITION);
        R = dv2 / dtheta + VERTICAL_ENCODER_POSITION;
        Y += 2 * R * sin(dtheta / 2);
        R = dh / dtheta + HORIZONTAL_ENCODER_POSITION;
        X += 2 * R * sin(dtheta / 2);
    }

    float getGyro() {
        return -((
                imu.getAngularOrientation(EXTRINSIC, XYZ, AngleUnit.DEGREES).thirdAngle
                        - gyroCalibration + 180
        ) % 360 - 180);
    }

    // In case I am stupid and wants to set my own 'angle offset' for the gyro
    float setGyro(double ang) {
        return (
                imu.getAngularOrientation(EXTRINSIC, XYZ, AngleUnit.DEGREES).thirdAngle
                        - gyroCalibration + 180
        ) % 360 - 180 + (float) ang;
    }

    double getAngle() {
        return toDegrees(theta % 360);
    }

    public void stop() {
        isActive = false;
    }

    double angleWrap(double ang) {
        ang = toRadians(ang);
        while (ang < -PI)
            ang += 2 * PI;
        while (ang > PI)
            ang -= 2 * PI;
        return ang;
    }
}