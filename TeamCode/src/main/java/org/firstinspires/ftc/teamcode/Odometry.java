package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Odometry {
    private final int DIAMETER = 6, CYCLES_PER_ROTATION = 8192;
    private final double DISTANCE_PER_CYCLE = DIAMETER * PI / CYCLES_PER_ROTATION; // cm per rotation cycle
    private final float HORIZONTAL_ENCODER_POSITION = 12.5f; // vertical distance from the center of the robot, in cm
    private final float gyroCalibration;
    private final DcMotor horizontalEncoder, verticalEncoder1, verticalEncoder2;
    private final BNO055IMU imu;
    public static volatile double x = 0, y = 0;
    private volatile double prevAngle = 0;
    public volatile boolean isActive = true;
    private double lastH = 0, lastV = 0;

    // Needs to make sure the encoders are in the right ports
    public Odometry(DcMotor verticalEncoder1,
                    DcMotor horizontalEncoder, DcMotor verticalEncoder2, BNO055IMU imu) {

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
        lastH = horizontalEncoder.getCurrentPosition();
        lastV = (-verticalEncoder1.getCurrentPosition() + verticalEncoder2.getCurrentPosition()) / 2d;
        if (!isActive) {
            isActive = true;
            new Thread(this::run).start();
        }

    }

    public void resetAt(double x, double y) {
        Odometry.x = x;
        Odometry.y = y;
    }

    public void calculateLocation() {
        double angle = -getGyro();
        double dh, dv, dx, dy;//, da = (angle - prevAngle + 180) % 360 - 180;

        // prevAngle = angle;
        // check DA!

        angle = toRadians(angle);

        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        dh = horizontalEncoder.getCurrentPosition() - lastH;
        dv = (-verticalEncoder1.getCurrentPosition() + verticalEncoder2.getCurrentPosition()) / 2d - lastV;


        lastH += dh;
        lastV += dv;

        /*if (da != 0)
            dh -= (180 / da) * Math.PI * HORIZONTAL_ENCODER_POSITION;*/


        dh *= DISTANCE_PER_CYCLE;
        dv *= DISTANCE_PER_CYCLE;

        dx = dh * cos + dv * sin;
        dy = dv * cos - dh * sin;

        x += dx;
        y += dy;
    }

    float getGyro() {
        return (
                imu.getAngularOrientation(EXTRINSIC, XYZ, AngleUnit.DEGREES).thirdAngle
                        - gyroCalibration + 180
        ) % 360 - 180;
    }

    public void stop() {
        isActive = false;
    }
}