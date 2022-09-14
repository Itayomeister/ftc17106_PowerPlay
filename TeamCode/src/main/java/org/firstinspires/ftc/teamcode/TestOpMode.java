package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

import static java.lang.Math.*;

@TeleOp
public class TestOpMode extends PlanTechOpMode {
    double x, y, angle, theta;

    @Override
    protected void postInit() {
        //initServos();
        //initCamera();
    }

    @Override
    protected void run() {
        //setStartingPosition(0, 0);
        //deployElevator(elementPos, 1);
        //goToPosition(0,  10/tileSize, 1);
        //turnToAngle(90);
        waitForStart();
        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        theta = atan(y / x);
        angle = toDegrees(theta);
        lockedAngleMoveFreeWithGyro(x, y, (float) angle);

        telemetry.addData("X, Y", "%.2f, %.2f", x, y);
        telemetry.addData("angle", "%.2f", angle);
        telemetry.addData("gyro", "%.2f", -getGyro());
        telemetry.addData("angle - gyro", "%.2f", abs(angle) - abs(getGyro()));
        telemetry.update();
    }

    @Override
    protected void end() {
        move(0, 0, true);
    }
}
