package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "Linear Opmode")
//@Disabled
public class Telop extends PlanTechOpMode {

    private boolean reverseIntake = false;
    float angle;

    @Override
    protected void postInit() {
        initServos();
        sleep(1000);
        initOdoMari();
        initOdometry();
    }

    @Override
    public void run() {
        //closeServos();
        runtime.reset();

        telemetry.addData("Say", "Hello Drive");

        waitForStart();

        angle = getGyro();

        while (opModeIsActive()) {

            // ELEVATOR COMMANDS
            if (gamepad2.dpad_up) elevatorGoTo(3, 1);

            else if (gamepad2.dpad_down) elevatorGoTo(1, 1);

            else if (gamepad1.dpad_right) deployElevator(3, 1);

            else if (gamepad1.dpad_up && elevatorMotor.getCurrentPosition() < 1900)
                elevatorMotor.setPower(1);

            else if (gamepad1.dpad_down && elevatorMotor.getCurrentPosition() > 20 && !touchSensor.isPressed())
                elevatorMotor.setPower(-1);

            else elevatorMotor.setPower(0);

            if (elevatorMotor.getPower() < 0 && touchSensor.isPressed()) elevatorMotor.setPower(0);


            // Servo deploy
            if (gamepad1.b || gamepad2.dpad_right) deployServo();

            // Intake (on/reverse/off) a
            if (gamepad1.a) {
                //intakeCycle();
                sleep(500);
                OdoMari.resetAt(0,0);
                odometry.resetAt(0, 0);
            }

            else if (gamepad1.y && !reverseIntake) {
                intakeFront.setPower(reverseintakePower);
                intakeBack.setPower(reverseintakePower);
                reverseIntake = true;
            } else if (reverseIntake){
                intakeOff();
                reverseIntake = false;
            }

            // Caro commands
            if (gamepad1.x) carousel.setPower(caroSpeed);
            else carousel.setPower(0);


            // FYI! Trigger speed is .75
            // moveFreeWithGyro movement
            double turn = (gamepad1.right_trigger - gamepad1.left_trigger) * .5;

            if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                if (turn == 0)
                    lockedAngleMoveFreeWithGyro(gamepad1.right_stick_x * .5, gamepad1.right_stick_y * .5, angle);
                else {
                    moveFreeWithGyro(gamepad1.right_stick_x * .5, gamepad1.right_stick_y * .5, turn * .5);
                    angle = getGyro();
                }
            } else if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                if (turn == 0)
                    lockedAngleMoveFreeWithGyro(gamepad1.left_stick_x * .75, gamepad1.left_stick_y * .75, angle);
                else {
                    moveFreeWithGyro(gamepad1.left_stick_x * .75, gamepad1.left_stick_y * .75, turn);
                    angle = getGyro();
                }
            } else if (turn != 0) {
                move(0, turn * .75, false);
                angle = getGyro();
            } else if (gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0) {
                moveFreeWithGyro(gamepad2.left_stick_x * .5, gamepad1.left_stick_y * .5, gamepad2.right_trigger - gamepad2.left_trigger);
            } else {
                move(0, 0, true);
                angle = getGyro();
            }

            if (gamepad1.options)
                calibrateGyro();

            telemetry.addData("Gyro", "%.2f", -getGyro());
            telemetry.addData("odo gyro","%.2f", OdoMari.getGyro());
            //telemetry.addData("left_stick_x", gamepad1.right_stick_x);
            //telemetry.addData("left_stick_y", gamepad1.right_stick_y);
            //telemetry.addData("Maalit pos", elevatorMotor.getCurrentPosition());
            //telemetry.addData("touch sensor is", touchSensor.isPressed() ? "pressed" : "not pressed");
            telemetry.addData("x, X",  "%.2f, %.2f",OdoMari.x, OdoMari.X);
            telemetry.addData("y, Y",  "%.2f, %.2f", OdoMari.y, OdoMari.Y);
            telemetry.addData("noam X", "%.2f", odometry.x);
            telemetry.addData("noam Y","%.2f", odometry.y);
            telemetry.update();
        }

    }

    @Override
    protected void end() {
        closeAllMotors();
    }
}