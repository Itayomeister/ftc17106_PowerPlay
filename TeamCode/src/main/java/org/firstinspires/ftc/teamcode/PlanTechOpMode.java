package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;
import static java.lang.StrictMath.toDegrees;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public abstract class PlanTechOpMode extends LinearOpMode {
    protected DcMotor intakeFront, intakeBack;
    protected double intakeTime = 0;
    protected double intakePower = .8;
    protected double reverseintakePower = -.5;
    protected DcMotorEx elevatorMotor, DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight;
    protected Servo box, servoHl, servoHr, servoV;
    protected OpenCvWebcam webcam;
    protected WebcamName webcamName;
    protected Odometry odometry;
    protected OdoMari OdoMari;

    protected TouchSensor touchSensor;

    protected ElapsedTime runtime = new ElapsedTime();

    protected double inchToCm = 2.54;

    protected double tileSize = 60.96; // 24 * inchToCm;
    protected double robotLength = 42;
    protected double axisYpos = robotLength / 2 / tileSize;
    protected double robotWidth = 30;
    protected double axisXpos = robotWidth / 2 / tileSize;

    protected double startY = -3 + axisYpos;
    protected double barrierOffset = -3;

    // speed calculations
    protected double wheelSize = 9.6 * PI;
    protected double RPM = 6000;
    protected double drivetrainRPM = RPM / 12;
    protected double drivetrainRPS = drivetrainRPM / 60;
    protected double secondForOneTile = tileSize / (wheelSize * drivetrainRPS);
    protected double milisecForOneTile = secondForOneTile * 1000;

    protected double carouselSize = 15 * inchToCm * PI;
    protected double carouselServoRPS = 180 / 60d;
    protected double timeForDuck = carouselSize / (carouselServoRPS * wheelSize) + 0.25;
    protected int side = 1;
    protected double caroSpeed = -.75 * side;

    protected double // floor heights in tics
            floor1 = 50,
            floor2 = 875,
            floor3 = 1850,
            maalitMaxPos = 1900;

    protected CRServo carousel;

    protected Gamepad.RumbleEffect oneWeightRumble;
    protected Gamepad.RumbleEffect twoWeightRumble;

    protected float gyroCalibration = 0;
    protected BNO055IMU imu;

    protected int cargoBox = 0;

    protected double hubRadius = 9 * inchToCm / tileSize;

    OpenCvCamera Logitech_C920;
    int elementPos = 3;

    protected DigitalChannel red1;
    protected DigitalChannel green1;
    protected DigitalChannel red2;
    protected DigitalChannel green2;
    protected DigitalChannel red3;
    protected DigitalChannel green3;
    protected DigitalChannel red4;
    protected DigitalChannel green4;

    protected boolean prevIntake = false, prevBackIntake = false, intakeOn = false, backIntakeOn = false;


    void initialize() {
        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");
        intakeFront.setDirection(DcMotor.Direction.REVERSE);

        intakeBack = hardwareMap.get(DcMotor.class, "intakeBack");
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorMotor = hardwareMap.get(DcMotorEx.class, "maalit");
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Drivetrain Motors
        DriveFrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DriveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveFrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DriveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DriveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveBackRight = hardwareMap.get(DcMotorEx.class, "BR");
        DriveBackRight.setDirection(DcMotor.Direction.FORWARD);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // CRservo carousel
        carousel = hardwareMap.crservo.get("caro");
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        // Cargo box servo
        box = hardwareMap.get(Servo.class, "servoBox");
        box.setDirection(Servo.Direction.REVERSE);

        // Odometry servos
        servoHl = hardwareMap.get(Servo.class, "odoL");
        servoHl.setDirection(Servo.Direction.REVERSE);
        servoHr = hardwareMap.get(Servo.class, "odoR");
        servoHr.setDirection(Servo.Direction.REVERSE);
        servoV = hardwareMap.get(Servo.class, "odoV");
        servoV.setDirection(Servo.Direction.REVERSE);

        webcamName = hardwareMap.get(WebcamName.class, "Logitech");

        touchSensor = hardwareMap.get(TouchSensor.class, "magnetic");

        oneWeightRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(.5, .5, 1).build();
        twoWeightRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 1).build();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.gyroPowerMode = BNO055IMU.GyroPowerMode.FAST;
        parameters.gyroRange = BNO055IMU.GyroRange.DPS2000;
        imu.initialize(parameters);

        red1 = hardwareMap.get(DigitalChannel.class, "red1");
        red1.setMode(DigitalChannel.Mode.OUTPUT);
        green1 = hardwareMap.get(DigitalChannel.class, "green1");
        green1.setMode(DigitalChannel.Mode.OUTPUT);

        red2 = hardwareMap.get(DigitalChannel.class, "red2");
        red2.setMode(DigitalChannel.Mode.OUTPUT);
        green2 = hardwareMap.get(DigitalChannel.class, "green2");
        green2.setMode(DigitalChannel.Mode.OUTPUT);

        red3 = hardwareMap.get(DigitalChannel.class, "red3");
        red3.setMode(DigitalChannel.Mode.OUTPUT);
        green3 = hardwareMap.get(DigitalChannel.class, "green3");
        green3.setMode(DigitalChannel.Mode.OUTPUT);

        red4 = hardwareMap.get(DigitalChannel.class, "red4");
        red4.setMode(DigitalChannel.Mode.OUTPUT);
        green4 = hardwareMap.get(DigitalChannel.class, "green4");
        green4.setMode(DigitalChannel.Mode.OUTPUT);

        calibrateGyro();

        telemetry.addLine("Waiting for start");
        telemetry.update();

    }

    @Override
    public void runOpMode() {
        initialize();

        postInit();

        waitForStart();

        elementPos = ElementDetector.elementPos;
        Log.e("ElementPOS", "" + elementPos);

        if (opModeIsActive()) {
            run();
        }

        odometry.stop();
        closeAllMotors();
        closeServos();

        end();
    }

    protected void postInit() {
        initOdometry();
        initCamera();
    }

    protected abstract void run();

    protected abstract void end();

    //ALL SERVOS METHODS n FUNCTIONS
    public void deployServo() {
        box.setPosition(1);
        sleep(550);
        box.setPosition(0);
        /*cargoBox = 0;
        redLED.setState(false);
        greenLED.setState(false);*/
    }

    public void closeServos() {
        servoHl.setPosition(1);
        servoHr.setPosition(1);
        servoV.setPosition(.75);
    }

    public void initServos() {
        servoHl.setPosition(0);
        servoHr.setPosition(0);
        servoV.setPosition(1);
        box.setPosition(0);
    }

    // ALL INTAKE BASIC METHODS n FUNCTIONS
    public void intakeCycle() {
        if (intakeFront.isBusy()) return;
        red2.setState(false);
        green2.setState(true);
        new Thread(() -> {
            double time = runtime.milliseconds();
            intakeFront.setPower(intakePower);
            intakeBack.setPower(intakePower);
            while (runtime.milliseconds() - time < 1500) {
                idle();
            }
            intakeBack();
        }).start();
    }

    public void intakeBack() {
        red2.setState(false);
        green2.setState(false);
        intakeFront.setPower(reverseintakePower);
        intakeBack.setPower(reverseintakePower);
        double time = runtime.milliseconds();
        while (runtime.milliseconds() - time < 1000) {
            move(0, 0, false);
        }
        intakeOff();
    }

    public void intakeOff() {
        red2.setState(true);
        green2.setState(false);
        intakeFront.setPower(0);
        intakeBack.setPower(0);
    }

    public void elevatorGoTo(int floor, double pow) {
        runInBackround(new Runnable() {
            @Override
            public void run() {
                double tics = 1700;
                // decide which position to send the elevator to
                switch (floor) {
                    case 1:
                        tics = floor1;
                        break;
                    case 2:
                        tics = floor2;
                        break;
                    case 3:
                        tics = floor3;
                        break;
                }
                // getting the elevator to the target position
                elevatorMotor.setTargetPosition((int) tics);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setPower(pow);
            }
        });

    }

    public void deployElevator(int floor, double pow) {
        elevatorGoTo(floor, pow);
        while (elevatorMotor.isBusy() && elevatorMotor.getCurrentPosition() < maalitMaxPos) {
            idle();
        }

        // deploying freight
        deployServo();

        // closing elevator
        elevatorGoTo(1, .85);
        while (elevatorMotor.isBusy() && elevatorMotor.getCurrentPosition() > 0 && !touchSensor.isPressed()) {
            idle();
        }
        elevatorMotor.setPower(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ALL OF LED FUNCTIONS
    public void cargoLedIndicator(int cargo) {
        switch (cargo) {
            case 0:
                red1.setState(true);
                green1.setState(true);
                red4.setState(true);
                green4.setState(true);
            case 1:
                red1.setState(true);
                green1.setState(false);
                red4.setState(true);
                green4.setState(false);
            case 2:
                red1.setState(false);
                green1.setState(false);
                red4.setState(false);
                red4.setState(false);
                break;
            case 3:
                red1.setState(false);
                green1.setState(true);
                red4.setState(false);
                green4.setState(true);
        }
    }


    void driftWithGyro(double pow, double turn) {
        driftWithGyro(pow, turn, 0);
    }

    void driftWithGyro(double pow, double turn, float angle) {
        double frontLeftPower, frontRightPower, backRightPower, backLeftPower;
        backLeftPower = frontRightPower = -pow - turn;
        backRightPower = frontLeftPower = -pow + turn;
        double s = normalizeAngle(getGyro() - angle) / 90d;
        frontRightPower -= s;
        backRightPower -= s;
        frontLeftPower += s;
        backLeftPower += s;
        DriveFrontRight.setPower(Range.clip(frontRightPower, -1, 1));
        DriveBackRight.setPower(Range.clip(backRightPower, -1, 1));
        DriveFrontLeft.setPower(Range.clip(frontLeftPower, -1, 1));
        DriveBackLeft.setPower((Range.clip(backLeftPower, -1, 1)));
    }

    void waitWhile(Condition condition) {
        while (condition.condition() && opModeIsActive()) {
            idle();
        }
    }

    void move(double pow, double turn, boolean isDrift) {
        double frontLeftPower, frontRightPower, backRightPower, backLeftPower;
        if (isDrift) {
            backLeftPower = frontRightPower = Range.clip(pow + turn, -1.0, 1.0);
            backRightPower = frontLeftPower = Range.clip(pow - turn, -1.0, 1.0);
        } else {
            backLeftPower = frontLeftPower = Range.clip(pow + turn, -1.0, 1.0);
            backRightPower = frontRightPower = Range.clip(pow - turn, -1.0, 1.0);
        }

        DriveFrontRight.setPower(frontRightPower);
        DriveBackRight.setPower(backRightPower);
        DriveFrontLeft.setPower(frontLeftPower);
        DriveBackLeft.setPower(backLeftPower);

    }

    float getGyro() {
        return (imu.getAngularOrientation(EXTRINSIC, XYZ, AngleUnit.DEGREES).thirdAngle - gyroCalibration + 180) % 360 - 180;
    }

    void calibrateGyro() {
        gyroCalibration = (imu.getAngularOrientation().firstAngle + 180) % 360 - 180;
    }

    void setGyro(double angle) {
        gyroCalibration = (imu.getAngularOrientation().firstAngle + 180) % 360 - 180 + (float) angle;
    }

    interface Condition {
        boolean condition();
    }

    interface Corrector {
        double[] correct();
    }

    void runInBackground(Runnable runnable) {
        new Thread(runnable).start();
    }


    private float normalizeAngle(float angle) {
        return (angle + 540) % 360 - 180;
    }

    private int normalizeAngle(int angle) {
        return (angle + 540) % 360 - 180;
    }

    void closeAllMotors() {
        DriveFrontRight.setPower(0);
        DriveFrontLeft.setPower(0);
        DriveBackRight.setPower(0);
        DriveBackLeft.setPower(0);
        carousel.setPower(0);
        elevatorMotor.setPower(0);
        intakeFront.setPower(0);
        intakeBack.setPower(0);
    }

    void turnToAngle(int angle) {
        turnToAngle(angle, this::opModeIsActive);
    }

    void turnToAngle(int angle, Condition condition) {
        angle *= side;
        float delta = normalizeAngle(getGyro() - angle);
        float pow;
        double time = runtime.milliseconds();

        while ((Math.abs(delta) > 1.5 || runtime.milliseconds() - time < 100) && condition.condition()) {
            delta = normalizeAngle(getGyro() - angle);
            if (Math.abs(delta) > 1.5) {
                pow = delta * delta / 90 / 90;
                if (delta < 0)
                    pow = -pow;

                double actualPow = clip(pow, -1, -0.18, 1, 0.18);

                Log.e("Pow", String.valueOf(actualPow));

                move(0, actualPow, false);
                time = runtime.milliseconds();
            } else {
                move(0, 0, true);
            }
        }

        move(0, 0, true);

    }

    final protected void moveFreeWithGyro(double x, double y, double turn) {
        // this method takes the x and y coordinates of a joystick, and calculating new x and y that
        // will move the robot in the right direction from the driver's frame of reference instead
        // of the robot's

        // we do that by taking the position of the joystick, converting it to angle and length then
        // calculating the position of a `virtual joystick` that has the same length but with the
        // the angle of the original minus the robot's angle. this way, it actually converts the
        // motion from being at the robot's frame of reference, to being at the driver's

        // the calculation itself goes like this:
        //      we calculate the distance from the origin using the pythagorean theorem,
        //          i.e. d = √(x² + y³)
        //      the angle of the joystick is calculated using the TAN trigonometry function,
        //          taking into account spacial cases (α = 90k: x == 0, y == 0)
        //      then the `virtual joystick`'s angle is calculating by subtracting the gyro angle
        //          from the angle we got
        //      and the actual coordinates are calculated by using, again, the pythagorean theorem
        //          and the trigonometry function TAN, we get two equations with two variables:
        //              x² + y² = d², and x / y = tan(α), where x and y are the new coordinates,
        //              d is the distance and α is the new angle. if you work it out, you get:
        //              x = d / √(tan(α)² + 1), y = tan(α) * x
        //              (still though, we need to take edge cases into account, where TAN is not defined)

        double pow, drift, virtualAngle, powToDriftRatio, realAngle, d = sqrt(x * x + y * y);
        double frontRightPower, frontLeftPower, backLeftPower, backRightPower;

        x = -x;

        if (x == 0) {
            realAngle = y > 0 ? 90 : -90;
        } else if (y == 0) {
            realAngle = x > 0 ? 0 : 180;
        } else {
            realAngle = toDegrees(atan(y / x));
            if (x < 0) {
                realAngle = 180 + realAngle;
            }
        }

        virtualAngle = realAngle - getGyro() - 180;

        virtualAngle = (virtualAngle + 360) % 360;

        powToDriftRatio = tan(toRadians(virtualAngle));

        drift = d / sqrt(powToDriftRatio * powToDriftRatio + 1);
        pow = drift * powToDriftRatio;

        // distinguish between both solutions to our equations
        if (virtualAngle > 90 && virtualAngle <= 270) {
            pow = -pow;
        } else {
            drift = -drift;
        }

        backLeftPower = frontRightPower = pow + drift;
        backRightPower = frontLeftPower = pow - drift;
        frontRightPower = Range.clip(frontRightPower - turn, -1, 1);
        backRightPower = Range.clip(backRightPower - turn, -1, 1);
        frontLeftPower = Range.clip(frontLeftPower + turn, -1, 1);
        backLeftPower = Range.clip(backLeftPower + turn, -1, 1);

        DriveFrontRight.setPower(frontRightPower);
        DriveBackRight.setPower(backRightPower);
        DriveFrontLeft.setPower(frontLeftPower);
        DriveBackLeft.setPower(backLeftPower);
    }

    final protected void lockedAngleMoveFreeWithGyro(double x, double y, float angle) {
        double pow, drift, virtualAngle, powToDriftRatio, realAngle, d = sqrt(x * x + y * y);
        double frontRightPower, frontLeftPower, backLeftPower, backRightPower;

        x = -x;

        if (x == 0) {
            realAngle = y > 0 ? 90 : -90;
        } else if (y == 0) {
            realAngle = x > 0 ? 0 : 180;
        } else {
            realAngle = toDegrees(atan(y / x));
            if (x < 0) {
                realAngle = 180 + realAngle;
            }
        }

        virtualAngle = realAngle - getGyro() - 180;

        virtualAngle = (virtualAngle + 360) % 360;

        powToDriftRatio = tan(toRadians(virtualAngle));

        drift = d / sqrt(powToDriftRatio * powToDriftRatio + 1);
        pow = drift * powToDriftRatio;

        // distinguish between both solutions to our equations
        if (virtualAngle > 90 && virtualAngle <= 270) {
            pow = -pow;
        } else {
            drift = -drift;
        }

        backLeftPower = frontRightPower = pow + drift;
        backRightPower = frontLeftPower = pow - drift;

        double s = normalizeAngle((getGyro() - angle)) / 100d;

        frontRightPower = Range.clip(frontRightPower - s, -1, 1);
        backRightPower = Range.clip(backRightPower - s, -1, 1);
        frontLeftPower = Range.clip(frontLeftPower + s, -1, 1);
        backLeftPower = Range.clip(backLeftPower + s, -1, 1);

        DriveFrontRight.setPower(frontRightPower);
        DriveBackRight.setPower(backRightPower);
        DriveFrontLeft.setPower(frontLeftPower);
        DriveBackLeft.setPower(backLeftPower);
    }


    protected double clip(double num, double absMin, double zeroMin, double absMax, double zeroMax) {
        if (0 < num) {
            if (num < zeroMax) {
                return zeroMax;
            } else if (num > absMax) {
                return absMax;
            }
        } else if (0 > num) {
            if (num > zeroMin) {
                return zeroMin;
            } else if (num < absMin) {
                return absMin;
            }
        }

        return num;
    }

    // ALL ODOMETRY FUNCTIONS AND METHODS: goToPos & goToStat

    public void initOdometry() {
        odometry = new Odometry(DriveBackLeft, DriveFrontLeft, DriveBackRight, imu);
        odometry.resetAt(0, 0);
    }

    public void initOdoMari() {
        OdoMari = new OdoMari(DriveBackLeft, DriveFrontLeft, DriveBackRight, imu);
        OdoMari.resetAt(0, 0);
    }

    protected final void setStartingPosition(double x, double y) {
        odometry.resetAt(x * tileSize * side, y * tileSize);
    }

    protected final void setStartingPositionMari(double x, double y) {
        OdoMari.resetAt(x * tileSize * side, y * tileSize);
    }

    // goToPos
    protected final boolean goToPosition(final double x, final double y, final double maxPow) {
        return goToPosition(x, y, maxPow, null, () -> !opModeIsActive());
    }

    protected final boolean goToPosition(final double x, final double y, final double maxPow, final Integer angle) {
        return goToPosition(x, y, maxPow, angle, () -> !opModeIsActive());
    }

    protected final boolean goToPosition(double x, double y, double maxPow,
                                         Integer arrivalAngle, Condition stopIf) {

        x *= tileSize * side;
        y *= tileSize;

        arrivalAngle = arrivalAngle == null ? (int) getGyro() : arrivalAngle;
        while ((Math.abs(Odometry.x - x) > 3 || Math.abs(Odometry.y - y) > 3)) {
            double pow = Odometry.y - y,
                    drift = x - Odometry.x;

            pow = pow < 0 ? -pow * pow : pow * pow;
            drift = drift < 0 ? -drift * drift : drift * drift;

            pow = clip(pow / 1500, -maxPow, -0.125, maxPow, 0.125);
            drift = clip(drift / 1500, -maxPow, -0.125, maxPow, 0.125);


            if (arrivalAngle == null) {
                driftWithGyro(pow, drift);
            } else {
                moveFreeWithGyro(drift, pow, normalizeAngle((getGyro() - arrivalAngle)) / 100);
            }

            if (stopIf.condition()) {
                return false;
            }
        }

        move(0, 0, false);
        return true;
    }

    protected final boolean goToPositionMari(double x, double y, double maxPow,
                                         Integer arrivalAngle, Condition stopIf) {

        x *= tileSize * side;
        y *= tileSize;

        arrivalAngle = arrivalAngle == null ? (int) getGyro() : arrivalAngle;
        while ((Math.abs(OdoMari.x - x) > 3 || Math.abs(OdoMari.y - y) > 3)) {
            double pow = OdoMari.y - y,
                    drift = x - OdoMari.x;

            pow = pow < 0 ? -pow * pow : pow * pow;
            drift = drift < 0 ? -drift * drift : drift * drift;

            pow = clip(pow / 1500, -maxPow, -0.125, maxPow, 0.125);
            drift = clip(drift / 1500, -maxPow, -0.125, maxPow, 0.125);


            if (arrivalAngle == null) {
                driftWithGyro(pow, drift);
            } else {
                moveFreeWithGyro(drift, pow, normalizeAngle((getGyro() - arrivalAngle)) / 100);
            }

            if (stopIf.condition()) {
                return false;
            }
        }

        move(0, 0, false);
        return true;
    }
    // goToStation
    protected final boolean goToStation(final double x, final double y, final double maxPow) {
        return goToStation(x, y, maxPow, null, () -> !opModeIsActive());
    }

    protected final boolean goToStation(final double x, final double y, final double maxPow, final Integer angle) {
        return goToStation(x, y, maxPow, angle, () -> !opModeIsActive());
    }

    protected final boolean goToStation(double x, double y, double maxPow,
                                        Integer arrivalAngle, Condition stopIf) {
        x *= tileSize * side;
        y *= tileSize;

        arrivalAngle = arrivalAngle == null ? (int) getGyro() : arrivalAngle;
        while ((Math.abs(Odometry.x - x) > 10 || Math.abs(Odometry.y - y) > 10)) {
            double pow = Odometry.y - y,
                    drift = x - Odometry.x;

            pow = pow < 0 ? -pow * pow : pow * pow;
            drift = drift < 0 ? -drift * drift : drift * drift;

            pow = clip(pow / 1500, -maxPow, -0.125, maxPow, 0.125);
            drift = clip(drift / 1500, -maxPow, -0.125, maxPow, 0.125);


            if (arrivalAngle == null) {
                driftWithGyro(pow, drift);
            } else {
                moveFreeWithGyro(drift, pow, normalizeAngle((getGyro() - arrivalAngle)) / 100);
            }

            if (stopIf.condition()) {
                return false;
            }
        }

        move(0, 0, false);
        return true;
    }

    // AUTO FUNCTIONS AND METHODS
    public void autoDucks() {
        // side == 1? red : blue
        setStartingPosition(-1.5, startY);
        runtime.reset();

        // Deploying the preload
        goToDeploy(-1 + hubRadius / 2 - 2.5 / tileSize, -1 - hubRadius / 2 - axisYpos - 2.5 / tileSize, .8, elementPos, -45);

        // Spinning the carousel
        caroSpin(caroSpeed);

        // Collecting the duck
        collectDuck();

        // Moving to the shipping hub from the side
        goToStation(-2, -2 + axisXpos, 1); // moving towards y=-1

        goToStation(-2.5, -1 + axisXpos, 1); // trying to avoid the team element

        goToDeploy(-1, -1 + axisXpos, .8, 3, -90);

        // Going to the storage unit
        park(false);
    }

    public void autoCycle() {
        setStartingPosition(.5, -3 + axisYpos);
        runtime.reset();
        double time = 0;
        /*goToPosition(.5, -3 + 10 / tileSize, 1);
        odometry.stop();
        turnToAngle(90);
        odometry.start();
        cycle(true);
        cycle(false);*/
        cycle(true);
        park(true);
        telemetry.addData("time", runtime.seconds());
        telemetry.update();
        sleep(27500 - (long) runtime.milliseconds());

        /*for (int i = 0; i < 3; i++) {
            time = runtime.seconds();
            cycle();
            telemetry.addData("time for cycle", runtime.seconds() - time);
        }*/
    }

    public void goToDeploy(double x, double y, double maxPow) {
        goToDeploy(x, y, maxPow, 3, (int) getGyro(), 0);
    }

    public void goToDeploy(double x, double y, double maxPow, int floor) {
        goToDeploy(x, y, maxPow, floor, (int) getGyro(), 0);
    }

    public void goToDeploy(double x, double y, double maxPow, int floor, int angle) {
        goToDeploy(x, y, maxPow, floor, angle, 0);
    }

    public void goToDeploy(double x, double y, double maxPow, int floor, int prevAng, int finalAng) {
        /*goToPosition(x, y, maxPow);
        if (getGyro() != prevAng) {
            odometry.stop();
            turnToAngle(prevAng);
            odometry.start();
        }
        deployElevator(floor, 1);
        if (getGyro() != finalAng) {
            odometry.stop();
            turnToAngle(finalAng);
            odometry.start();
        }*/
        elevatorGoTo(floor, 1);
        goToPosition(x, y, maxPow, prevAng);
        deployServo();
        elevatorGoTo(1, 1);
    }

    public void caroSpin(double caroSpeed) {
        goToStation(-2.25, barrierOffset, 1, -90);
        goToPosition(-2.45, barrierOffset, .35, -90);
        double time = runtime.milliseconds();
        while (runtime.milliseconds() - time < 2500 && opModeIsActive())
            carousel.setPower(caroSpeed);
        carousel.setPower(0);
    }

    public void collectDuck() {
        goToPosition(-2.3, barrierOffset, .35, -90);
        intakeFront.setPower(intakePower);
        intakeBack.setPower(intakePower);
        turnToAngle(0);
        intakeOff();
    }

    public void park(boolean warehouse) {
        if (warehouse) {
            turnToAngle(0);
            return;
        }
        goToPosition(-3 + axisYpos + 5 / tileSize, -1.5 + axisXpos, 1, 0);
    }

    public void cycle(boolean first) {
        //GoTo starting position
        if (!first) goToStation(.5, barrierOffset, 1, 90);

        //Deploying preload at target floor / high floor
        goToDeploy(0, -1 - hubRadius - axisYpos - 10 / tileSize, 1, first ? elementPos : 3, 45);
        goToStation(.5, barrierOffset, 1, 90);
        double intakeTime = runtime.seconds();
        goToStation(2 - axisYpos, barrierOffset, 1, 90);
        telemetry.addData("intake time", runtime.seconds() - intakeTime);
    }

    public void runInBackround(Runnable runnable) {
        new Thread(runnable).start();
    }

    public void initCamera() {
        Logitech_C920 = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        ElementDetector detector = new ElementDetector(telemetry);
        Logitech_C920.setPipeline(detector);
        Logitech_C920.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Logitech_C920.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }


}