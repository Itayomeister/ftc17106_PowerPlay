package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;
import static java.lang.Math.asin;
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

    Gamepad gamepad1, gamepad2;

    Servo servo1, servo2;
    CRServo crServo1, crServo2;
    DcMotorEx dcMotorEx1, dcMotorEx2;

    protected WebcamName webcamName;
    OpenCvCamera Logitech_C920;

    protected ElapsedTime runtime = new ElapsedTime();

    void initialize() {
        webcamName = hardwareMap.get(WebcamName.class, "logitech_C920");

    }

    @Override
    public void runOpMode() {
        initialize();

        postInit();

        waitForStart();


        if (opModeIsActive()) {
            run();
        }

        end();
    }

    protected void postInit() {

    }

    protected abstract void run();

    protected abstract void end();

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


