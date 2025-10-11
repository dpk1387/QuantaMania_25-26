package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Blue_Short Auto")

public class Blue_Short extends LinearOpMode {

    private WebcamName camera;

    private DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    private DcMotor leftLauncher, rightLauncher;
    private CRServo intakeStage1, intakeStage2, intakeStage3;

    boolean endgame = false;

    double SHORT_SHOT = 0.3;
    double LONG_SHOT = 0.75;

    DriveActions driver = new DriveActions (frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
            intakeStage1, intakeStage2, intakeStage3, leftLauncher, rightLauncher, camera);

    @Override

    public void runOpMode() {

        frontRightWheel = hardwareMap.get(DcMotorEx.class, "backLeftWheel");
        frontLeftWheel = hardwareMap.get(DcMotorEx.class, "frontLeftWheel");
        backRightWheel = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
        backLeftWheel = hardwareMap.get(DcMotorEx.class, "backLeftWheel");

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        intakeStage1 = hardwareMap.get(CRServo.class, "stage1");
        intakeStage2 = hardwareMap.get(CRServo.class, "stage2");
        intakeStage3 = hardwareMap.get(CRServo.class, "stage3");

        leftLauncher.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.REVERSE);
        intakeStage1.setDirection(CRServo.Direction.REVERSE);
        intakeStage2.setDirection(CRServo.Direction.REVERSE);
        intakeStage3.setDirection(CRServo.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(629.0, 629.0, 320.0, 240.0)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor) //add created tag processor
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();

        waitForStart();

        if (opModeIsActive()) {
            driver.drive(-60, 0, 0, 1000);
            driver.adjust(-60, 0, 0);
            ///forward and right are inches, rotate is degrees, need to fix later
            SHOOT();
            driver.turn(-45, 1000);
            driver.drive(-40, 0, 0, 1000);
            sleep(100);
            driver.drive(40, 0, 0, 1000);
            driver.turn(45, 1000);
            SHOOT();
        }
    }

    private void SHOOT() {
        //driver.adjust(0, 60, 0);
        //driver.intake(1000); //don't need until we have 4+ artifacts in auto
        driver.shoot(SHORT_SHOT);
        driver.intake(15000); //intake is on while we are shooting
    }

    private void drive(double forward, double right, double rotate) {
        telemetry.addData("Forward", forward);
        telemetry.addData("Right", right);
        telemetry.addData("Rotate", rotate);
        telemetry.update();

        frontRightWheel.setPower(forward - right - rotate);
        frontLeftWheel.setPower(forward + right + rotate);
        backRightWheel.setPower(forward + right - rotate);
        backLeftWheel.setPower(forward - right + rotate);
    }
}