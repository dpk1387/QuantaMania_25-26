package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleOp Version 1.0")

public class TeleOpV1 extends LinearOpMode {
    private DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    private DcMotor leftLauncher, rightLauncher;
    private CRServo intakeStage1, intakeStage2, intakeStage3;

    private WebcamName camera;
    private AprilTagProcessor tagProcessor;

    //prevents shooting if too close to goal for shooting
    private double inside_shot = 80;
    private double long_shot = 120;

    private double INSIDE_SHOT_PWR = 0.3;
    private double LONG_SHOT_PWR = 0.75;

    boolean endgame = false;
    boolean shot_override = false;

    String shooting_mode;

    ElapsedTime runtime;

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

        runtime.reset();

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
            runtime.startTime();
            while (opModeIsActive()) {
                telemetry.addData("Intake Power", intakeStage1.getPower() + " " + intakeStage2.getPower() + " " + intakeStage3.getPower());
                telemetry.addData("Shooter Power", leftLauncher.getPower() + " " + rightLauncher.getPower());
                telemetry.addData("Shooting Mode", shooting_mode);
                telemetry.addData("Endgame", endgame);
                telemetry.update();

                double forward = gamepad1.left_stick_y;
                double right = gamepad1.left_stick_x;
                double rotate = gamepad1.right_stick_x;

                drive(forward, right, rotate);

                intake();
                shoot();
                OVERRIDE();

                endgameExpansion();
            }
        }
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

    private void shoot() {
        double distance = getDistance();

        if (gamepad2.left_trigger >= 0.1 || gamepad2.right_trigger >= 0.1) {
            if (distance > long_shot || shot_override) {
                //for shooting in far launch zone
                leftLauncher.setPower(LONG_SHOT_PWR);
                rightLauncher.setPower(LONG_SHOT_PWR);
            }

            else if (distance < inside_shot || shot_override) {
                //for shooting in closer launch zone
                leftLauncher.setPower(INSIDE_SHOT_PWR);
                rightLauncher.setPower(INSIDE_SHOT_PWR);
            }
        }
    }

    private double getDistance() {
        double distance = -1; // backup distance if goal tags not found
        if (tagProcessor.getDetections() != null) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            for (AprilTagDetection tag : detections) {
                if ((tag.id == 20 | tag.id == 24) && tag.ftcPose != null) {
                    // 20 is the blue alliance goal tag
                    // 24 is the red alliance goal tag
                    distance = tag.ftcPose.range;
                }
            }
        }
        return distance;
    }

    private void intake() {
        if (gamepad2.dpad_up) {
            intakeStage1.setPower(1);
            intakeStage2.setPower(1);
            intakeStage3.setPower(1);
        }

        if (gamepad2.dpad_down) {
            intakeStage1.setPower(-1);
            intakeStage2.setPower(-1);
            intakeStage3.setPower(-1);
        }
    }

    private void endgameExpansion() {
        if (gamepad2.x) {
            // extend up to height limit to fit both robots in base
            // for later
        }
    }

    private void OVERRIDE() {
        if (gamepad1.left_bumper && gamepad1.right_bumper) shot_override = !shot_override;
        // if camera doesn't work use this to allow shots

        if (shot_override) shooting_mode = "CAMERA_ENABLED"; else shooting_mode = "CAMERA_DISABLED";

        if (gamepad1.y || runtime.seconds() >= 100) {
            endgame = !endgame;
            // allow expansion during endgame
        }

    }
}
