package org.firstinspires.ftc.teamcode;

// import android.util.Size;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleOp Version 1.2")

public class TeleOpV1 extends LinearOpMode {
    private DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    private DcMotor shooter;
    private DcMotor intakeStage1, intakeStage2, intakeStage3;
    private Servo blockShooter;
    private Rev2mDistanceSensor distanceSensor;

    private AprilTagProcessor tagProcessor;

    //prevents shooting if too close to goal for shooting
    // private double inside_shot = 80;
    // private double long_shot = 120;

    private double shot_power = 1;

    boolean endgame = false;
    boolean shot_override = false;

    String shooting_mode;

    boolean shooter_active = false;

    ElapsedTime runtime;

    @Override

    public void runOpMode() {

        frontRightWheel = hardwareMap.get(DcMotor.class, "frontRightWheel");
        frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeftWheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "backRightWheel");
        backLeftWheel = hardwareMap.get(DcMotor.class, "backLeftWheel");

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intakeStage1 = hardwareMap.get(DcMotor.class, "stage1");
        intakeStage2 = hardwareMap.get(DcMotor.class, "stage2");
        intakeStage3 = hardwareMap.get(DcMotor.class, "stage3");

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        intakeStage1.setDirection(DcMotor.Direction.REVERSE);
        intakeStage2.setDirection(DcMotor.Direction.REVERSE);
        intakeStage3.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
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
                .build();*/

        waitForStart();

        if (opModeIsActive()) {
            // runtime.startTime();
            while (opModeIsActive()) {
                double forward = gamepad1.left_stick_y;
                double right = gamepad1.left_stick_x;
                double rotate = gamepad1.right_stick_x;

                telemetry.addData("Intake Power", intakeStage1.getPower() + " " + intakeStage2.getPower() + " " + intakeStage3.getPower());
                telemetry.addData("Shooter Power", shooter.getPower());
                telemetry.addData("Shooting Mode", shooting_mode);
                telemetry.addData("Endgame", endgame);
                telemetry.addData("Forward", forward);
                telemetry.addData("Right", right);
                telemetry.addData("Rotate", rotate);
                telemetry.addData("fl fr bl br power",
                        JavaUtil.formatNumber(frontLeftWheel.getPower(), 2) + " " +
                        JavaUtil.formatNumber(frontRightWheel.getPower(), 2) + " " +
                        JavaUtil.formatNumber(backLeftWheel.getPower(), 2) + " " +
                        JavaUtil.formatNumber(backRightWheel.getPower(), 2));
                telemetry.update();

                drive(forward, right, rotate);

                intake();
                shoot();
                OVERRIDE();

                endgameExpansion();
            }
        }
    }

    //MECANUM DRIVE
    private void drive(double forward, double right, double rotate) {
        double MAX_DRIVE_PWR = 0.6;
        frontRightWheel.setPower(Range.clip(forward - right - rotate, -MAX_DRIVE_PWR, MAX_DRIVE_PWR));
        frontLeftWheel.setPower(Range.clip(forward + right + rotate, -MAX_DRIVE_PWR, MAX_DRIVE_PWR));
        backRightWheel.setPower(Range.clip(forward + right - rotate, -MAX_DRIVE_PWR, MAX_DRIVE_PWR));
        backLeftWheel.setPower(Range.clip(forward - right + rotate, -MAX_DRIVE_PWR, MAX_DRIVE_PWR));
    }

    private void shoot() {
        double open = 0;
        double closed = 0.6;
        // double distance = getDistance();
        if (gamepad1.dpad_left) {
            shooter.setPower(shooter.getPower() - 0.05);
        } else if (gamepad1.dpad_right) {
            shooter.setPower(shooter.getPower() + 0.05);
        }

        if (gamepad1.a && !shooter_active) {
            shooter_active = true;
            getDistance();
            shooter.setPower(0.95);
            sleep(500);
            blockShooter.setPosition(open);
        }

        if (gamepad1.b && shooter_active) {
            shooter.setPower(0);
            blockShooter.setPosition(closed);
        }

        if (gamepad1.dpad_up) {
            blockShooter.setPosition(open);
        }

        if (gamepad1.dpad_down) {
            blockShooter.setPosition(closed);
        }
    }

    private double getCameraDistance() {
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

    private double getDistance() {
        double r1 = distanceSensor.getDistance(DistanceUnit.INCH);
        double r2 = distanceSensor.getDistance(DistanceUnit.INCH);
        double r3 = distanceSensor.getDistance(DistanceUnit.INCH);
        double r4 = distanceSensor.getDistance(DistanceUnit.INCH);
        double r5 = distanceSensor.getDistance(DistanceUnit.INCH);
        return (r1 + r2 + r3 + r4 + r5) / 5.0;
    }

    private void intake() {
        if (gamepad2.left_trigger >= 0.1) { //stage 3
            intakeStage3.setPower(1);
        } else {
            intakeStage3.setPower(0);
        }

        if (gamepad2.right_trigger >= 0.1) { //stages 1 and 2
            intakeStage1.setPower(1);
            intakeStage2.setPower(1);
        } else {
            intakeStage1.setPower(0);
            intakeStage2.setPower(0);
        }
    }

    private void endgameExpansion() {
        if (gamepad2.x) {
            // extend up to height limit to fit both robots in base
            // for later
            sleep(100);
        }
    }

    private void OVERRIDE() {
        if (gamepad1.left_bumper && gamepad1.right_bumper) shot_override = !shot_override;
        // if camera doesn't work use this to allow shots

        if (shot_override) shooting_mode = "CAMERA_ENABLED"; else shooting_mode = "CAMERA_DISABLED";

        if (gamepad1.y /*|| runtime.seconds() >= 100*/) {
            endgame = !endgame;
            // allow expansion during endgame
        }
    }
}