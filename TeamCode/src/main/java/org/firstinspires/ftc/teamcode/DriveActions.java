package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

public class DriveActions {

    static final double COUNTS_PER_MOTOR_REV = 537.7; //
    static final double GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (Math.PI * WHEEL_DIAMETER);
    static final double TRACK_WIDTH = 16.0;

    AprilTag apriltag = new AprilTag();

    public ElapsedTime runtime1 = new ElapsedTime();
    public ElapsedTime runtime2 = new ElapsedTime();

    public Telemetry telemetry;

    public double forward, right, rotate;
    public long timeouts_ms;
    public double SHORT_SHOT = 0.3;
    public double LONG_SHOT = 0.75;
    public ElapsedTime runtime;

    public DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;
    public DcMotor leftLauncher, rightLauncher;
    public CRServo intakeStage1, intakeStage2, intakeStage3;
    public WebcamName camera;

    public double x, y, z, roll, pitch, yaw, range, bearing, elevation;

    double tagSize = 6.4375;
    AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
            .addTag(20, "Tag20", tagSize, DistanceUnit.INCH)
            .addTag(21, "Tag21", tagSize, DistanceUnit.INCH)
            .addTag(22, "Tag22", tagSize, DistanceUnit.INCH)
            .addTag(23, "Tag23", tagSize, DistanceUnit.INCH)
            .addTag(24, "Tag24", tagSize, DistanceUnit.INCH)
            .build();


    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setTagLibrary(tagLibrary)
            .setLensIntrinsics(629.0, 629.0, 320.0, 240.0)
            .build();

    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor) //add created tag processor
            .setCamera(camera)
            .setCameraResolution(new Size(640,480))
            .enableLiveView(true)
            .build();

    public DriveActions(DcMotor frontLeftWheel, DcMotor frontRightWheel, DcMotor backLeftWheel, DcMotor backRightWheel,
                        CRServo intakeStage1, CRServo intakeStage2, CRServo intakeStage3,
                        DcMotor leftLauncher, DcMotor rightLauncher, WebcamName camera) {

        this.frontLeftWheel = frontLeftWheel;
        this.frontRightWheel = frontRightWheel;
        this.backLeftWheel = backLeftWheel;
        this.backRightWheel = backRightWheel;
        this.intakeStage1 = intakeStage1;
        this.intakeStage2 = intakeStage2;
        this.intakeStage3 = intakeStage3;
        this.leftLauncher = leftLauncher;
        this.rightLauncher = rightLauncher;

    }

    public void drive (double forward, double right, double rotate, long timeouts_ms) {
        resetEncoders();
        runUsingEncoder();

        //desired wheel travel distance
        double flInches = forward + right;
        double frInches = forward - right;
        double blInches = forward - right;
        double brInches = forward + right;

        //add rotation
        double arcInches = Math.PI * TRACK_WIDTH * (rotate / 360.0);
        flInches -= arcInches;
        blInches -= arcInches;
        frInches += arcInches;
        brInches += arcInches;

        //convert to encoders
        int flTarget = (int) (flInches * COUNTS_PER_INCH);
        int frTarget = (int) (frInches * COUNTS_PER_INCH);
        int blTarget = (int) (blInches * COUNTS_PER_INCH);
        int brTarget = (int) (brInches * COUNTS_PER_INCH);

        setTarget(flTarget, frTarget, blTarget, brTarget);
        runToPosition();

        if (timeouts_ms > 0) {
            while (runtime.milliseconds() > 0) {
                if (runtime.milliseconds() >= timeouts_ms) {
                    stopWheel();
                    break;
                }
            }
        }
    }

    public void turn (double rotate, long timeouts_ms) {
        drive(0, 0, rotate, timeouts_ms);
    }

    public void intake(long timeouts_ms) {
        runtime2.reset();
        while (runtime2.milliseconds() < timeouts_ms) {
            startIntake();
            if (runtime2.milliseconds() >= timeouts_ms) {
                stopIntake();
                break;
            }
        }
    }

    public void shoot (double location) {
        if (location == SHORT_SHOT) {
            leftLauncher.setPower(SHORT_SHOT);
            rightLauncher.setPower(SHORT_SHOT);
        }
        else if (location == LONG_SHOT) {
            leftLauncher.setPower(LONG_SHOT);
            rightLauncher.setPower(LONG_SHOT);
        }
    }

    public void adjust (double targetX, double targetY, double targetH) {

        double currentX = getTagPose() [0];
        double currentY = getTagPose() [1];
        double currentH = getTagPose() [2];

        double kP_linear = 0.02;
        double kP_turn = 0.01;

        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        double errorH = targetH - currentH;

        while (Math.abs(errorX) > 0.5 | Math.abs(errorY) > 0.5 | Math.abs(errorH) > 1.0) {
            double strafe = kP_linear * errorX;
            double forward = kP_linear * errorY;
            double turn = kP_turn * errorH;

            drive(forward, strafe, turn, 0);
        }
    }

    public double[] getTagPose() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();

        if (detections.isEmpty()) return null;

        else {
            //AprilTagDetection tag = tagProcessor.getDetections().get(0);
            for (AprilTagDetection tag : detections) {
                //tag is first detected tag
                telemetry.addData("Tag ID", tag.id);

                if (tag.ftcPose != null) {
                    double x = tag.ftcPose.x;
                    double y = tag.ftcPose.y;
                    double z = tag.ftcPose.z;
                    double roll = tag.ftcPose.roll;
                    double pitch = tag.ftcPose.pitch;
                    double yaw = tag.ftcPose.yaw;
                    double bearing = tag.ftcPose.bearing;
                    double elevation = tag.ftcPose.elevation;
                    double range = tag.ftcPose.range;

                    telemetry.addData("XYZ", String.format(Locale.US, "%.2f %.2f %.2f", x, y, z));
                    telemetry.addData("RPY", String.format(Locale.US, "%.2f %.2f %.2f", roll, pitch, yaw));
                    telemetry.addData("RBE", String.format(Locale.US, "%.2f %.2f %.2f", range, bearing, elevation));
                    telemetry.addLine("");

                    return new double[] {x, y, yaw};
                }
            }
            return null;
        }
    }

    public void startWheel(double power) {
        frontLeftWheel.setPower(power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    public void startIntake () {
        intakeStage1.setPower(1);
        intakeStage2.setPower(1);
        intakeStage3.setPower(1);
    }

    public void stopWheel() {
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
    }

    public void stopIntake() {
        intakeStage1.setPower(0);
        intakeStage2.setPower(0);
        intakeStage3.setPower(0);
    }

    public void resetEncoders() {
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTarget(int fl, int fr, int bl, int br) {
        frontLeftWheel.setTargetPosition(fl);
        frontRightWheel.setTargetPosition(fr);
        backLeftWheel.setTargetPosition(bl);
        backRightWheel.setTargetPosition(br);
    }

    public void runToPosition() {
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //sets the runmode to run using encoder
    public void runUsingEncoder() {
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //sets the runmode to run without encoder
    //it will run using a given power and time
    public void runWithoutEncoder() {
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private boolean AreMotorsBusy() {
        return frontLeftWheel.isBusy() | frontRightWheel.isBusy() | backLeftWheel.isBusy() | backRightWheel.isBusy();
    }
}
