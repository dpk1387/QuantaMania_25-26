package org.firstinspires.ftc.teamcode;

import android.util.Size;

//ftc sdk imports
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//telemetry and camera imports
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//ftc vision imports for ftc apriltag processing
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

public class DriveActions {

    //constants and robot dimensions
    //this is for drive without encoders
    //will plug in odometry later
    static final double COUNTS_PER_MOTOR_REV = 537.7; //encoder ticks per revolution
    static final double GEAR_REDUCTION = 1.0; //ratio
    static final double WHEEL_DIAMETER = 4.0; //wheel diameter

    //conversion factor from inches to encoder ticks
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (Math.PI * WHEEL_DIAMETER);
    //distance between left and right wheel centers
    static final double TRACK_WIDTH = 16.0;

    //object reference for apriltag
    AprilTag apriltag = new AprilTag();

    //timers for controlling movement and intake duration
    public ElapsedTime runtime1 = new ElapsedTime();
    public ElapsedTime runtime2 = new ElapsedTime();

    public Telemetry telemetry;

    //movement control variables
    public double forward, right, rotate;
    public long timeouts_ms;

    //launch power constants
    public double SHORT_SHOT = 0.3;
    public double LONG_SHOT = 0.75;

    //general runtime tracker for timing operations
    public ElapsedTime runtime;

    //hardware mapping
    public DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;
    public DcMotor leftLauncher, rightLauncher;
    public DcMotor intakeStage1, intakeStage3;
    public CRServo intakeStage2;
    public WebcamName camera;

    //variables for apriltag pose data
    public double x, y, z, roll, pitch, yaw, range, bearing, elevation;

    //APRIL TAGS

    //define tag size and create library of field tags
    //more info in AprilTag class
    double tagSize = 6.4375;
    AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
            .addTag(20, "Tag20", tagSize, DistanceUnit.INCH)
            .addTag(21, "Tag21", tagSize, DistanceUnit.INCH)
            .addTag(22, "Tag22", tagSize, DistanceUnit.INCH)
            .addTag(23, "Tag23", tagSize, DistanceUnit.INCH)
            .addTag(24, "Tag24", tagSize, DistanceUnit.INCH)
            .build();

    //create apriltag processor to analyze camera images for tags
    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setTagLibrary(tagLibrary)
            .setLensIntrinsics(629.0, 629.0, 320.0, 240.0)
            .build();

    //vision portal connects the camera feed to the AprilTag processor
    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor) //add created tag processor
            .setCamera(camera)
            .setCameraResolution(new Size(640,480))
            .enableLiveView(true)
            .build();

    //DRIVING
    //assign hardware
    public DriveActions(DcMotor frontLeftWheel, DcMotor frontRightWheel, DcMotor backLeftWheel, DcMotor backRightWheel,
                        DcMotor intakeStage1, CRServo intakeStage2, DcMotor intakeStage3,
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

    //mecanum drive
    public void drive (double forward, double right, double rotate, long timeouts_ms) {
        resetEncoders(); //clear previous encoder position
        runUsingEncoder(); //set all motors to use encoders

        //MECANUM DRIVE
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

        //set target encoder positions for all wheels
        setTarget(flTarget, frTarget, blTarget, brTarget);
        runToPosition(); //make motors move to that position

        //stop if time limit exceeded
        if (timeouts_ms > 0) {
            while (runtime.milliseconds() > 0) {
                if (runtime.milliseconds() >= timeouts_ms) {
                    stopWheel();
                    break;
                }
            }
        }
    }

    //turning in place only
    public void turn (double rotate, long timeouts_ms) {
        drive(0, 0, rotate, timeouts_ms);
    }

    //INTAKE CONTROL
    public void intake(long timeouts_ms) {
        runtime2.reset(); //reset timer for intake

        while (runtime2.milliseconds() < timeouts_ms) {
            startIntake(); //run servos
            if (runtime2.milliseconds() >= timeouts_ms) {
                stopIntake(); //stop servos after timeout
                break;
            }
        }
    }

    //SHOOTING CONTROL
    public void shoot (double location) {
        //select power level based on pre-defined shot type
        if (location == SHORT_SHOT) {
            leftLauncher.setPower(SHORT_SHOT);
            rightLauncher.setPower(SHORT_SHOT);
        }
        else if (location == LONG_SHOT) {
            leftLauncher.setPower(LONG_SHOT);
            rightLauncher.setPower(LONG_SHOT);
        }
    }

    //POSITION ADJUSTMENT USING APRILTAG
    public void adjust (double targetX, double targetY, double targetH) {

        //get current tag pose (x, y, heading)
        double currentX = getTagPose() [0];
        double currentY = getTagPose() [1];
        double currentH = getTagPose() [2];

        //simple proportional control constants
        double kP_linear = 0.02;
        double kP_turn = 0.01;

        //calculate error between current and target positions
        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        double errorH = targetH - currentH;

        //continue adjusting until error is within tolerance
        while (Math.abs(errorX) > 0.5 | Math.abs(errorY) > 0.5 | Math.abs(errorH) > 1.0) {
            double strafe = kP_linear * errorX;
            double forward = kP_linear * errorY;
            double turn = kP_turn * errorH;

            drive(forward, strafe, turn, 0);
        }
    }

    //APRILTAG POSE RETRIEVAL
    public double[] getTagPose() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();

        //return null if there's no tags detected
        if (detections.isEmpty()) return null;

        else {
            //AprilTagDetection tag = tagProcessor.getDetections().get(0);
            for (AprilTagDetection tag : detections) {
                //tag is first detected tag
                telemetry.addData("Tag ID", tag.id);

                if (tag.ftcPose != null) {
                    //extract important positional and rotational values
                    double x = tag.ftcPose.x;
                    double y = tag.ftcPose.y;
                    double yaw = tag.ftcPose.yaw;

                    //log them to telemetry
                    telemetry.addData("XYZ", String.format(Locale.US, "%.2f %.2f %.2f", x, y, z));
                    telemetry.addData("RPY", String.format(Locale.US, "%.2f %.2f %.2f", roll, pitch, yaw));
                    telemetry.addData("RBE", String.format(Locale.US, "%.2f %.2f %.2f", range, bearing, elevation));
                    telemetry.addLine("");

                    //returned simplified pose data for alignment
                    return new double[] {x, y, yaw};
                }
            }
            return null;
        }
    }

    //start driving motors with given power
    public void startWheel(double power) {
        frontLeftWheel.setPower(power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    //intake servos
    public void startIntake () {
        intakeStage1.setPower(1);
        intakeStage2.setPower(1);
        intakeStage3.setPower(1);
    }

    public void reverseIntake() {
        intakeStage1.setPower(-1);
        intakeStage2.setPower(-1);
        intakeStage3.setPower(-1);
    }

    //stop drive motors
    public void stopWheel() {
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
    }

    //stop intake motors
    public void stopIntake() {
        intakeStage1.setPower(0);
        intakeStage2.setPower(0);
        intakeStage3.setPower(0);
    }

    //reset encoders to 0
    public void resetEncoders() {
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //set encoder target positions for all 4 wheels
    public void setTarget(int fl, int fr, int bl, int br) {
        frontLeftWheel.setTargetPosition(fl);
        frontRightWheel.setTargetPosition(fr);
        backLeftWheel.setTargetPosition(bl);
        backRightWheel.setTargetPosition(br);
    }

    //tell motors to move to target encoder positions
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

    //check if any motors are still moving toward target
    public boolean AreMotorsBusy() {
        return frontLeftWheel.isBusy() | frontRightWheel.isBusy() | backLeftWheel.isBusy() | backRightWheel.isBusy();
    }
}
