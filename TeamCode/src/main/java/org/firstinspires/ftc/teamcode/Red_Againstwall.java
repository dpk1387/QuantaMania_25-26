package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Blue Long Auto")

public class Blue_Long extends LinearOpMode {

    private WebcamName camera;

    private DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    private DcMotor shooter;
    private DcMotor intakeStage1, intakeStage2, intakeStage3;
    private Servo blockShooter;

    boolean endgame = false;

    double SHORT_SHOT = 0.3;
    double LONG_SHOT = 0.75;

    DriveActions driver = new DriveActions(frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
            intakeStage1, intakeStage2, intakeStage3, shooter, blockShooter);

    @Override

    public void runOpMode() {

        frontRightWheel = hardwareMap.get(DcMotor.class, "backLeftWheel");
        frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeftWheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "frontRightWheel");
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

        shooter.setDirection(DcMotor.Direction.REVERSE);
        intakeStage1.setDirection(CRServo.Direction.REVERSE);
        intakeStage2.setDirection(CRServo.Direction.FORWARD);
        intakeStage3.setDirection(CRServo.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*camera = hardwareMap.get(WebcamName.class, "Webcam 1");

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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();*/

        waitForStart();

        if (opModeIsActive()) {
            driver.drive(-36, 0, 0, 2000);
            driver.drive(0, -2, 0, 2000); //get against wall
            SHOOT(); //shoot three preloads
            driver.drive(24, 51, 0, 2000); //go to first three artifacts
            driver.intake(2000);
            driver.drive(24, 0, 0, 2000); //collect artifacts
            sleep(100);
            driver.drive(-48, -51, 0, 2000); //get back into shooting position
            SHOOT(); //shoot three
          
            driver.drive(24, 75, 0, 2000); //go to second three artifacts
            driver.intake(2000);
            driver.drive(24, 0, 0, 2000); //collect artifacts
            sleep(100);
            driver.drive(-48, -75, 0, 2000); //get back into shooting position
            SHOOT(); //shoot three

            driver.drive(24, 99, 0, 2000); //go to last three artifacts
            driver.intake(2000);
            driver.drive(24, 0, 0, 2000); //collect artifacts
            sleep(100);
            driver.drive(-48, -99, 0, 2000); //get back into shooting position
            SHOOT(); //shoot three
        }

    }

    private void SHOOT() {
        //driver.adjust(0, 60, 0);
        //driver.intake(1000); //don't need until we have 4+ artifacts in auto
        driver.shoot(LONG_SHOT);
        driver.intake(15000); //intake is on while we are shooting
    }

}
