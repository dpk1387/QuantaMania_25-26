package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

@Autonomous(name = "RED FAR", group = "Autonomous")
@Config
public class RedFar extends LinearOpMode {
    /* HARDWARE */
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private DcMotor stage1 = null;
    // private DcMotor stage2 = null;
    private DcMotor stage3 = null;
    private Servo blockShooter = null;
    final private double OPENSHOOTER_OPEN = 0.8; //0.19 //0.3;
    final private double OPENSHOOTER_CLOSED = 1.0; // OPENSHOOTER_OPEN + 28//0.55
    final private double SHOOTER_VELOCITY = 2500; //2000 //2100 //2200 //2150

    final private double SHOOTER_GEAR_RATIO = 17.0/18.0;
    /* INIT */
    private ElapsedTime runtime = new ElapsedTime();

    //SHOOTING CLASS
    public class ShootAllAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("ShootAllAction", "Firing 3 shots");
                // Fire three balls in sequence (blocking, similar to SleepAction(3))
                shootN(6);
                //sleep(500); //sleep before moving to next position

                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    // Convenience factory so you can just write shootAll() in your SequentialAction
    public Action shootAll() {
        return new ShootAllAction();
    }

    //START THE SHOOTER/POWER THE SHOOTER
    public class PowerShooter implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("PowerShooterAction", "Power shooter to power = 0.9");
                //set the shooter power to 0.9
                //shooter.setVelocity(5400);
                shootVelocity(SHOOTER_VELOCITY);
                //shooter.setPower(0.90);
                //sleep(500);
                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    // Convenience factory so you can just write startShooter() in your SequentialAction
    public Action startShooter() {
        return new PowerShooter();
    }

    //run the intake
    public class startIntakeAction implements Action {
        private boolean initialized = false;
        private double s1, s3;
        public startIntakeAction(double s1_in, double s3_in){
            s1 = s1_in;
            s3 = s3_in;
            blockShooter.setPosition(OPENSHOOTER_CLOSED);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("Action", "Closing gate");
                //runIntake(0, 0.7, 1.0);
                runIntake(s1, s3);
                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }
    public Action startIntake(double s1, double s3) {
        return new startIntakeAction(s1, s3);
    }


    public class intakeDelay1 implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("intake Delay", "");
                // Fire three balls in sequence (blocking, similar to SleepAction(3))
                sleep(400-75); //550 //750 //1000 //1500

                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    // Convenience factory so you can just write shootAll() in your SequentialAction
    public Action intakeWait1() {
        return new intakeDelay1();
    }

    public class intakeDelay2 implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("intake Delay", "");
                // Fire three balls in sequence (blocking, similar to SleepAction(3))
                sleep(350-50); //1500

                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    // Convenience factory so you can just write shootAll() in your SequentialAction
    public Action intakeWait2() {
        return new intakeDelay2();
    }

    /// ***********************************************************************************
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        telemetry.update();

        Pose2d startPose = new Pose2d(60, 11, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        initMotors();
        // initAprilTagAndColorBlob();

        // -------------------------------------------------------------------------
        // FIX 1: Define all poses and variables BEFORE waitForStart() so trajectories
        //         can be pre-built during the init phase. This eliminates the per-segment
        //         build-time pause that occurred at every actionBuilder boundary.
        // -------------------------------------------------------------------------
        double shootX = 54, shootY = 13;

        Pose2d shootPose = new Pose2d(shootX, shootY, Math.toRadians(155));

        Pose2d loadZonePose = new Pose2d(62, 62, Math.toRadians(90));

        double stage1power = 0.8;
        double stage3power = 0.1;

        // Traj 1: start -> shoot position
        // (strafeTo is fine here since it's the opening move with no prior tangent)
        Action traj1 = drive.actionBuilder(startPose)
                .splineToLinearHeading(shootPose, Math.toRadians(170))
                .build();

        // Traj 2: shoot pose -> loading zone
        Action traj2_toLoadingZone = drive.actionBuilder(shootPose)//
                .splineToLinearHeading(loadZonePose, Math.toRadians(180))
                .build();


        // Traj 3: loading zone -> shoot pose
        Action traj3 = drive.actionBuilder(loadZonePose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose, Math.toRadians(90))
                .build();

        // Traj 4: shoot pose -> 3rd row -> shoot pose
        Action traj4_thirdRow = drive.actionBuilder(shootPose)
                .splineToLinearHeading(new Pose2d(36, 28, Math.toRadians(90)), Math.toRadians(135))
                .strafeTo(new Vector2d(36, 42))
                .build();

        Action traj5 = drive.actionBuilder(new Pose2d(36, 42, Math.toRadians(90)))
                .splineToLinearHeading(shootPose, Math.toRadians(110))
                .build();

        // Traj 5: shoot pose -> classifier (SECOND TIME)
        Action traj6_leaveLaunchZone = drive.actionBuilder(shootPose)
                .strafeTo(new Vector2d(30, 20))
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Create telemetry thread
        Thread telemetryThread = new Thread(() -> {
            while (!isStopRequested()) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Runtime (seconds)", runtime.seconds());
                dashboard.sendTelemetryPacket(packet);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });
        waitForStart();
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        runtime.reset();
        telemetryThread.start();

        while (opModeIsActive()){
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.update();
            //*
            try {
                Actions.runBlocking(
                        new SequentialAction(
                                startShooter(),
                                startIntake(stage1power, stage3power),

                                // --- Traj 1: go to shooting place ---
                                traj1,
                                //shooterWait(), //let shooter accelerate
                                shootAll(), //shoot 3 balls
                                startIntake(stage1power, stage3power), //start intake

                                // --- Traj 2+3: go to loading zone, go shoot
                                traj2_toLoadingZone,
                                // shootAll() and startIntake() are now inside traj2 via afterDisp

                                intakeWait1(), // wait for intake at loading zone

                                traj3, // back to shooting position

                                shootAll(),

                                traj4_thirdRow, //get artifacts from third row

                                traj5, // back to shooting position
                                shootAll(),

                                traj6_leaveLaunchZone // leave launch zone for leave points
                        )
                );
                telemetry.addData("Trajectory", "Executed Successfully");
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }

            blockShooter.setPosition(OPENSHOOTER_CLOSED);
            break; ///quite the opmode loop
        }
    }
    private void initMotors(){

        //1. need initial the shooter, stage1, 2, 3, servo
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "topShooterMotor");
        stage1 = hardwareMap.get(DcMotor.class, "stage1");
        // stage2 = hardwareMap.get(DcMotor.class, "stage2");
        stage3 = hardwareMap.get(DcMotor.class, "stage3");
        blockShooter = hardwareMap.get(Servo.class, "blockShooter");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //ALREADY set in the driver

        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        stage1.setDirection(DcMotor.Direction.REVERSE);
        // stage2.setDirection(DcMotor.Direction.REVERSE);
        stage3.setDirection(DcMotor.Direction.REVERSE);
        blockShooter.setDirection(Servo.Direction.REVERSE); //Do we really need this?

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shootN(int count) {
        //*
        final double targetVel = SHOOTER_VELOCITY + 100; //close = 2200. far = 2500.   // same units you use in setVelocity/getVelocity
        final double lowRecoverMargin = 100; //100;      // tune (smaller than dropMargin)
        final double stage3FeedPower = 0.6;    //tune down if multiple balls sneak
        final double stage3HoldPower = 0.0;

        startIntake(0.9, 0.3); //start intake to move thing up

        final double GATE_HOLD = OPENSHOOTER_CLOSED;   // you may want a slightly-open "hold" instead
        final double GATE_PULSE_OPEN = OPENSHOOTER_OPEN; // tune so 1 ball passes, not 2

        final int pulseMs = 250; //180;//400;//130;              // tune: shorter = fewer double-feeds
        // final int stableMs = 120;             // require speed stable before feeding next ball
        final int loopSleepMs = 15;

        // Spin up
        blockShooter.setPosition(GATE_HOLD);
        shootVelocity(targetVel);
        stage3.setPower(stage3HoldPower);

        // sleep(600);
        ElapsedTime time_pass = new ElapsedTime();
        time_pass.reset();

        while(time_pass.milliseconds() <= 1000-300){ //1000-200
            // 4) Wait for recovery enough to avoid weak/overpowered 2nd/3rd shots.getVelocity()
            while (opModeIsActive() && shooter.getVelocity() < targetVel - lowRecoverMargin) { //shooter.getVelocity() < targetVel - lowRecoverMargin && shooter.getVelocity() > targetVel + highRecoverMargin
                sleep(loopSleepMs);
                idle();
            }

            stage3.setPower(stage3FeedPower);
            blockShooter.setPosition(GATE_PULSE_OPEN);
            sleep(pulseMs);

            // 3) Immediately block the next ball
            blockShooter.setPosition(GATE_HOLD);
        }

        // Stop / reset
        stage3.setPower(0);
        blockShooter.setPosition(GATE_HOLD);
        startIntake(0.8, 0.3); //start intake

        /*
        final double targetVel = SHOOTER_VELOCITY + 200; //close = 2200. far = 2500.   // same units you use in setVelocity/getVelocity
        final double stage3FeedPower = 0.8;    //tune down if multiple balls sneak
        startIntake(0.9, 0.5); //start intake
        final int pulseMs = 700; //180;//400;//130;              // tune: shorter = fewer double-feeds
        // Spin up
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        shootVelocity(targetVel);

        stage3.setPower(stage3FeedPower);
        blockShooter.setPosition(OPENSHOOTER_OPEN);
        sleep(pulseMs);


        startIntake(1.0, 0.5); //start intake
        //blockShooter.setPosition(OPENSHOOTER_CLOSED);
        //*/
    }

    //running intake
    public void runIntake(double s1, double s3) {
        stage1.setPower(s1);
        // stage2.setPower(s2);
        stage3.setPower(s3);
    }

    public void shootVelocity(double base){
        shooter.setVelocity(base);
        shooter2.setVelocity((double) (base * SHOOTER_GEAR_RATIO));

    }
}