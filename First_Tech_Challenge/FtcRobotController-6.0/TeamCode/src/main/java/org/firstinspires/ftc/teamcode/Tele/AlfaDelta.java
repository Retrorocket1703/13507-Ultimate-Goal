package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.TeleOpAugmentedDriving;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class AlfaDelta extends LinearOpMode {
    //TODO: EXPERIMENTAL (ROADRUNNER AUTO MOVEMENT TRAJECTORIES)
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    //Experimental ends here

    private DcMotor lift = null;
    private DcMotor flyWheel = null;
    private DcMotor Intake = null;
    private Servo liftServo = null;

    ColorSensor color_sensor;
    TouchSensor touch;

    private Servo TransferArm = null;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        liftServo = hardwareMap.get(Servo.class, "liftServo");
        //flyWheel.setDirection(DcMotor.Direction.REVERSE);

        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //flyWheel.setDirection(DcMotor.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotor.class, "intake");

        TransferArm = hardwareMap.get(Servo.class, "transferArm");

        color_sensor = hardwareMap.colorSensor.get("sensor");
        touch = hardwareMap.touchSensor.get("touch");


        double flyWheelSpeed = 1;
        double speedArc = .3;

        boolean overrideAll = true; //Todo: Reset this

        //Todo: EXPERIMENTAL (ROADRUNNER AUTO MOVEMENT TRAJECTORIES)
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            //TODO: EXPERIMENTAL (ROADRUNNER AUTO MOVEMENT TRAJECTORIES)
            drive.update();

            //Telemetry
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());


            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    //Send data to drive
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * speedArc,
                                    -gamepad1.left_stick_x * speedArc,
                                    -gamepad1.right_stick_x * speedArc
                            )
                    );

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(targetBVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }

            //End Experimental

            //Initialization
            //lift.setPower(1);

            //Speedarc controls
            /*if(gamepad1.a)
                speedArc = .3;
            else if (gamepad1.b)
                speedArc = .5;
            else if (gamepad1.y)
                speedArc = .7;
            else if (gamepad1.x)
                speedArc = 1;*/

            //Flywheel controls
            /*if(gamepad2.left_bumper)
                flyWheel.setPower(flyWheelSpeed);
            else if (gamepad2.right_bumper)
                flyWheel.setPower(0);*/

            //TODO: EXPERIMENTAL (GENERAL CONTROL SCHEME OVERRIDES)
            //Automatic elevation controls / Override
            Pose2d poseEstimate = drive.getPoseEstimate();
            if(gamepad2.x) {
                overrideAll = true;
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(gamepad2.y) {
                overrideAll = false;
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                lift.setPower(1);
            }

            if(overrideAll = false){

                lift.setTargetPosition((int)Math.round(poseEstimate.getX()));

            }
            else if (overrideAll = true){

                //Lift Controls
                if(gamepad2.dpad_left)
                    lift.setPower(1);
                else if (gamepad2.dpad_right)
                    lift.setPower(-1);
                else
                    lift.setPower(0);

                if(gamepad2.a){
                    TransferArm.setPosition(.95);
                }
                else if (gamepad2.b){
                    TransferArm.setPosition(.2);
                }

                Intake.setPower(-1);

            }

            //End experimental

            //TODO: EXPERIMENTAL (AUTO SENSOR CONTROLS)
            //Experimental auto arm controller

            telemetry.addData("red", color_sensor.red());
            telemetry.addData("green", color_sensor.green());
            telemetry.addData("blue", color_sensor.blue());
            telemetry.addData("alpha", color_sensor.alpha());
            telemetry.addData("argb", color_sensor.argb());
            telemetry.addData("Arm Pos", TransferArm.getPosition());
            telemetry.update();

            if((color_sensor.red() > 180) && (color_sensor.green() > 70 && color_sensor.green() < 190) && (color_sensor.blue() > 60 && color_sensor.blue() < 200) && overrideAll != true){
                TransferArm.setPosition(.2);
            }
            if (touch.isPressed() == true && overrideAll != true) {
                TransferArm.setPosition(.95);
            }
            //End Experimental

            //TODO: EXPERIMENTAL (SERVO LIFT CONTROLS)
        if(gamepad2.dpad_up){
            liftServo.setPosition(1);
        } else if (gamepad2.dpad_down)
            liftServo.setPosition(0);

        }
    }
