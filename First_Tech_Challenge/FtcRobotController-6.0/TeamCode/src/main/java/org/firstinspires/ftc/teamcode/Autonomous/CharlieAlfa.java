 package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import androidx.core.location.GnssStatusCompat;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.FTCLib.UGBasicHighGoalPipeline;
import com.qualcomm.robotcore.hardware.ColorSensor;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

@Autonomous(group = "Charlie")
@Disabled
public class CharlieAlfa extends LinearOpMode{

    private UGBasicHighGoalPipeline pipeline;
    private UGContourRingPipeline ringPipeline;

    OpenCvCamera webcam;
    OpenCvCamera FrontCamera;

    ColorSensor Low_Sensor;
    ColorSensor High_Sensor;

    private int HeightStorage = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Initialization step: ", "Booting Cameras");
        telemetry.update();

        //New Camera Init
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                    .splitLayoutForMultipleViewports(
                            cameraMonitorViewId, //The container we're splitting
                            2, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

            FrontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[0]);
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);

            FrontCamera.openCameraDevice();
            webcam.openCameraDevice();

            FrontCamera.setPipeline(ringPipeline = new UGContourRingPipeline());
            webcam.setPipeline(pipeline = new UGBasicHighGoalPipeline());

            FrontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }

        Low_Sensor = hardwareMap.colorSensor.get("LowSensor");
        High_Sensor = hardwareMap.colorSensor.get("HighSensor");

        telemetry.addData("Initialization step: ", "Loading RoadRunner");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-63, -25, Math.toRadians(180))); // -60, -25

        drive.wobbleClamp.setPosition(1);
        drive.wobbleRotate.setPosition(1);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime MatchTimer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();

        drive.intakeLift.setPosition(1);

        /** GENERIC MOVEMENT */

        Trajectory trajGeneric1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                //Move to the starter stack
                .lineToLinearHeading(new Pose2d(0, -22, Math.toRadians(184))) //-10
                .build();

        Trajectory trajGeneric2 = drive.trajectoryBuilder(trajGeneric1.end().plus(new Pose2d(0, 0, 30)))
                //Drive to pickup disks
                .lineToLinearHeading(new Pose2d(-21, -42, Math.toRadians(230)))
                .build();

        Trajectory trajNull2 = drive.trajectoryBuilder(trajGeneric2.end())
                .forward(20)
                .build();
        Trajectory trajNull3 = drive.trajectoryBuilder(trajNull2.end())
                .back(20)
                .build();

        //Fire powershots

        /** HEIGHT ZERO */

        Trajectory trajHeightZERO = drive.trajectoryBuilder(trajNull3.end())
                //Drive to wobble goal delivery
                .lineToLinearHeading(new Pose2d(15, -50, Math.toRadians(0)))
                .build();

        Trajectory trajHeightZERO2 = drive.trajectoryBuilder(trajHeightZERO.end())
                //Move to collect second wobble goal
                .lineToLinearHeading(new Pose2d(-40, -35, Math.toRadians(-50)))
                .build();

        Trajectory trajHeightZERO3 = drive.trajectoryBuilder(trajHeightZERO2.end())

                //Move to second wobble goal delivery
                .lineToLinearHeading(new Pose2d(2, -50, Math.toRadians(50)))
                .build();

        Trajectory trajHeightZERO4 = drive.trajectoryBuilder(trajHeightZERO3.end())
                //Move to sit on line
                .lineToLinearHeading(new Pose2d(10, -20, Math.toRadians(0)))
                .build();

        /** HEIGHT ONE */

        Trajectory trajHeightONE = drive.trajectoryBuilder(trajNull3.end())
                //Drive to wobble goal delivery
                .lineToLinearHeading(new Pose2d(47, -35, Math.toRadians(0)), DriveConstants.SPEED_CONSTRAINTS)
                .build();

        Trajectory trajHeightONE2 = drive.trajectoryBuilder(trajHeightONE.end(), true)
                //Drive to pickup second wobble goal
                .lineToLinearHeading(new Pose2d(-40, -29, Math.toRadians(-50)))
                .build();

        Trajectory trajHeightONE3 = drive.trajectoryBuilder(trajHeightONE2.end())
                //Move to deliver second wobble goal
                .lineToLinearHeading(new Pose2d(20, -25, Math.toRadians(80)), DriveConstants.SPEED_CONSTRAINTS)
                .build();

        Trajectory trajHeightONE4 = drive.trajectoryBuilder(trajHeightONE3.end())
                //Drive to sit on line
                .lineToLinearHeading(new Pose2d(15, -25, Math.toRadians(80)), DriveConstants.ULTRA_SPEED_CONSTRAINTS)
                .build();

        /** HEIGHT FOUR */

        Trajectory trajHeightFOUR =  drive.trajectoryBuilder(trajNull3.end())
                //Drive to wobble goal delivery zone
                //.lineToLinearHeading(new Pose2d(55, -50, 50), DriveConstants.SPEED_CONSTRAINTS)
                .splineToSplineHeading(new Pose2d(47, -54, Math.toRadians(30)), Math.toRadians(0))//, DriveConstants.SPEED_CONSTRAINTS)
                //Drop first wobble goal
                .build();

        /*Trajectory trajHeightFOUR2 = drive.trajectoryBuilder(trajHeightFOUR.end())
                //Drive to pickup second wobble goal
                .lineToLinearHeading(new Pose2d(-43, -33, Math.toRadians(-50)))
                //Pickup second wobble goal
                .build();*/

        /*Trajectory trajHeightFOUR3 = drive.trajectoryBuilder(trajHeightFOUR2.end())
                //Drive to deliver second wobble goal (Possibly pickup remaining disk???)
                .lineToLinearHeading(new Pose2d(50, -50, -50), DriveConstants.SPEED_CONSTRAINTS)
                //Drop second wobble goal in target zone
                .build();*/

        Trajectory trajHeightFOUR4 = drive.trajectoryBuilder(trajHeightFOUR.end())
                //Move to rest on line
                //.lineToLinearHeading(new Pose2d(15, -50, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(15, -50, Math.toRadians(0)), Math.toRadians(180))
                .build();

        telemetry.addLine("Finished computing, waiting for start");
        telemetry.addData("CharlieAlfa Version : ", "1.11");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        MatchTimer.reset();

        //Initialization
        drive.intakeLift.setPosition(0);
        drive.TreadGate.setPosition(.7);
        drive.flyWheel.setVelocity(490, AngleUnit.DEGREES);
        drive.Intake.setPower(-1);

        //Move to power shot firing solution
        drive.followTrajectory(trajGeneric1);

        //Do powershots
        PowerShotsAuto(timer, drive);

        //Do some init stuff
        drive.TreadGate.setPosition(.7);
        drive.flyWheel.setVelocity(520, AngleUnit.DEGREES);
        drive.Intake.setPower(-1);
        drive.Tread.setPower(1);

        //Move to the starter stack
        drive.followTrajectory(trajGeneric2);
        //Do movements to capture disks
        drive.followTrajectory(trajNull2);
        drive.followTrajectory(trajNull3);

        //Leave the tread idling
        drive.Tread.setPower(1);
        drive.Intake.setPower(-1);

        //sleep(1000);

        //Detect starter stack
        HeightStorage = detectColorHeight();



        if(HeightStorage == 0){
            //Shut down the Fly wheel and tread because we won't need it
            drive.flyWheel.setPower(0);
            drive.Tread.setPower(0);

            //Move to Zero wobble goal delivery zone
            drive.followTrajectory(trajHeightZERO);
            //Drop first wobble goal
            DeployLifter(drive, timer, 0);
            //Move to second wobble goal pickup
            drive.followTrajectory(trajHeightZERO2);
            //Pickup second wobble goal
            DeployLifter(drive, timer, 1);
            //Move to second wobble goal delivery
            drive.followTrajectory(trajHeightZERO3);
            //Drop second wobble goal
            DeployLifter(drive, timer, 2);
            //Move to sit on line
            drive.followTrajectory(trajHeightZERO4);


        } else if (HeightStorage == 1){

            //Do normal shooting
            drive.flyWheel.setVelocity(520, AngleUnit.DEGREES);
            drive.turn(Math.toRadians(-60));
            AutoFire(timer, drive, drive.getPoseEstimate());

            //Shut down the tread
            drive.Tread.setPower(0);

            //Move to wobble goal delivery zone, shut down flywheel
            drive.followTrajectory(trajHeightONE);
            drive.flyWheel.setPower(0);
            DeployLifter(drive, timer, 0);

            //Move to pickup second wobble goal
            drive.followTrajectory(trajHeightONE2);
            DeployLifter(drive, timer, 1);

            //Move to drop second wobble goal
            drive.followTrajectory(trajHeightONE3);
            DeployLifter(drive, timer, 2);

            //Move to sit on line
            //drive.followTrajectory(trajHeightONE4);


        } else if (HeightStorage == 4) {

            //Do Normal shooting
            drive.flyWheel.setVelocity(520, AngleUnit.DEGREES);
            drive.turn(Math.toRadians(-60));
            AutoFire(timer, drive, drive.getPoseEstimate());

            //Leave our intake systems running in case we run into a stray disk
            drive.Intake.setPower(.8);
            drive.Tread.setPower(.4);

            //Drive to first wobble goal delivery zone
            drive.followTrajectory(trajHeightFOUR);
            drive.flyWheel.setPower(0);
            DeployLifter(drive, timer, 0);

            //Drive to pickup second wobble goal
            /*drive.followTrajectory(trajHeightFOUR2);
            DeployLifter(drive, timer, 1);

            //Drive to second wobble goal delivery zone
            drive.followTrajectory(trajHeightFOUR3);
            DeployLifter(drive, timer, 2);*/

            //Drive to sit on line
            drive.followTrajectory(trajHeightFOUR4);
        }

        //Shut down an extraneous systems
        drive.Tread.setPower(0);
        drive.Intake.setPower(0);

        //Display telemetry data to the console
        telemetry.addData("Guessed Height: ", HeightStorage);
        telemetry.addData("Match: ", MatchTimer.milliseconds());
        telemetry.update();

        PoseStorage.currentPose = drive.getPoseEstimate();


    }

    public int detectColorHeight(){
        //Low: 123, 83, 56
        //High: 400, 255, 155
        //----Empty
        //Low: 37, 46, 42
        //High: 97, 139, 70
        if((Low_Sensor.red() > 54))// && Low_Sensor.green() > 60 && Low_Sensor.blue() > 48))
            return 4;
        else if ((High_Sensor.red() > 248 && High_Sensor.green() > 197 && High_Sensor.blue() > 112))
            return 1;
        else
            return 0;
        //(High_Sensor.red() > 248 && High_Sensor.green() > 197 && High_Sensor.blue() > 112) &&
    }

    /** AUTOMATIC SALVO SYSTEMS */
    public void AutoFire(ElapsedTime timer, SampleMecanumDrive drive, Pose2d poseEstimate) {
        //VERSION 1.03b-a-s
        //Code: [Current code][Extra code]-[Tele/Auto]-[Experimental/Stable]
        /**
         * Changelog:
         * 1.02e: Added Automatic shutoff via color sensor recognition. Added assorted comments.
         * 1.03e: Removed Automatic shutoff via color sensor. Added wait for start in an attempt to remove first shot misses.
         * 1.03a-s: Working consistently, only change if in error.
         * 1.03b-t-s: Adjusted the speed of the robot during Manual Override
         * 1.03b-a-s: Update for autonomous mode
         */
        double ResolvedX;
        int TargetX = 185;
        boolean ReadyToFire = false;
        double ResolvedAccuracy = 0.01;
        double kV = .006;

        telemetry.update();

        //Turn on assorted systems
        drive.Tread.setPower(0.0);
        drive.Intake.setPower(-0.2);
        drive.TreadGate.setPosition(0.7);
        drive.flyWheel.setVelocity(520, AngleUnit.DEGREES);

        telemetry.update();

        //Is the high goal visible? If yes, continue firing routine.
        if(pipeline.isRedVisible()){

            ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
            timer.reset();
            while((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 1000 && gamepad1.dpad_right != true && gamepad2.dpad_right != true) {
                while ((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 1000 && gamepad1.dpad_right != true && gamepad2.dpad_right != true) {
                    telemetry.addLine("Auto Aligning. Press Dpad Right to cancel.");
                    telemetry.update();
                    ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
                    drive.setMotorPowers(ResolvedX, ResolvedX, -ResolvedX, -ResolvedX);
                    drive.update();
                }
            }

            drive.setMotorPowers(0, 0, 0, 0);

            ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);

            if(ResolvedX > -ResolvedAccuracy && ResolvedX < ResolvedAccuracy || timer.milliseconds() > 950){
                ReadyToFire = true;
            }
        }

        while(drive.flyWheel.getVelocity(DEGREES) < 518) drive.update();

        ReadyToFire = true;

        timer.reset();
        while (isStopRequested() != true && opModeIsActive() && ReadyToFire == true){

            drive.update();

            telemetry.addData("Flywheel Voltage ", drive.flyWheel.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Firing", ": Yes");



            //If the flywheel is at the correct velocity, open the gate and fire
            if(drive.flyWheel.getVelocity(DEGREES) > 515){
                drive.Tread.setPower(1);
                drive.TreadGate.setPosition(0.4);
            } else {
                drive.Tread.setPower(0);
            }
            if(timer.milliseconds() > 2300 && HeightStorage == 1)
                ReadyToFire = false;
            else if (timer.milliseconds() > 5000 && HeightStorage == 4)
                ReadyToFire = false;

            telemetry.addData("Flywheel Velocity", drive.flyWheel.getVelocity(DEGREES));
            telemetry.addData("Target X Pos", pipeline.getCenterofRect(pipeline.getRedRect()).x);

            telemetry.update();
        }

        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.7);
        timer.reset();
    }

    public void DeployLifter(SampleMecanumDrive drive, ElapsedTime timer, int mode){
        //Mode 0: Initial deployment
        //Mode 1: Pickup
        //Mode 2: Drop
        if(mode == 0){

            timer.reset();
            while(timer.milliseconds() < 1500){
                drive.wobbleLift.setPower(.6);
            }

            timer.reset();
            while(timer.milliseconds() < 1500){
                drive.wobbleLift.setPower(-.6);
            }
            drive.wobbleRotate.setPosition(.75);

            sleep(400);

            drive.wobbleClamp.setPosition(.4);
            drive.wobbleLift.setPower(0);



        } else if (mode == 1){
            drive.wobbleClamp.setPosition(1);

            timer.reset();
            while(timer.milliseconds() < 800){
                drive.wobbleLift.setPower(.6);
            }
            drive.wobbleLift.setPower(0);



        } else if (mode == 2){
            drive.wobbleClamp.setPosition(.4);

            timer.reset();
            while(timer.milliseconds() < 800){
                drive.wobbleLift.setPower(-.6);
            }

            drive.wobbleLift.setPower(0);
        }
    }

    public void PowerShotsAuto(ElapsedTime timer, SampleMecanumDrive drive) {

        //Open the gate and start firing
        drive.TreadGate.setPosition(.1);
        drive.Tread.setPower(.95);

        //Turn slowly
        drive.turnAsyncVelControl(Math.toRadians(30), Math.toRadians(34), Math.toRadians(34));

        //Wait for the drive to finish
        while(drive.isBusy()) drive.update();

        sleep(300);

    }


}

/********** BONEYARD **************
 *

 while(drive.isBusy()){
 if(isStopRequested() == true || opModeIsActive() == false){
 drive.cancelFollowing();
 break;
 }
 drive.update();
 telemetry.addData("Pose Estimate ", drive.getPoseEstimate());
 telemetry.addData("Height ", HeightStorage);
 telemetry.update();
 }

    public void DeadFire (ElapsedTime timer, SampleMecanumDrive drive, double flyWheelPower) {
        drive.flyWheel.setVelocity(450, AngleUnit.DEGREES);
        drive.Tread.setPower(0);
        drive.Intake.setPower(-.2);
        drive.TreadGate.setPosition(.7);

        timer.reset();
        while(timer.milliseconds() < 500 && isStopRequested() != true && opModeIsActive());

        drive.TreadGate.setPosition(.1);
        drive.Tread.setPower(1);

        timer.reset();
        while(timer.milliseconds() < 400 && isStopRequested() != true && opModeIsActive());

        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.7);
    }*/