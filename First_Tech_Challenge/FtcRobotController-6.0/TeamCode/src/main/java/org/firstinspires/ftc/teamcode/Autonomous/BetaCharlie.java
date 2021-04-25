package org.firstinspires.ftc.teamcode.Autonomous;

import android.service.autofill.FieldClassification;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.Angle;

import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGRectRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.advanced.TeleOpAlignWithPoint;
import org.firstinspires.ftc.teamcode.drive.advanced.TeleOpAugmentedDriving;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.FTCLib.UGBasicHighGoalPipeline;


import com.acmerobotics.roadrunner.control.PIDFController;

import java.lang.annotation.Target;

/**
 * This file represents the primary autonomous routine for the robot, using Roadrunner, OpenCV, and that other pipeline.
 */

@Autonomous(group = "Beta")
public class BetaCharlie extends LinearOpMode {

    private UGBasicHighGoalPipeline pipeline;
    private UGContourRingPipeline ringPipeline;

    OpenCvCamera webcam;
    OpenCvCamera FrontCamera;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Initialization step: ", "Booting Cameras");
        telemetry.update();

        //New Camera Init

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



        telemetry.addData("Initialization step: ", "Loading RoadRunner");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        boolean isDiskPrimed = false;
        int DisksInStowage = 0;
        int HeightStorage = 0;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        Pose2d startPose = new Pose2d(-60, -25, 0);

        drive.setPoseEstimate(startPose);

       /* Trajectory GenericStartTrajP1 = drive.trajectoryBuilder(startPose)
                //.lineTo(new Vector2d(-60, -20))
                //.splineTo(new Vector2d(-60, -20), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-60, -15), Math.toRadians(0))
                //.splineTo(new Vector2d(-30, -20), -45)
                .build();

        Trajectory GenericStartTrajP2 = drive.trajectoryBuilder(GenericStartTrajP1.end())
                .splineToConstantHeading(new Vector2d(-60, -25), Math.toRadians(0))
                .build();*/

        Trajectory GenericStartTrajP3 = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-30, -20, Math.toRadians(300)), Math.toRadians(0))
                .build();


        /**
         * All builds for height zero go here
         */
        Trajectory TrajHeightZeroP1 = drive.trajectoryBuilder(GenericStartTrajP3.end())
                .splineToSplineHeading(new Pose2d(20, -40, Math.toRadians(0)), Math.toRadians(0))
                .build();




        /**
         * All builders for height one go here
         */

        Trajectory TrajHeightOneP1 = drive.trajectoryBuilder(GenericStartTrajP3.end())
                .splineToSplineHeading(new Pose2d(-20, -20, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(35, -30, Math.toRadians(0)), Math.toRadians(-60))
                .build();

        /**
         * All builders for height four go here
         */


        drive.intakeLift.setPosition(.1);


        waitForStart();

        if (isStopRequested()) return;

        drive.intakeLift.setPosition(.9);

        DeployLifter(drive, timer, 0);

        DeployLifter(drive, timer, 1);

        drive.followTrajectory(GenericStartTrajP3);

        if(ringPipeline.getHeight() == UGContourRingPipeline.Height.ZERO)
            HeightStorage = 0;
        else if (ringPipeline.getHeight() == UGContourRingPipeline.Height.ONE)
            HeightStorage = 1;
        else if (ringPipeline.getHeight() == UGContourRingPipeline.Height.FOUR)
            HeightStorage = 4;

        drive.flyWheel.setPower(.7);

        if(HeightStorage == 0) {

            Trajectory TrajPowershots0 = drive.trajectoryBuilder(TrajHeightZeroP1.end())
                    .splineToSplineHeading(new Pose2d(-10, -10, Math.toRadians(175)), Math.toRadians(180))
                    .build();

            drive.followTrajectory(TrajHeightZeroP1);

            DeployLifter(drive, timer, 2);

            drive.followTrajectory(TrajPowershots0);

            while(drive.isBusy());

            //AutoFire(timer, drive);
            PowerShotsAuto(timer, drive);

            Trajectory TrajHeightZeroP2 = drive.trajectoryBuilder(TrajPowershots0.end().plus(new Pose2d(0,0,11)))
                    .splineToSplineHeading(new Pose2d(-40, -38, Math.toRadians(0)), Math.toRadians(0))
                    .build();

            Trajectory TrajHeightZeroP3 = drive.trajectoryBuilder(TrajHeightZeroP2.end())
                    .splineToSplineHeading(new Pose2d(0, -50, Math.toRadians(60)), Math.toRadians(0))
                    .build();

            Trajectory TrajHeightZeroP4 = drive.trajectoryBuilder(TrajHeightZeroP3.end())
                    .splineToConstantHeading(new Vector2d(10, -30), Math.toRadians(0))
                    //.splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(0)), Math.toRadians(0))
                    .build();



            drive.followTrajectory(TrajHeightZeroP2);

            DeployLifter(drive, timer, 1);

            drive.followTrajectory(TrajHeightZeroP3);

            DeployLifter(drive, timer, 2);

            drive.followTrajectory(TrajHeightZeroP4);

            PoseStorage.currentPose = TrajHeightZeroP4.end();

        } else if (HeightStorage == 1) {



            Trajectory TrajPowershots1 = drive.trajectoryBuilder(TrajHeightOneP1.end())
                    .splineToSplineHeading(new Pose2d(-10, -10, Math.toRadians(175)), Math.toRadians(180))
                    .build();

            drive.followTrajectory(TrajHeightOneP1);
            //Drop wobble goal 1
            DeployLifter(drive, timer, 2);
            drive.followTrajectory(TrajPowershots1);
            //Drive and do powershots

            while(drive.isBusy());

            //AutoFire(timer, drive);
            PowerShotsAuto(timer, drive);

            /*Trajectory TrajHeightOneP2 = drive.trajectoryBuilder(TrajPowershots1.end().plus(new Pose2d(0,0,20)))
                    .splineToSplineHeading(new Pose2d(-30, -20, Math.toRadians(-100)), Math.toRadians(0))
                    .build();*/

            Trajectory TrajHeightOneP2 = drive.trajectoryBuilder(TrajPowershots1.end().plus(new Pose2d(0,0,11)))
                    .splineToSplineHeading(new Pose2d(-40, -38, Math.toRadians(0)), Math.toRadians(0))
                    .build();

            Trajectory TrajHeightOneP3 = drive.trajectoryBuilder(TrajHeightOneP2.end())
                    .splineToSplineHeading(new Pose2d(30, -30, Math.toRadians(60)), Math.toRadians(20))
                    .build();

            Trajectory TrajHeightOneP4 = drive.trajectoryBuilder(TrajHeightOneP3.end())
                    .splineToSplineHeading(new Pose2d(-10, -10, Math.toRadians(180)), Math.toRadians(180))
                    .build();

            Trajectory TrajHeightOneP5 = drive.trajectoryBuilder(TrajHeightOneP4.end())
                    .splineToSplineHeading(new Pose2d(10, -10, Math.toRadians(180)), Math.toRadians(0))
                    .build();

            drive.followTrajectory(TrajHeightOneP2);
            //Pick up second wobble goal

            drive.Intake.setPower(-1);
            drive.Tread.setPower(.5);

            DeployLifter(drive, timer, 1);

            drive.followTrajectory(TrajHeightOneP3);
            //Drive to deliver wobble goal and pick up disk

            DeployLifter(drive, timer, 2);


            drive.followTrajectory(TrajHeightOneP4);
            //Fire disk
            AutoFire(timer, drive);

            drive.followTrajectory(TrajHeightOneP5);

            PoseStorage.currentPose = TrajHeightOneP5.end();

        }
        else if (HeightStorage == 4){


            //Drive to drop off wobble goal 1 Step 2
            Trajectory TrajHeightFourP1 = drive.trajectoryBuilder(GenericStartTrajP3.end())
                    .splineToSplineHeading(new Pose2d(50, -50, Math.toRadians(60)), Math.toRadians(0))
                    .build();

            //Drive to fire powershots Step 3
            Trajectory TrajPowershots2 = drive.trajectoryBuilder(TrajHeightFourP1.end())
                    .splineToSplineHeading(new Pose2d(-10, -10, Math.toRadians(175)), Math.toRadians(180))
                    .build();

            //Drive to collect wobble goal 2 Step 4
            Trajectory TrajHeightFourP2 = drive.trajectoryBuilder(TrajPowershots2.end().plus(new Pose2d(0,0,11)))
                    .splineToSplineHeading(new Pose2d(-40, -38, Math.toRadians(0)), Math.toRadians(0))
                    .build();

            //Drive to collect disks and deliver wobble goal 2 Step 5
            Trajectory TrajHeightFourP3 = drive.trajectoryBuilder(TrajHeightFourP2.end())
                    .splineToSplineHeading(new Pose2d(-52, -40, Math.toRadians(0)), Math.toRadians(0))
                    .build();

            //Idk what this does but if I remove it it breaks everything so
            Trajectory TrajHeightFourP4 = drive.trajectoryBuilder(TrajHeightFourP3.end())
                    .splineToSplineHeading(new Pose2d(50, -50, Math.toRadians(60)), Math.toRadians(-90))
                    .build();

            //Drive to fire
            Trajectory TrajGoal = drive.trajectoryBuilder(TrajHeightFourP4.end())
                    .splineToSplineHeading(new Pose2d(-10, -10, Math.toRadians(180)), Math.toRadians(180))
                    .build();

            /*Trajectory TrajHeightFourP5 = drive.trajectoryBuilder(TrajGoal.end())
                    .splineToSplineHeading(new Pose2d(-20, -30, Math.toRadians(300)), Math.toRadians(0))
                    .build();

            Trajectory TrajGoal2 = drive.trajectoryBuilder(TrajHeightFourP5.end())
                    .splineToSplineHeading(new Pose2d(-10, -10, Math.toRadians(180)), Math.toRadians(180))
                    .build();*/

            Trajectory TrajHeightFourP6 = drive.trajectoryBuilder(TrajGoal.end())
                    .splineToSplineHeading(new Pose2d(10, -10, Math.toRadians(180)), Math.toRadians(0))
                    .build();



            drive.followTrajectory(TrajHeightFourP1);
            //TODO: Drop wobble goal 1

            DeployLifter(drive, timer, 2);
            drive.followTrajectory(TrajPowershots2);

            //Fire powershots
            //AutoFire(timer, drive);
            PowerShotsAuto(timer, drive);

            drive.followTrajectory(TrajHeightFourP2);
            //TODO: Pick up wobble goal 2
            DeployLifter(drive, timer, 1);
            //Turn on intake system
            drive.Intake.setPower(-1);
            drive.Tread.setPower(1);

            drive.followTrajectory(TrajHeightFourP3);


            //Turn off intake system
            drive.Intake.setPower(-.2);
            drive.Tread.setPower(0);

            drive.followTrajectory(TrajHeightFourP4);

            DeployLifter(drive, timer, 2);
            //TODO: Drop wobble goal 2

            drive.followTrajectory(TrajGoal);

            //Fire disks
            AutoFire(timer, drive);

            //drive.followTrajectory(TrajHeightFourP5);
            //drive.followTrajectory(TrajGoal2);
            drive.followTrajectory(TrajHeightFourP6);

            PoseStorage.currentPose = TrajHeightFourP6.end();


        }





        drive.flyWheel.setPower(0);
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("Height", HeightStorage);
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();


        while (!isStopRequested() && opModeIsActive()) ;

    }

    public void AutoFire(ElapsedTime timer, SampleMecanumDrive drive) {
        double ResolvedX;
        int TargetX = 185;
        boolean ReadyToFire = false;
        double ResolvedAccuracy = .02;

        drive.flyWheel.setPower(1);
        drive.Tread.setPower(0);
        drive.Intake.setPower(-.2);
        drive.TreadGate.setPosition(.7);

        if (pipeline.isRedVisible()) {

            ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
            timer.reset();

            while ((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 2000 && gamepad1.dpad_right != true) {
                telemetry.addData("Aligning", "");
                telemetry.update();
                ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
                drive.setMotorPowers(ResolvedX, ResolvedX, -ResolvedX, -ResolvedX);
            }

            drive.setMotorPowers(0, 0, 0, 0);

            ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);

            if (ResolvedX > -ResolvedAccuracy && ResolvedX < ResolvedAccuracy || timer.milliseconds() > 1900) {
                ReadyToFire = true;
            }
        }

        timer.reset();

        while (timer.milliseconds() < 1000) {
            //Hold until flywheel spins up
            telemetry.addData("Spooling up...", "");
            telemetry.update();
        }

        timer.reset();

        while (isStopRequested() != true && ReadyToFire == true && timer.milliseconds() < 2000) {

            telemetry.addData("Firing", ": Yes");
            drive.TreadGate.setPosition(.1);
            drive.Tread.setPower(1);
            if (gamepad1.dpad_right) {
                ReadyToFire = false;
            }
            telemetry.update();
        }

        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.7);
    }

    public void DeployLifter(SampleMecanumDrive drive, ElapsedTime timer, int mode){
        timer.reset();
        while(timer.milliseconds() < 2500 && mode == 0){

            drive.wobbleClamp.setPosition(1);
            if(timer.milliseconds() > 1000)
                drive.wobbleLift.setPower(-.6);

        }
        while(timer.milliseconds() > 2500 && timer.milliseconds() < 3600 && mode == 0){
            drive.wobbleLift.setPower(.6);
        }
        drive.wobbleLift.setPower(0);

        if(mode == 1)
            drive.wobbleClamp.setPosition(0);

        while(timer.milliseconds() < 2000 && mode == 1){
            drive.wobbleClamp.setPosition(0);
            if(timer.milliseconds() > 1000)
                drive.wobbleLift.setPower(-.6);
        }

        if(mode == 2) {
            drive.wobbleClamp.setPosition(1);
            timer.reset();
            while(timer.milliseconds() < 1000)
                drive.wobbleLift.setPower(.6);
        }
        drive.wobbleLift.setPower(0);
    }

    public void PowerShotsAuto(ElapsedTime timer, SampleMecanumDrive drive) {




        DeadFire(timer, drive, .8);

        drive.turn(Math.toRadians(5));

        DeadFire(timer, drive, .8);

        drive.turn(Math.toRadians(6));

        DeadFire(timer, drive, .8);

        drive.flyWheel.setPower(1);

    }

    public void DeadFire (ElapsedTime timer, SampleMecanumDrive drive, double flyWheelPower) {
        drive.flyWheel.setPower(flyWheelPower);
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
    }


}

/****** BONEYARD ******
 *
 *         /*Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
 *                 .forward(20)
 *                 .build();

 //drive.followTrajectory(trajectory);

 //AutoFire(timer, drive);
 //PowerShotsAuto(timer, drive, 200);


 //This is a random spline, and is not particularly optimized
 /*Trajectory traj = drive.trajectoryBuilder(new Pose2d())
 .splineTo(new Vector2d(30, 30), 0)
 .build();

 drive.followTrajectory(traj);

timer.reset();
while(timer.milliseconds() < 5)*/