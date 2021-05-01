package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.FTCLib.UGBasicHighGoalPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(group = "Beta")
public class BetaAlfa extends LinearOpMode {

    private UGBasicHighGoalPipeline pipeline;
    private UGContourRingPipeline ringPipeline;

    OpenCvCamera webcam;
    OpenCvCamera FrontCamera;



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
                            3, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

            FrontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[0]);
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);
            //webcam3 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 3"), viewportContainerIds[2]);


            FrontCamera.openCameraDevice();
            webcam.openCameraDevice();
            //webcam3.openCameraDevice();

            FrontCamera.setPipeline(ringPipeline = new UGContourRingPipeline());
            webcam.setPipeline(pipeline = new UGBasicHighGoalPipeline());

            FrontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            //webcam3.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }




        telemetry.addData("Initialization step: ", "Loading RoadRunner");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        /*PIDFController xPIDFController = new PIDFController(new PIDCoefficients(.01, 0, 0));
        PIDFController yPIDFController = new PIDFController(new PIDCoefficients(.01, 0, 0));
        PIDFController headingPIDFController = new PIDFController(new PIDCoefficients(.01, 0, 0));

        xPIDFController.setTargetPosition(targetPose.getX());
        yPIDFController.setTargetPosition(targetPose.getY());
        headingPIDFController.setTargetPosition(targetPose.getHeading());*/

        double speedArc = .5;
        double speedArcStorage = .5;

        Pose2d savePose;


        int Mode = 0;

        boolean isDiskPrimed = false;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime MatchTimer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();

        telemetry.addLine("Waiting for start");
        telemetry.addData("BetaAlfa Version : ", "1.06");
        telemetry.update();




        waitForStart();

        if (isStopRequested()) return;

        timer.reset();
        MatchTimer.reset();
        timer2. reset();
        timer3.reset();
        drive.TreadGate.setPosition(.7);

        Pose2d poseEstimate = drive.getPoseEstimate();

        //targetsUltimateGoal.activate();





        while (opModeIsActive() && !isStopRequested()) {


            /** ROADRUNNER RUNTIME INITIALIZATION STEPS */

            poseEstimate = drive.getPoseEstimate();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * speedArc,
                            -gamepad1.left_stick_x * speedArc,
                            -gamepad1.right_stick_x * speedArc
                    )
            );

            drive.update();
            //End Roadrunner Stuff

            /** AUTOMATIC PICKUP */

            if(ringPipeline.getHeight() != UGContourRingPipeline.Height.ZERO){
                isDiskPrimed = true;
                timer2.reset();
                //speedArc = .3;
            }
            else{
                speedArc = speedArcStorage;
            }

            if(isDiskPrimed == true && timer2.milliseconds() < 800){
                drive.Intake.setPower(-1);
                drive.Tread.setPower(1);
            } else {
                isDiskPrimed = false;
                drive.Tread.setPower(-gamepad2.right_stick_y);
                if(gamepad2.right_stick_y != 0) {
                    drive.Intake.setPower(gamepad2.right_stick_y);
                } else {
                    drive.Intake.setPower(-.2);
                }
            }


            //End automatic pickup



            /** MANUAL OVERRIDES AND CONTROLS*/
            //Keep the fly wheel spinning at partial speed
            //drive.flyWheel.setPower(0.82);
            drive.flyWheel.setVelocity(520, DEGREES);


            if(gamepad2.dpad_left || gamepad1.dpad_left) {

                AutoFire(timer4, drive, poseEstimate);

            }
            /*if(gamepad2.dpad_left || gamepad1.dpad_left) {

                drive.setDrivePower(new Pose2d(
                        new Vector2d(
                                xPIDFController.update(drive.getPoseEstimate().getX()),
                                yPIDFController.update(drive.getPoseEstimate().getY())
                        ).rotated(drive.getPoseEstimate().getHeading()).norm(),
                        headingPIDFController.update(drive.getPoseEstimate().getHeading())
                ));
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speedArc,
                                -gamepad1.left_stick_x * speedArc,
                                -gamepad1.right_stick_x * speedArc
                        )
                );
            }*/

            if(gamepad1.start)
                drive.setPoseEstimate(new Pose2d(0, 0, 0));

            if(gamepad1.back) {
                savePose = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(savePose.getX(), savePose.getY(), 180));
            }

            if(gamepad1.a) {
                speedArc = .3;
                speedArcStorage = .3;
            }
            else if(gamepad1.b) {
                speedArc = .5;
                speedArcStorage = .5;
            }
            else if (gamepad1.y) {
                speedArc = .7;
                speedArcStorage = .7;
            }
            else if (gamepad1.x) {
                speedArc = 1;
                speedArcStorage = 1;
            }

            if(gamepad2.y){
                drive.intakeLift.setPosition(.8);

            } else if (gamepad2.x){
                drive.intakeLift.setPosition(.1);
            }

            if(gamepad2.a)
                drive.wobbleClamp.setPosition(0);
            else if (gamepad2.b)
                drive.wobbleClamp.setPosition(1);

            if(gamepad2.left_bumper){
                drive.TreadGate.setPosition(.1);
            } else if (gamepad2.right_bumper){
                drive.TreadGate.setPosition(.7);
            }

            if(gamepad2.dpad_up){
                drive.TreadGate.setPosition(.4);
                drive.Tread.setPower(1);
            } else {
                drive.TreadGate.setPosition(.7);
            }

            drive.wobbleLift.setPower(-gamepad2.left_stick_y);

            /** ASSORTED TELEMETRY SYSTEMS */

            {
                telemetry.addData("Raw positions", drive.getWheelPositions());
                //telemetry.addData("Null", StandardTrackingWheelLocalizer.)
                telemetry.addData("RR Pose X ", drive.getPoseEstimate().getX());
                telemetry.addData("RR Pose Y ", drive.getPoseEstimate().getY());
                telemetry.addData("RR Pose Heading ", Math.toDegrees(drive.getPoseEstimate().getHeading()));
                if(pipeline.isRedVisible()) {
                    telemetry.addData("Img X Pos", pipeline.getRedRect().x);
                    telemetry.addData("Img Pos", pipeline.getCenterofRect(pipeline.getRedRect()));
                    telemetry.addData("Img Size", pipeline.getRedRect());
                }
                telemetry.addData("Flywheel Amperage ", drive.flyWheel.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
        }
    }

    /** AUTOMATIC SALVO SYSTEMS */
    public void AutoFire(ElapsedTime timer, SampleMecanumDrive drive, Pose2d poseEstimate) {
        //VERSION 1.01
        double ResolvedX;
        int TargetX = 175;
        boolean ReadyToFire = false;
        double ResolvedAccuracy = 0.02;
        double kV = .004;

        telemetry.update();

        //Turn on assorted systems
        //drive.flyWheel.setPower(0.9);
        drive.Tread.setPower(0.0);
        drive.Intake.setPower(-0.2);
        drive.TreadGate.setPosition(0.7);

        //Initiate Automatic movement via Roadrunner
        GoToViaSpline(drive, poseEstimate, 0, 0, 0, 0);
        timer.reset();

        //Allow the driver to adjust the position of the robot
        while(gamepad1.dpad_right == false && gamepad2.dpad_right == false && gamepad1.dpad_left == false && gamepad2.dpad_left == false){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * .3,
                            -gamepad1.left_stick_x * .3,
                            -gamepad1.right_stick_x * .3
                    )
            );

            telemetry.addLine("Manually Aligning. Press Dpad Right to cancel.");
            telemetry.update();

            drive.update();
        }

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
                }
            }

            drive.setMotorPowers(0, 0, 0, 0);

            ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);

            if(ResolvedX > -ResolvedAccuracy && ResolvedX < ResolvedAccuracy || timer.milliseconds() > 950){
                ReadyToFire = true;
            }
        }

        while (isStopRequested() != true && opModeIsActive() && ReadyToFire == true){

            telemetry.addData("Flywheel Voltage ", drive.flyWheel.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Firing", ": Yes");
            drive.TreadGate.setPosition(0.4);

            if(drive.flyWheel.getVelocity(DEGREES) > 518){
                drive.Tread.setPower(0.6);
            } else {
                drive.Tread.setPower(0);
            }

            /*if(gamepad1.dpad_right || gamepad2.dpad_right){
                ReadyToFire = false;
            }*/

            if(gamepad1.dpad_up || gamepad2.dpad_up){
                ReadyToFire = false;
                drive.setPoseEstimate(new Pose2d(0, 0, 0));
                drive.setPoseEstimate(new Pose2d(0, 0, 0));
            } else if (gamepad2.dpad_down || gamepad1.dpad_down || gamepad2.dpad_right || gamepad1.dpad_right) {
                ReadyToFire = false;
            }

            telemetry.addData("Flywheel Velocity", drive.flyWheel.getVelocity(DEGREES));
            telemetry.addData("Target X Pos", pipeline.getCenterofRect(pipeline.getRedRect()).x);

            telemetry.update();
        }

        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.7);
        timer.reset();
    }

    /** GENERATE A TRAJECTORY ON THE FLY AND FOLLOW IT TO THE SET LOCATION */
    public void GoToViaSpline(SampleMecanumDrive drive, Pose2d poseEstimate, int X, int Y, int Heading, int Tangent){

        //Generate a trajectory splineToLinearHeading on the fly
        Trajectory traj = drive.trajectoryBuilder(poseEstimate)
                //.splineToLinearHeading(new Pose2d(X, Y, Math.toRadians(Heading)), Math.toRadians(Tangent))
                .lineToLinearHeading(new Pose2d(X, Y, Math.toRadians(Heading)))
                .build();

        //Follow the trajectory
        drive.followTrajectoryAsync(traj);

        //While waiting for the trajectory to finish, make sure to update the drive systems
        while(drive.isBusy()){
            drive.update();
            //Display telemetry data
            telemetry.addData("RR Pose X ", drive.getPoseEstimate().getX());
            telemetry.addData("RR Pose Y ", drive.getPoseEstimate().getY());
            telemetry.addData("RR Pose Heading ", drive.getPoseEstimate().getHeading());
            telemetry.update();


        }

        //Generate a trajectory splineToLinearHeading on the fly
        /*Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(X, Y, Math.toRadians(Heading)), Math.toRadians(Tangent))
                .build();

        //Follow the trajectory
        drive.followTrajectory(traj1);

        //While waiting for the trajectory to finish, make sure to update the drive systems
        while(drive.isBusy()){
            drive.update();
            //Display telemetry data
            telemetry.addData("RR Pose X ", drive.getPoseEstimate().getX());
            telemetry.addData("RR Pose Y ", drive.getPoseEstimate().getY());
            telemetry.addData("RR Pose Heading ", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }*/

    }

}