 package org.firstinspires.ftc.teamcode.Tele;

import androidx.annotation.RequiresPermission;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.FTCLib.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

@TeleOp(group = "Crimson Solace")
public class CrimsonSolace extends LinearOpMode {

    private UGBasicHighGoalPipeline pipeline;
    private UGContourRingPipeline ringPipeline;

    OpenCvCamera webcam;
    OpenCvCamera FrontCamera;

    ColorSensor Low_Sensor;
    ColorSensor High_Sensor;

    //NormalizedColorSensor RightGround_Sensor;
    //NormalizedColorSensor LeftGround_Sensor;


    //Automatic Heading Correction Values
    ElapsedTime StoredDelay = new ElapsedTime();
    double StoredFinalDelay;
    boolean CalculatingHeading = false;
    String FirstPass = new String();




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




        telemetry.addData("Initialization step: ", "Loading RoadRunner");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(PoseStorage.currentPose);
        //drive.wobbleRotate.setPosition(1);
        //drive.setPoseEstimate(new Pose2d(0, 0, 0));


        Low_Sensor = hardwareMap.colorSensor.get("LowSensor");
        High_Sensor = hardwareMap.colorSensor.get("HighSensor");
        /*RightGround_Sensor = hardwareMap.get(NormalizedColorSensor.class, "RightGround");
        LeftGround_Sensor = hardwareMap.get(NormalizedColorSensor.class, "LeftGround");
        RightGround_Sensor.setGain(2);
        LeftGround_Sensor.setGain(2);*/



        double speedArc = .5;
        double speedArcStorage = .5;

        Pose2d savePose;


        int Mode = 0;

        boolean isDiskPrimed = false;
        ElapsedTime timerColor = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime MatchTimer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();

        double RotatePose = 1;
        //RotatePose = drive.wobbleRotate.getPosition();

        double ClampPose = 0;
        //ClampPose = drive.wobbleClamp.getPosition();

        telemetry.addLine("Waiting for start");
        telemetry.addData("BetaAlfa Version : ", "1.06");
        telemetry.update();




        waitForStart();

        if (isStopRequested()) return;

        timer.reset();
        MatchTimer.reset();
        timer2. reset();
        timer3.reset();
        drive.TreadGate.setPosition(.65);

        drive.wobbleClamp.setPosition(0);

        drive.wobbleRotate.setPosition(1);






        while (opModeIsActive() && !isStopRequested()) {


            /** ROADRUNNER RUNTIME INITIALIZATION STEPS */

            Pose2d poseEstimate = drive.getPoseEstimate();

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

            {
                if (ringPipeline.getHeight() != UGContourRingPipeline.Height.ZERO) {
                    isDiskPrimed = true;
                    timer2.reset();
                } else {
                    speedArc = speedArcStorage;
                }

                if (isDiskPrimed == true && timer2.milliseconds() < 800) {
                    drive.Intake.setPower(-1);
                    drive.Tread.setPower(1);
                } else {
                    isDiskPrimed = false;
                    drive.Tread.setPower(-gamepad2.right_stick_y);
                    if (gamepad2.right_stick_y != 0) {
                        drive.Intake.setPower(gamepad2.right_stick_y);
                    } else {
                        drive.Intake.setPower(-.2);
                    }
                }
            }

            //End automatic pickup


            /** AUTOMATIC HEADING CORRECTION */

            /*{
                NormalizedRGBA left_colors = LeftGround_Sensor.getNormalizedColors();
                NormalizedRGBA right_colors = RightGround_Sensor.getNormalizedColors();


                if ((left_colors.red > 420 && left_colors.green > 420 && left_colors.blue > 420) && CalculatingHeading != true) {
                    CalculatingHeading = true;
                    FirstPass = "LEFT";
                    StoredDelay.reset();
                } else if ((right_colors.red > 420 && right_colors.green > 420 && right_colors.blue > 420) && CalculatingHeading != true) {
                    CalculatingHeading = true;
                    FirstPass = "RIGHT";
                    StoredDelay.reset();
                }

                if (CalculatingHeading == true && FirstPass == "LEFT" && (right_colors.red > 420 && right_colors.green > 420 && right_colors.blue > 420)) {
                    StoredFinalDelay = StoredDelay.milliseconds();
                    CalculatingHeading = false;
                }
                if (CalculatingHeading == true && FirstPass == "RIGHT" && (left_colors.red > 420 && left_colors.green > 420 && left_colors.blue > 420)) {
                    StoredFinalDelay = StoredDelay.milliseconds();
                    CalculatingHeading = false;
                }
                telemetry.addData("Delay: ", StoredFinalDelay);
            }*/

            //End Automatic Heading Correction



            /** MANUAL OVERRIDES AND CONTROLS*/
            //Keep the fly wheel spinning at full speed

            drive.flyWheel.setVelocity(-295, DEGREES);


            //If left Dpad is triggered, go to autofire.
            if(gamepad2.dpad_left || gamepad1.dpad_left || gamepad1.left_bumper || gamepad1.right_bumper) {

                AutoFire(timer4, drive, poseEstimate);

            }


            //Reset the Robot Pose
            if(gamepad1.start)
                drive.setPoseEstimate(new Pose2d(0, 0, 0));

            if(gamepad1.back) {
                savePose = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(savePose.getX(), savePose.getY(), 180));
            }

            //Speedarc Controls
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

            if(gamepad2.a){
                drive.wobbleClamp.setPosition(0);
                ClampPose = 0;
            }
            else if (gamepad2.b){
                drive.wobbleClamp.setPosition(.35);
                ClampPose = 1;
            }

            if(gamepad2.y){
                drive.wobbleRotate.setPosition(1);
            }
            else if (gamepad2.x){
                drive.wobbleRotate.setPosition(.6);
            }

            //Clamp: 0 is Closed, 1 is Open
            //Rotate: 0 is Down, 1 is Up

            //Clamp controls
            /*if(gamepad2.a && timer2.milliseconds() > 300){

                if(ClampPose == 0) {

                    //Open the Clamp fully
                    drive.wobbleClamp.setPosition(.35);
                    ClampPose = 1;

                    if(RotatePose == 1) {
                        //Move the Rotator down
                        drive.wobbleRotate.setPosition(.6);
                        RotatePose = 0;
                    }

                }

                else if (ClampPose == 1) {

                    //Close the Clamp fully
                    drive.wobbleClamp.setPosition(0);
                    ClampPose = 0;

                }

                timer2.reset();

            }

            //Rotator controls
            if(gamepad2.b && timer3.milliseconds() > 300){

                //If the Rotator is Down already
                if(RotatePose == 0){

                    //Lift the Rotator fully
                    drive.wobbleRotate.setPosition(1);
                    RotatePose = 1;

                    if(ClampPose == 1){

                        //Close the Clamp
                        drive.wobbleClamp.setPosition(0);
                        ClampPose = 0;

                    }



                }

                //If the Rotator is Up already
                else if(RotatePose == 1){

                    //Drop the Rotator
                    drive.wobbleRotate.setPosition(.6);
                    RotatePose = 0;

                    if(ClampPose == 0){

                        //Open the Clamp
                        drive.wobbleClamp.setPosition(.35);
                        ClampPose = 1;

                    }

                }

                timer3.reset();

            }*/

            //Wobble Lift Controls
            drive.wobbleLift.setPower(-gamepad2.left_stick_y);

            telemetry.addData("Clamp ", drive.wobbleClamp.getPosition());
            telemetry.addData("Rotate ", drive.wobbleRotate.getPosition());


            //Manually overriden shooter controls
            if(gamepad2.dpad_up){
                drive.TreadGate.setPosition(.4);
                drive.Tread.setPower(1);
            } else {
                drive.TreadGate.setPosition(.65);
            }


            /** ASSORTED TELEMETRY SYSTEMS */

            {
                telemetry.addData("Raw positions", drive.getWheelPositions());
                //telemetry.addData("Null", StandardTrackingWheelLocalizer.)
                telemetry.addData("RR Pose X ", drive.getPoseEstimate().getX());
                telemetry.addData("RR Pose Y ", drive.getPoseEstimate().getY());
                telemetry.addData("RR Pose Heading ", Math.toDegrees(drive.getPoseEstimate().getHeading()));
                /*if(pipeline.isRedVisible()) {
                    telemetry.addData("Img X Pos", pipeline.getRedRect().x);
                    telemetry.addData("Img Pos", pipeline.getCenterofRect(pipeline.getRedRect()));
                    telemetry.addData("Img Size", pipeline.getRedRect());
                }*/
                telemetry.addData("Flywheel Amperage ", drive.flyWheel.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
        }
    }

    /** AUTOMATIC SALVO SYSTEMS */
    public void AutoFire(ElapsedTime timer, SampleMecanumDrive drive, Pose2d poseEstimate) {
        //VERSION 1.05a-t-s
        //Code: [Current code][Extra code]-[Tele/Auto]-[Experimental/Stable]
        /**
         * Changelog:
         * 1.02e: Added Automatic shutoff via color sensor recognition. Added assorted comments.
         * 1.03e: Removed Automatic shutoff via color sensor. Added wait for start in an attempt to remove first shot misses.
         * 1.03a-s: Working consistently, only change if in error.
         * 1.03b-t-s: Adjusted the speed of the robot during Manual Override.
         * 1.04a-t-e: Required drive to create full path segment. Added operator controls. Cleaned up code. Lowered kV.
         * 1.05a-t-s: Removed time based vision cycles. Working consistently.
         */
        double ResolvedX;
        int TargetX = 145;
        boolean ReadyToFire = false;
        double ResolvedAccuracy = 0.001;
        double kV = .003;
        double flyVel = 0;

        telemetry.update();

        //Turn on assorted systems
        drive.Tread.setPower(0.0);
        drive.Intake.setPower(-0.6);
        drive.TreadGate.setPosition(0.65);

        //Initiate Automatic movement via Roadrunner
        if(drive.getPoseEstimate() != new Pose2d(0, 0, 0)) {
            GoToViaSpline(drive, poseEstimate, 0, 0, 0);
        }

        //Allow the driver to adjust t he position of the robot
        while(gamepad1.dpad_right == false && gamepad2.dpad_right == false && gamepad1.dpad_left == false && gamepad2.dpad_left == false){
            //Manual movement controls
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * .3,
                            -gamepad1.left_stick_x * .3,
                            -gamepad1.right_stick_x * .3
                    )
            );

            //Operator controls
            drive.Tread.setPower(-gamepad2.right_stick_y);
            if(gamepad2.right_stick_y != 0) {
                drive.Intake.setPower(gamepad2.right_stick_y);
            } else {
                drive.Intake.setPower(-.4);
            }

            telemetry.addLine("Manually Aligning. Press Dpad Right to cancel.");
            telemetry.update();

            drive.update();
        }

        telemetry.update();

        //Is the high goal visible? If yes, continue firing routine.
        /*if(pipeline.isRedVisible()){

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
        }*/

        ReadyToFire = true;

        /*if(pipeline.isRedVisible()){

            ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
            timer.reset();

            while(timer.milliseconds() < 2000 && isStopRequested() != true && opModeIsActive()){
                telemetry.addLine("Auto Aligning. Press Dpad Right to cancel.");
                telemetry.addData("Runtime: ",  timer.milliseconds());
                telemetry.update();
                ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
                drive.setMotorPowers(ResolvedX, ResolvedX, -ResolvedX, -ResolvedX);
            }

            drive.setMotorPowers(0, 0, 0, 0);
            ReadyToFire = true;

        }*/

        while(flyVel < 285){
            flyVel = drive.flyWheel.getVelocity() * -1;
            drive.update();
            telemetry.addData("Flywheel Velocity:", drive.flyWheel.getVelocity(DEGREES));
            telemetry.addData("FlyVel:", flyVel);
            telemetry.update();
        }

        while (isStopRequested() != true && opModeIsActive() && ReadyToFire == true){

            flyVel = drive.flyWheel.getVelocity() * -1;

            telemetry.addData("Flywheel Voltage ", drive.flyWheel.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Firing", ": Yes");



            //If the flywheel is at the correct velocity, open the gate and fire
            if(flyVel > 285){
                drive.Tread.setPower(0.6);
                drive.TreadGate.setPosition(0.4);
            } else {
                drive.Tread.setPower(0);
            }

            //Manual operator overrides. If Up is pressed, save the current position
            //However, if down or right is pressed, DON'T save the position
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
        drive.TreadGate.setPosition(.65);
        timer.reset();
    }

    /** GENERATE A TRAJECTORY ON THE FLY AND FOLLOW IT TO THE SET LOCATION */
    public void GoToViaSpline(SampleMecanumDrive drive, Pose2d poseEstimate, int X, int Y, int Heading) {

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

    }

    /** AUTOMATIC HEADING CORRECTION */
    public void AutoCorrectHeading(SampleMecanumDrive drive) {

    }

}