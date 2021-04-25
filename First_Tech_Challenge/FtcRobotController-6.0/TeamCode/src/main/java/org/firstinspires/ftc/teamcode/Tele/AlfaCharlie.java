package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Disabled
@TeleOp(name="Alfa Charlie", group="Alfa")
public class AlfaCharlie extends LinearOpMode{

    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private Servo liftServo = null;
    private DcMotor flyWheel = null;
    //private CRServo injector = null;
    private Servo injector = null;

    //private DcMotor Jack = null;

    //private Servo Injector = null;

    public void runOpMode(){

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        liftServo = hardwareMap.get(Servo.class, "liftServo");
        injector = hardwareMap.get(Servo.class, "injector");

        /*Jack = hardwareMap.get(DcMotor.class, "jack");
        Injector = hardwareMap.get(Servo.class, "injector");
        Jack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        float Gpad_Left_Y;
        float Gpad_Right_X;
        float left_Trigger;
        float right_Trigger;

        double leftFront_Pow = 0;
        double leftRear_Pow = 0;
        double rightFront_Pow = 0;
        double rightRear_Pow = 0;

        double Speed_Arc = .3;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
                //Initialize every frame
            Gpad_Left_Y = gamepad1.left_stick_y;
            Gpad_Right_X = gamepad1.right_stick_x;
            left_Trigger = gamepad1.left_trigger;
            right_Trigger = gamepad1.right_trigger;

            flyWheel.setPower(1);

            //Temporary Code
            //leftFront.setPower(Gpad_Right_X);
            /*Jack.setPower(Gpad_Left_Y);
            if(gamepad1.dpad_left)
                leftFront.setPower(1);
            else if (gamepad1.dpad_right)
                leftFront.setPower(-1);
            else if (gamepad1.dpad_down)
                leftFront.setPower(0);*/

            //Below is the movement algorithms
            leftFront_Pow = (-(Gpad_Right_X * Speed_Arc) + (Gpad_Left_Y * Speed_Arc) + -(left_Trigger * Speed_Arc) + (right_Trigger * Speed_Arc)) / 1.0;
            leftRear_Pow = (-(Gpad_Right_X * Speed_Arc) + -(Gpad_Left_Y * Speed_Arc) + -(left_Trigger * Speed_Arc) + (right_Trigger * Speed_Arc)) / 1.0;
            rightFront_Pow = ((Gpad_Right_X * Speed_Arc) + (Gpad_Left_Y * Speed_Arc) + -(left_Trigger * Speed_Arc) + (right_Trigger * Speed_Arc)) / 1.0;
            rightRear_Pow = ((Gpad_Right_X * Speed_Arc) + -(Gpad_Left_Y * Speed_Arc) + -(left_Trigger * Speed_Arc) + (right_Trigger * Speed_Arc)) / 1.0;

            //Adjust Speed Arc
            if(gamepad1.a == true)
                Speed_Arc = .3;
            if(gamepad1.b == true)
                Speed_Arc = .5;
            if(gamepad1.y == true)
                Speed_Arc = .7;
            if(gamepad1.x == true)
                Speed_Arc = 1;

            //Update Powers
            leftFront.setPower(leftFront_Pow);
            leftRear.setPower(leftRear_Pow);
            rightFront.setPower(rightFront_Pow);
            rightRear.setPower(rightRear_Pow);

            //Update Pose 2D
            //TODO: MAKE THIS WORK
            /*Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();*/



            if(gamepad2.dpad_up){
                liftServo.setPosition(1);
            } else if (gamepad2.dpad_down)
                liftServo.setPosition(0);

            //injector.setPower(-.7);
            if(gamepad2.a)
                injector.setPosition(.3);
            else if (gamepad2.b)
                injector.setPosition(1);
        }

    }

}
