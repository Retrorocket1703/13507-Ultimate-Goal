package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file represents an attempt at Intentional Tracking (India Tango), through use of an Autonomous program.
 *
 * Phonetics: Program names cycle through Alfa, Bravo, Charlie, and Delta plus another phonetic.
 * Names of items cycle through everything past Delta.
 */
@Disabled
@Autonomous(name="Alfa Bravo", group="Alfa")

public class AlfaBravo extends LinearOpMode {

    // Declare OpMode members.

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        //Telemetry
        telemetry.addData("Status", "Initialized [INDIA DELTA]");
        telemetry.update();
        leftDrive  = hardwareMap.get(DcMotor.class, "motor");

        //Initialize Motors
        /*leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);*/


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            leftDrive.setPower(1);

        }
    }
}
