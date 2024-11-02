/*
Author: Jerome Yang
Date: 9/21/2024
End: #/##/####
Purpose: FTC 19571 The Robo Brigade Team B 24-25 season robot
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class TeamB_Auto_Meet1 extends LinearOpMode {

//    DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
//    DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
//    DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
//    DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;




    @Override

    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // reverse because it the only one spinning in wrong direction idk
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // make the motors stop if zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        while (opModeIsActive()){

            moveDrive(0.25); //move forward a bit before strafe
            sleep(500);

            moveDrive(0); // stop before strafe
            sleep(500);

            strafe(0.3); // start strafing to the right
            sleep(3000);

            moveDrive(0); // stop robot
            sleep(50000);

            //return;
            //display values and stuff

        }


    }

    public void moveDrive(double power){
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void strafe(double power){
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);;
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
    }





}// class end
