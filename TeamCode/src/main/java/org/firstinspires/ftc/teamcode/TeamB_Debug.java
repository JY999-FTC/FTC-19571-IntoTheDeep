/*
Author: Jerome Yang
Start: 9/19/2024
End: #/##/####
Purpose: FTC 19571 The Robo Brigade Team B 24-25 season robot code
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TeamB_Debug extends LinearOpMode{

    int sleep = 100;


    int extendSlide_target_position = 0;
    int extendSlide_target_speed = 0;

    int rotateSlide_target_position = 0;
    int rotateSlide_target_speed = 0;

    int linearSlide_amount = 50;

    double grab_rotate_power = 0;
    double hand_rotate_target = 0;
    double arm_rotate_target = 0;

    double servo_amount = 0.02;

    double grab_rest = 0;
    double grab_intakePower = -0.4;
    double grab_outtakePower = 0.4;

    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    DcMotorEx rotateSlideMotor;
    DcMotorEx extendSlideMotor;

    CRServo grabServo;
    Servo handServo;
    Servo armServo;

    ColorSensor colorSensor;

    @Override

    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration2
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        rotateSlideMotor = hardwareMap.get(DcMotorEx.class, "rotateSlideMotor");
        extendSlideMotor = hardwareMap.get(DcMotorEx.class, "extendSlideMotor");

        grabServo = hardwareMap.get(CRServo.class, "grabServo");
        handServo = hardwareMap.get(Servo.class, "handServo");
        armServo = hardwareMap.get(Servo.class, "armServo");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //reverse because of gears or something
        extendSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Reset the linear slide
        extendSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendSlideMotor.setTargetPosition(0);
        extendSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendSlideMotor.setVelocity(0);

        rotateSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateSlideMotor.setTargetPosition(0);
        rotateSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateSlideMotor.setVelocity(0);

        // Set positions of motors/servos and move it.
        extendSlide_target_position = 0; extendSlide_target_speed = 100;
        rotateSlide_target_position = 0; rotateSlide_target_speed = 100;

        grab_rotate_power = 0.2;
        hand_rotate_target = 0.5;
        arm_rotate_target = 0.5;

        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);

        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);

        grabServo.setPower(grab_rotate_power);
        handServo.setPosition(hand_rotate_target);
        armServo.setPosition(arm_rotate_target);

        //Before is all initialized
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // drive the robot
            controllerDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x);
            rotateSlide();
            extendSlide();
            armRotate();
            handRotate();
            grabRotate();
            changeSlide();
            changeServo();

            // update telemetry or like what you see
            telemetry.addData("linearSlide change: ", linearSlide_amount);
            telemetry.addData("servo change: ", servo_amount);
            telemetry.addLine();

            telemetry.addData("Target Length of Linear slide:", extendSlide_target_position);
            telemetry.addData("Length of linear slide:", extendSlideMotor.getCurrentPosition());
            telemetry.addLine();

            telemetry.addData("Target Rotation of Linear slide:", rotateSlide_target_position);
            telemetry.addData("Rotation of linear slide:", rotateSlideMotor.getCurrentPosition());
            telemetry.addLine();

            telemetry.addData("Target of Arm Servo:", arm_rotate_target);
            telemetry.addData("Arm Servo:", armServo.getPosition());
            telemetry.addLine();

            telemetry.addData("Target of Hand Servo:", hand_rotate_target);
            telemetry.addData("Hand Servo:", handServo.getPosition());
            telemetry.addLine();

            telemetry.addData("Power of Grab Servo:", grab_rotate_power);
            telemetry.addData("Grab Servo:", grabServo.getPower());
            telemetry.addLine();

            telemetry.addData("2 TouchPad X: ", gamepad2.touchpad_finger_1_x);
            telemetry.addData("2 TouchPad Y: ", gamepad2.touchpad_finger_1_y);
            telemetry.addLine();

            telemetry.addData("1 Left Joystick X", gamepad1.left_stick_x);
            telemetry.addData("1 Left Joystick Y", gamepad1.left_stick_y);
            telemetry.addLine();

            telemetry.addData("1 Right Joystick X", gamepad1.right_stick_x);
            telemetry.addData("1 Right Joystick Y", gamepad1.right_stick_y);
            telemetry.addLine();

            telemetry.addData("2 Left Trigger", gamepad2.left_trigger);
            telemetry.addData("2 Right Trigger", gamepad2.right_trigger);
            telemetry.addLine();

            telemetry.addData("Color Sensor R: ", colorSensor.red());
            telemetry.addData("Color Sensor G: ", colorSensor.green());
            telemetry.addData("Color Sensor B: ", colorSensor.blue());
            telemetry.addLine();

            telemetry.update();

        }//op mode active end

    } // run OpMode end

    // ALL THE METHODS FUNCTIONS WHATEVER BELOW
    public void controllerDrive(double rightStickX, double rightStickY, double leftStickX){
        double x = rightStickX * 1.1; // Counteract imperfect strafing
        double y = -rightStickY; // Remember, Y stick value is reversed
        double rx = leftStickX * 0.7; // turning speed

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower * 0.5);
        frontRightMotor.setPower(frontRightPower * 0.5);
        backLeftMotor.setPower(backLeftPower * 0.5);
        backRightMotor.setPower(backRightPower * 0.5);
    }// controller drive end
    public void rotateSlide(){
        if (gamepad2.dpad_up){
            rotateSlide_target_position += linearSlide_amount;
            sleep(sleep);
        }
        else if (gamepad2.dpad_down){
            rotateSlide_target_position -= linearSlide_amount;
            sleep(sleep);
        }
        rotateSlide_target_speed = 500;
        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);
    }// rotate slide end

    public void extendSlide(){
        if (gamepad2.dpad_left){
            extendSlide_target_position -= linearSlide_amount;
            sleep(sleep);
        }
        else if (gamepad2.dpad_right){
            extendSlide_target_position += linearSlide_amount;
            sleep(sleep);
        }
        extendSlide_target_speed = 500;
        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);
    }// extend slide end
    public void armRotate(){
        if (gamepad2.left_bumper){
            arm_rotate_target -= servo_amount;
        }
        else if (gamepad2.right_bumper){
            arm_rotate_target += servo_amount;
        }
        armServo.setPosition(arm_rotate_target);
        sleep(sleep);
    }// arm rotate end
    public void handRotate(){
        if (gamepad2.x) {
            hand_rotate_target -= servo_amount;
        }
        else if (gamepad2.y){
            hand_rotate_target += servo_amount;
        }
        handServo.setPosition(hand_rotate_target);
        sleep(sleep);
    }// hand rotate end
    public void grabRotate(){
        if (gamepad2.a){
            grab_rotate_power = grab_intakePower;
        }
        else if (gamepad2.b){
            grab_rotate_power = grab_outtakePower;
        }
        else{
            grab_rotate_power = grab_rest;
        }
        grabServo.setPower(grab_rotate_power);
        sleep(sleep);
    }// arm rotate end
    public void changeSlide(){
        if (gamepad1.dpad_up){
            linearSlide_amount += 25;
            sleep(250);
        }
        else if (gamepad1.dpad_down){
            linearSlide_amount -= 25;
            sleep(250);
        }
    }
    public void changeServo(){
        if (gamepad1.y){
            servo_amount += 0.01;
            sleep(250);
        }
        else if (gamepad1.a){
            servo_amount -= 0.01;
            sleep(250);
        }
    }

} // class end
