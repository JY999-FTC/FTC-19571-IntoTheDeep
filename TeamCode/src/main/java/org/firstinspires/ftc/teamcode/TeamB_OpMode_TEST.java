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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class TeamB_OpMode_TEST extends LinearOpMode {

    // tim stuff idk
    public enum State {
        IDLE,
        INTAKE,
        CONVERT,
        LIFT,
        OUTTAKE,
    }
    State state = State.INTAKE;

    // time variables??? copied from tim... hee hee haa haa
    public ElapsedTime runtime = new ElapsedTime();
    boolean[] flag = new boolean[20];
    double[] stoptime = new double[20];
    int preintake = 0;
    int adjustment = 1;
    int specimen = 2;
    int specimenConvert = 3;
    int hang = 4;
    int firstTime = 5;


    // Status of Robot Actions
    int step = 0;
    int reset_RotateAmount = 0;
    int sampleIntake_Position = 1;

    double drive_factor = 1;

    double as = 0.7; // arm servo

    //arrays # system start at 0-infinite. I start with 0 @ 0 b/c I want it to start at 1 for 1st value
    double[] array_arm    = new double[]{0, as-0.03   , as-0.03    , as-0.03   , as-0.035  , as-0.035  , as-0.035  , as-0.035  , as-0.04   , as-0.04   , as-0.04   , as-0.045  , as-0.045  , as-0.045  , as-0.045   };
    int[]array_rotate_intakeup=new int[]{0, 100       , 85         , 70        , 60        , 52        , 44        , 38        , 32        , 28        , 24        , 20        , 18        , 16        , 14         };
    int[] array_rotate    = new int[]   {0, -100      , -85        , -70       , -60       , -52       , -44       , -38       , -32       , -28       , -24       , -20       , -18       , -16       , -14        };
    double[] array_extend = new double[]{0, 1.26      , 1.22       , 1.18      , 1.14      , 1.10      , 1.06      , 1.02      , 1.02      , 1.02      , 1.01      , 1.01      , 1.01      , 1.01      , 1.01       };


    // extend linear slide variable
    int extendSlide_target_position = 0;
    int extendSlide_rest_position = 0;
    int extend_intakeSample = 800 - 465;// 800 - 465 (intake interval) b/c start at position 1
    int extendSlide_intakeSample_Interval = 465;
    int extend_intakeSpecimens = 0;
    int extend_sampleHigh = 10000;
    int extend_specimenHigh = 3500;
    int extend_specimenHigh_down = extend_specimenHigh - 2000;

    int extend_preintake_position = 1265;
    int extend_postintake_position = 1265;

    int extendSlide_target_speed = 0;

    int currentSampleIntake_Position;// = (extend_intakeSample + extendSlide_intakeSample_Interval * sampleIntake_Position);


    // rotate linear slide variable
    int rotateSlide_target_position = 0;
    int rotateSlide_rest_position = 100;
    int rotate_intakeSample = 0;
    int rotate_intakeSpecimens = 160;
    int rotate_sampleHigh = 900; // sample low = 900    specimen low = 480
    int rotate_specimenHigh = 850;

    int rotate_adjust_intake = 0;

    int rotateSlide_target_speed = 0;

    double grab_rotate_power = 0;
    double grab_intakePower = -0.6;
    double grab_intakeMaintain = -0.2;
    double grab_outtakePower = 0.3;
    double grab_rest = 0;

    // hand servo variable
    double hand_rotate_target = 0.63;
    double hand_perpendicular = 0.63; // 45 right = 0.34 45 left = 0.70 parallel = 0.16
    double hand_parallel = 0.94;
    double hand_intake = 0.94;

    // arm servo variable
    double arm_rotate_target = 0.7;
    double arm_down = 0.7;
    double arm_SampleIntake = 0.5;
    double arm_SpecimenIntake = 0.26;
    double arm_sampleHigh = 0.68;

    // motor and servo definitions
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    DcMotorEx rotateSlideMotor;
    DcMotorEx extendSlideMotor;

    CRServo grabServo; // alpha servo b/c can rotate continuisly rotating so CR servo idk
    Servo handServo;
    Servo armServo;

    ColorSensor colorSensor;

    @Override

    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure ID's match the configuration
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

        // reverse because it the only one spinning in wrong direction idk
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // reverse because of gears or something
        extendSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the linear slide
        rotateSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateSlideMotor.setPower(0);
        rotateSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateSlideMotor.setTargetPosition(0);
        rotateSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateSlideMotor.setVelocity(0);

        extendSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendSlideMotor.setTargetPosition(0);
        extendSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendSlideMotor.setVelocity(0);

        // make the motors stop if zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set positions of motors/servos
        extendSlide_target_position = extendSlide_rest_position;
        extendSlide_target_speed = 0;
        rotateSlide_target_position = rotateSlide_rest_position;
        rotateSlide_target_speed = 500;

        grab_rotate_power = grab_rest;
        hand_rotate_target = hand_perpendicular;
        arm_rotate_target = arm_down;

        // move the motors/servos
        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);

        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);

        sleep(500);

        grabServo.setPower(grab_rotate_power);
        handServo.setPosition(hand_rotate_target);
        armServo.setPosition(arm_rotate_target);

        // to prevent the robot from moving, idk
        rotateSlide_target_speed = 0;
        rotateSlideMotor.setVelocity(rotateSlide_target_speed);

        // Before is all initialized
        waitForStart();

        if (isStopRequested()) return;


        // State you start at
        // flag[specimen] = false;// automaticlly false so its sample intak
        state = State.IDLE;


        // While running
        while (opModeIsActive()) {

            // drive the robot
            controllerDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x);

            // reset linear slide
            if (gamepad2.y || gamepad2.a) {
                resetLinearSlide();
            }

            switch (state) {
                case IDLE:

                    if (flag[firstTime]){
                        drive_factor = 1;
                        setIdle();// set slowly go to IDLE position
                        flag[firstTime] = false;
                    }

                    if (gamepad2.options && timera(400, specimen)) {
                        flag[specimen] = !flag[specimen];
                        timera(0, specimen);
                    }

                    // change if need to go to intake sample first for specimen
                    if (gamepad2.dpad_up && timera(400, specimenConvert)){
                        flag[specimenConvert] = !flag[specimenConvert];
                        timera(0, specimenConvert);
                    }

                    if (gamepad2.right_bumper){
                        state = State.INTAKE;
                    }


                    break;

                case INTAKE:

                    if (gamepad2.left_bumper){
                        state = State.IDLE;
                        flag[firstTime] = true;
                    }

                    if (!flag[specimen]) {
                        if (gamepad2.left_bumper || flag[preintake]) {
                            preintake_sample();
                        }
                        if (gamepad2.left_stick_x != 0 || gamepad2.right_stick_y != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0 || sampleIntake_Position != currentSampleIntake_Position) {
                            adjustSampleIntake(gamepad2.left_stick_x, gamepad2.right_stick_y, sampleIntake_Position);
                        }
                        if (gamepad2.right_bumper) {
                            grabSample();
                            state = State.LIFT;
                        }
                    }// sample intake end
                    else if (flag[specimen] && !flag[specimenConvert]) {

                        if (gamepad2.left_bumper) {
                            preintake_specimen();
                        }
                        if (gamepad2.right_bumper) {
                            grabSpecimen();
                            drive_factor = 1;
                            state = State.CONVERT;
                        }
                    }// specimen intake end
                    else if (flag[specimen] && flag[specimenConvert]) {

                        if (gamepad2.left_bumper || flag[preintake]) {
                            preintake_sample();
                        }
                        if (gamepad2.left_stick_x != 0 || gamepad2.right_stick_y != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0 || sampleIntake_Position != currentSampleIntake_Position) {
                            adjustSampleIntake(gamepad2.left_stick_x, gamepad2.right_stick_y, sampleIntake_Position);
                        }
                        if (gamepad2.right_bumper) {
                            grabSample();
                            state = State.CONVERT;
                        }
                    }// sample intake for specimen convert end

                    break;


                case CONVERT:// only for specimen

                    if (gamepad2.left_bumper){
                        state = State.INTAKE;
                        flag[firstTime] = true;
                    }

                    if (gamepad2.dpad_down){
                        dropSample();
                    }
                    if (gamepad2.dpad_up) {
                        grabSpecimen();
                    }
                    if (gamepad2.right_bumper){
                        state = State.OUTTAKE;
                    }

                    break;

                case LIFT:// only for sample

                    if (gamepad2.left_bumper){
                        state = State.INTAKE;
                        flag[firstTime] = true;
                    }

                    if (gamepad1.right_bumper) {
                        sampleLift();
                        drive_factor = 0.4;
                    }
                    if (extendSlideMotor.getCurrentPosition() > 9000) {
                        state = State.OUTTAKE;
                    }
                    if (gamepad2.left_bumper) {
                        intake_drop_sample();
                        state = State.INTAKE;
                    }
                    break;


                case OUTTAKE:

                    if (gamepad2.left_bumper){
                        state = State.INTAKE;
                        flag[firstTime] = true;
                    }

                    if (!flag[specimen]) {

                        if (gamepad2.right_bumper) {
                            moveAllServo(grab_outtakePower, hand_perpendicular, arm_sampleHigh, 0);
                            sleep(500);
                            move_grabServo(grab_rest);
                            moveDrive(-0.3);
                            sleep(700);
                            stopDrive();
                            move_extendSlide(extend_postintake_position, 6000);
                            sleep(2000);
                            move_rotateSlide(rotateSlide_rest_position, 2000);
                            state = State.IDLE;
                        }
                    } // if sample outtake end
                    else if (flag[specimen]) {
                        if (gamepad2.left_bumper) {
                            intake_drop_specimen();
                            state = State.IDLE;
                        }
                        if (gamepad2.right_bumper) {
                            specimenScore();
                            intake_drop_specimen();
                            state = State.IDLE;
                        }
                    } // if specimen outtake end


                    break;
            }// switch end


            // hang
            if (gamepad1.dpad_up || flag[hang]) {
                hang();
            }// hand end

            // update telemetry or like what you see
            updateTelementry();

        }// op mode active end

    } // run OpMode end

    // after OpMode, still in class!!!
    // ALL THE METHODS FUNCTIONS WHATEVER BELOW

    // DC Motor functions
    public void controllerDrive(double rightStickX, double rightStickY, double leftStickX) {
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

        frontLeftMotor.setPower(frontLeftPower * drive_factor);
        frontRightMotor.setPower(frontRightPower * drive_factor);
        backLeftMotor.setPower(backLeftPower * drive_factor);
        backRightMotor.setPower(backRightPower * drive_factor);
    }// controller drive end

    public void stopDrive() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }// stop drive end

    public void moveDrive(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }// move drive end

    public void move_extendSlide(int position, int speed) {
        extendSlide_target_position = position;
        extendSlide_target_speed = speed;
        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);
    }// move extend motor end

    public void move_rotateSlide(int position, int speed) {
        rotateSlide_target_position = position;
        rotateSlide_target_speed = speed;
        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);
    }// move rotate motor end

    public void moveAllSlide(int rotateSlide, int extendSlide, int sleepTime) {

        rotateSlide_target_speed = 2000;
        rotateSlide_target_position = rotateSlide;

        extendSlide_target_speed = 5000;
        extendSlide_target_position = extendSlide;

        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);

        sleep(sleepTime);

        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);

        sleep(sleepTime);
    }// moveSlide end

    // Servo functions
    public void move_grabServo(double target) {
        grab_rotate_power = target;
        grabServo.setPower(grab_rotate_power);
    }// move grab servo end

    public void move_handServo(double target) {
        hand_rotate_target = target;
        handServo.setPosition(hand_rotate_target);
    }// move hand servo end

    public void move_armServo(double target) {
        arm_rotate_target = target;
        armServo.setPosition(arm_rotate_target);
    }// move arm servo end

    public void moveAllServo(double grab, double hand, double arm, int sleepTime) {

        grab_rotate_power = grab;
        hand_rotate_target = hand;
        arm_rotate_target = arm;

        grabServo.setPower(grab_rotate_power);
        sleep(sleepTime);
        handServo.setPosition(hand_rotate_target);
        sleep(sleepTime);
        armServo.setPosition(arm_rotate_target);
        sleep(sleepTime);
    }// intake outtake end

    // For intake outtake functions
    public void setIdle(){

        moveAllSlide(rotateSlide_rest_position, extendSlide_rest_position,0);
        moveAllServo(grab_rest, hand_perpendicular, arm_down, 0);
    }

    // Sample
    public void preintake_sample() {
        if (!flag[preintake]) {
            move_extendSlide(extend_preintake_position, 3000);
            timera(0, preintake);
            flag[preintake] = true;
        }
        if (timera(1500, preintake)) {
            move_rotateSlide(rotate_intakeSample, 2000);
            flag[preintake] = false;
        }

    }// pre intake sample end

    public void grabSample() {

        moveAllSlide(array_rotate[sampleIntake_Position] += rotate_adjust_intake, (int) ((extend_intakeSample + extendSlide_intakeSample_Interval * sampleIntake_Position) * array_extend[sampleIntake_Position]), 0);
        move_rotateSlide(array_rotate[sampleIntake_Position] += rotate_adjust_intake, 3000);
        move_grabServo(grab_intakePower);
        sleep(1000);
        move_grabServo(grab_rest);
        moveAllSlide(rotateSlide_rest_position, extend_postintake_position, 0);

    }// grab sample end

    public void adjustSampleIntake(double leftX, double rightY, int position) {
        if (!timera(150, adjustment)) return;

        currentSampleIntake_Position = (int) ((extendSlideMotor.getCurrentPosition() - 335) / 465 + 0.5);

        if (rightY < -0.3) {
            sampleIntake_Position += 1;
        } else if (rightY > 0.3) {
            sampleIntake_Position -= 1;
        }
        if (sampleIntake_Position < 1) {
            sampleIntake_Position = 1;
        }
        if (sampleIntake_Position > 14) {
            sampleIntake_Position = 14;
        }

        move_handServo(hand_intake += (leftX * 0.1));

        if (hand_intake > 1) {
            hand_intake = 1;
        }
        else if (hand_intake < 0){
            hand_intake = 0;
        }

        moveAllServo(grab_rest, hand_intake, arm_SampleIntake, 0);
        moveAllSlide(array_rotate_intakeup[position], ((extend_intakeSample + extendSlide_intakeSample_Interval * position)), 0);
        timera(0, adjustment);

    }// adjust intake end

    public void sampleLift() {

        moveAllServo(grab_rest, hand_perpendicular, arm_sampleHigh, 0);
        moveAllSlide(rotate_sampleHigh, extend_sampleHigh, 0);

    }// sample lift end

    public void intake_drop_sample() {

        preintake_sample();
        //moveAllServo(grab_rest, hand_rotate_target, arm_rotate_target, 0);

    }// intake drop end

    // Specimen
    public void preintake_specimen() {

        moveAllServo(grab_rest, hand_perpendicular, arm_SpecimenIntake, 0);
        moveAllSlide(rotate_intakeSpecimens, extend_intakeSpecimens, 0);
    }// preintake_specimen

    public void grabSpecimen() {

        move_grabServo(grab_intakePower);
        moveDrive(0.2);
        sleep(750);
        stopDrive();
        moveAllServo(grab_rest, hand_perpendicular, arm_down, 0);
        moveAllSlide(rotate_specimenHigh, extend_specimenHigh, 0);
        sleep(450);
        moveDrive(-0.3);
        sleep(500);
        stopDrive();

    }// grab specimen end

    public void intake_drop_specimen() {

        preintake_specimen();
    }// intake drop specimen end

    public void dropSample() {

        moveAllSlide(rotateSlide_rest_position, extendSlide_rest_position, 0);
        sleep(100);
        move_grabServo(grab_outtakePower);
        sleep(300);
        move_grabServo(grab_rest);

    }// dropSample end

    public void specimenScore() {

        extendSlide_target_position = extend_specimenHigh_down;

        move_extendSlide(extendSlide_target_position, 4000);
        move_grabServo(grab_intakeMaintain);
        sleep(800);
        move_grabServo(grab_outtakePower);
        sleep(250);
        move_grabServo(grab_rest);
        moveDrive(-0.3);
        sleep(500);
    }// specimen score end

    // Other
    public void resetLinearSlide() {
        if (gamepad2.y) {
            reset_RotateAmount += 10;
        }
        if (gamepad2.a) {
            reset_RotateAmount -= 10;
        }
        if (reset_RotateAmount > 200) {
            reset_RotateAmount = 200;
        }
        if (reset_RotateAmount < -400) {
            reset_RotateAmount = -400;
        }

        rotateSlide_target_position = 0 + reset_RotateAmount;
        rotateSlide_rest_position = 100 + reset_RotateAmount;
        rotate_intakeSample = 0 + reset_RotateAmount;
        rotate_intakeSpecimens = 200 + reset_RotateAmount;
        rotate_sampleHigh = 900 + reset_RotateAmount; // sample low = 900    specimen low = 480
        rotate_specimenHigh = 850 + reset_RotateAmount;

        rotate_adjust_intake = 0 + reset_RotateAmount;

        rotateSlide_target_speed = 0;

        int[] array_rotate = new int[]{0, -120 + reset_RotateAmount, -105 + reset_RotateAmount, -90 + reset_RotateAmount, -80 + reset_RotateAmount, -72 + reset_RotateAmount, -64 + reset_RotateAmount, -58 + reset_RotateAmount, -52 + reset_RotateAmount, -48 + reset_RotateAmount, -44 + reset_RotateAmount, -40 + reset_RotateAmount, -38 + reset_RotateAmount, -36 + reset_RotateAmount, -34 + reset_RotateAmount};

        sleep(100);
    }// reset linear slide end

    public boolean timera(double period, int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;
    }// timera end

    public void hang() {
        //steps in team b discord
        if (step == 13) {
            return;
        }

        if (step == 0) {
            //stop motors
            stopDrive();
            drive_factor = 0;
            //move servos so not in the way
            move_grabServo(grab_rest);
            move_handServo(hand_perpendicular);
            move_armServo(arm_down);

            //1st stage 1st step: rotate the arm minus and point low rung
            move_rotateSlide(400 + reset_RotateAmount, 2000);
            step = 1;
            flag[hang] = true;
        }
        if (step == 1 && rotateSlideMotor.getCurrentPosition() > 380 + reset_RotateAmount) {
            //2nd: extend the motor plus
            move_extendSlide(4000, 5000);
            step = 2;
        }
        if (step == 2 && extendSlideMotor.getCurrentPosition() > 3980) {
            //3rd:
            move_rotateSlide(530 + reset_RotateAmount, 2000);
            step = 3;
        }
        if (step == 3 && rotateSlideMotor.getCurrentPosition() > 510 + reset_RotateAmount) {
            //4th
            move_extendSlide(-250, 4000);
            move_rotateSlide(1000 + reset_RotateAmount, 2000);
            //moveDrive(-0.2);
            step = 4;
        }
        if (step == 4 && extendSlideMotor.getCurrentPosition() < -200) {
            //5th
            sleep(500);
            move_extendSlide(500, 5000);
            step = 5;
        }
        if (step == 5 && extendSlideMotor.getCurrentPosition() > 480) {
            //6th
            move_rotateSlide(800 + reset_RotateAmount, 2000);
            step = 6;
        }
        if (step == 6 && rotateSlideMotor.getCurrentPosition() > 780 + reset_RotateAmount) {
            //7th
            move_extendSlide(5900, 5000);
            step = 7;
        }
        if (step == 7 && extendSlideMotor.getCurrentPosition() > 5880) {
            //8th hit the 2nd bar
            move_rotateSlide(1250 + reset_RotateAmount, 2000);
            step = 8;
        }
        if (step == 8 && rotateSlideMotor.getCurrentPosition() > 1230 + reset_RotateAmount) {
            //9th
            move_extendSlide(4600, 5000);
            step = 9;
        }
        if (step == 9 && extendSlideMotor.getCurrentPosition() < 4620) {
            //10th
            move_rotateSlide(550 + reset_RotateAmount, 2000);
            step = 10;
        }
        if (step == 10 && rotateSlideMotor.getCurrentPosition() < 570 + reset_RotateAmount) {
            //11th
            move_extendSlide(-400, 5000);
            step = 11;
        }
        if (step == 11 && extendSlideMotor.getCurrentPosition() < -350) {
            //12th
            sleep(300);
            move_rotateSlide(900 + reset_RotateAmount, 2000);
            step = 12;
        }
        if (step == 12 && rotateSlideMotor.getCurrentPosition() > 880 + reset_RotateAmount) {
            //13th
            sleep(1500);
            move_extendSlide(600, 5000);
            step = 13;
            flag[hang] = false;
        }
    }// hang end

    public void updateTelementry() {
        telemetry.addData("Elapsed time: ", runtime);
        telemetry.addData("Enum State", state);
        telemetry.addData("Specimens?", flag[specimen]);
        telemetry.addData("Specimen Convert?", flag[specimenConvert]);
        telemetry.addLine();
        telemetry.addData("Intake Position", sampleIntake_Position);
        telemetry.addData("Current Intake Position", currentSampleIntake_Position);
        telemetry.addData("Rotate Intake Adjust Intake", rotate_adjust_intake);
        telemetry.addData("Adjust Rotate", reset_RotateAmount);
        telemetry.addData("Hang Step", step);
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
        telemetry.addData("Target of Grab Servo:", grab_rotate_power);
        telemetry.addData("Grab Servo:", grabServo.getPower());
        telemetry.addLine();
        telemetry.addData("Color Sensor R: ", colorSensor.red());
        telemetry.addData("Color Sensor G: ", colorSensor.green());
        telemetry.addData("Color Sensor B: ", colorSensor.blue());
        telemetry.addLine();
        telemetry.update();
    }//update telementry end

} // class end