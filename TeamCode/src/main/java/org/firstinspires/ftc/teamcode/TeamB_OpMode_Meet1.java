/*
Author: Jerome Yang
Start: 9/19/2024
End: #/##/####
Purpose: FTC 19571 The Robo Brigade Team B 24-25 season robot code
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp

public class TeamB_OpMode_Meet1 extends LinearOpMode {

    // tim stuff idk
    public enum State {
        INTAKE,
        LIFT,
        OUTTAKE,
    }

    State state = State.INTAKE;

    // time variables??? copied from tim... hee hee haa haa
    public ElapsedTime runtime = new ElapsedTime();
    boolean out_handling = false;
    double st = 0;// start time???
    int timePeriod = 200;
    boolean[] flag = new boolean[20];
    double[]  stoptime = new double[20];
    int preintake = 0, adjustment = 1, adjustmentRotate = 2, adjustmentGrab = 3,lift = 4, specimen = 5, hang = 6;


    // Status of Robot Actions
    double robot_Stage = 0;
    int step = 0;
    int reset_RotateAmount = 0;


    int sampleIntake_Position = 3;

    //arrays start at 0-infinite start with -1 b/c I want it to start at 1
    double[] array_arm    = new double[]{0, 0.7  , 0.7  , 0.7  , 0.695, 0.695, 0.695, 0.68 , 0.68 , 0.68 , 0.68 , 0.675, 0.675, 0.675, 0.675  };
    int[] array_rotate    = new int[]   {0, -145 , -128 , -111 , -96  , -81  , -74  , -70  , -65  , -60  , -56  , -52  , -48  , -44  , -40    };
    double[] array_extend = new double[]{0, 1.26 , 1.22 , 1.18 , 1.14 , 1.10 , 1.06 , 1.02 , 1.02 , 1.02 , 1.01 , 1.01 , 1.01 , 1.01 , 1.01   };


    double drive_factor = 1;
    // extend linear slide variable
    int extendSlide_target_position = 0;
    int extendSlide_rest_position = 0;
    int extend_intakeSample = 800 - 465;// 800 - 465 (intake interval) b/c start at position 1
    int extendSlide_intakeSample_Interval = 465;
    int extend_intakeSpecimens = 0;
    //int extend_sampleLow = 4600;
    int extend_sampleHigh = 9700;
    int extend_specimenLow = 2700;
    //int extend_specimenLow_down = extend_specimenLow - 1800;
    int extend_specimenHigh = 3600;
    int extend_specimenHigh_down = extend_specimenHigh - 1900;

    int extend_preintake_position = 1265;
    int extend_postintake_position = 1265;

    int extendSlide_target_speed = 0;

    int currentSampleIntake_Position;// = (extend_intakeSample + extendSlide_intakeSample_Interval * sampleIntake_Position);


    // rotate linear slide variable
    int rotateSlide_target_position = 0;
    int rotateSlide_rest_position = 100;
    int rotate_intakeSample = 0;
    int rotate_intakeSpecimens = 200;
    int rotate_sampleHigh = 900; // sample low = 900    specimen low = 480
    int rotate_specimenHigh = 850;

    int rotate_adjust_intake = 0;

    int rotateSlide_target_speed = 0;

    // grab servo variable
    double grab_adjust = 0;

    double grab_rotate_target = 0.2;
    double grab_open = 0.2;
    double grab_close = 0.49;

    // hand servo variable
    double hand_rotate_target = 0.52;
    double hand_perpendicular = 0.52; // 45 right = 0.34 45 left = 0.70 parallel = 0.16

    // arm servo variable
    double arm_rotate_target = 0.7;
    double arm_down = 0.7;
    double arm_lowered_SpecimenIntake = 0.28;
    double arm_sampleHigh = 0.68;

    // motor and servo definitions
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    DcMotorEx rotateSlideMotor;
    DcMotorEx extendSlideMotor;

    Servo grabServo;
    Servo handServo;
    Servo armServo;

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

        grabServo = hardwareMap.get(Servo.class, "grabServo");
        handServo = hardwareMap.get(Servo.class, "handServo");
        armServo = hardwareMap.get(Servo.class, "armServo");

        // reverse because it the only one spinning in wrong direction idk
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // reverse because of gears or something
        extendSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the linear slide
        rotateSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateSlideMotor.setPower(0);
        rotateSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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

        // Set positions of motors/servos and move it.
        extendSlide_target_position = extendSlide_rest_position; extendSlide_target_speed = 0;
        rotateSlide_target_position = rotateSlide_rest_position; rotateSlide_target_speed = 500;

        grab_rotate_target = grab_close;
        hand_rotate_target = hand_perpendicular;
        arm_rotate_target = arm_down;

        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);

//        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
//        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);

        sleep(1000);

        grabServo.setPosition(grab_rotate_target);
        handServo.setPosition(hand_rotate_target);
        armServo.setPosition(arm_rotate_target);

        state = State.OUTTAKE;
        flag[specimen] = true;

        // Before is all initialized
        waitForStart();

        rotateSlideMotor.setTargetPosition(0);
        rotateSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateSlideMotor.setVelocity(0);

        if (isStopRequested()) return;

        state = State.INTAKE;

        while (opModeIsActive()) {

            // drive the robot
            controllerDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x);

            if (gamepad1.x || gamepad1.y || gamepad2.x || gamepad2.y){
                resetLinearSlide();
            }

            switch (state) {
                case INTAKE:

                    drive_factor = 1;

                    if (gamepad2.options && timera(400,specimen)){
                        flag[specimen]=!flag[specimen];
                        timera(0,specimen);
                    }

                    if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0){
                        adjustGrab();
                    }

                    if (!flag[specimen]) {
                        if (gamepad2.left_bumper || flag[preintake]) {
                            preintake();
                        }

                        if (gamepad2.left_stick_x != 0 || gamepad2.right_stick_y != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0 || sampleIntake_Position != currentSampleIntake_Position) {
                            adjustIntake(gamepad2.left_stick_x, gamepad2.right_stick_y, sampleIntake_Position);
                        }
                        if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0){
                            adjustRotate();
                        }
                        if (gamepad2.right_bumper) {
                            grabSample();
                            state = State.LIFT;
                        }
                    }// sample intake end
                    else if (flag[specimen]){

                        if (gamepad2.left_bumper) {
                            preintake_specimen();
                        }
                        if (gamepad2.right_bumper) {
                            grabSpecimen();
                            state = State.OUTTAKE;
                        }
                    }// specimen intake end
                    break;
                case LIFT:

                    if (gamepad1.right_bumper) {
                        sampleLift();
                    }
                    if (extendSlideMotor.getCurrentPosition() > 9000) {
                        state = State.OUTTAKE;
                    }
                    if(gamepad2.left_bumper) {
                        intake_drop();
                        state= State.INTAKE;
                    }
                    break;
                case OUTTAKE:

                    if (!flag[specimen]){

                        if (gamepad2.right_bumper){
                            intakeOuttake(grab_open, hand_perpendicular, arm_sampleHigh, 0);
                            sleep(300);
                            moveDrive(-0.2);
                            sleep(1000);
                            stopDrive();
                            move_extendSlide(extend_postintake_position,6000);
                            sleep(2000);
                            move_rotateSlide(rotateSlide_rest_position,2000);
                            state = State.INTAKE;
                        }
                    } // if sample outtake end
                    else if (flag[specimen]) {
                        if (gamepad2.left_bumper) {
                            intake_drop_specimen();
                            state = State.INTAKE;
                        }
                        if (gamepad2.right_bumper) {
                            specimenScore();
                            intake_drop_specimen();
                            state = State.INTAKE;
                        }
                    } // if specimen outtake end
                    break;
            }// states of robot end

            // hang
            if (gamepad1.dpad_up || flag[hang]) {
                hang();
            }

            // update telemetry or like what you see
            updateTelementry();

        }// op mode active end

    } // run OpMode end

    // ALL THE METHODS FUNCTIONS WHATEVER BELOW

    // FUNCTIONS THAT ARE IN OTHER FUNCTIONS/ HELPER FUNCTIONS I THINK!!!
    public void stopDrive(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void moveDrive(double power){
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }
    public void move_extendSlide(int position, int speed){
        extendSlide_target_position = position;
        extendSlide_target_speed = speed;
        extendSlideMotor.setVelocity(extendSlide_target_speed);
        extendSlideMotor.setTargetPosition(extendSlide_target_position);
    }
    public void move_rotateSlide(int position, int speed){
        rotateSlide_target_position = position;
        rotateSlide_target_speed = speed;
        rotateSlideMotor.setVelocity(rotateSlide_target_speed);
        rotateSlideMotor.setTargetPosition(rotateSlide_target_position);
    }
    public void move_grabServo(double target){
        grab_rotate_target = target;
        grabServo.setPosition(grab_rotate_target);
    }
    public void move_handServo(double target){
        hand_rotate_target = target;
        handServo.setPosition(hand_rotate_target);
    }
    public void move_armServo(double target){
        arm_rotate_target = target;
        armServo.setPosition(arm_rotate_target);
    }
    // FUNCTIONS THAT MOVE ROBOT/ ARE IN MAIN CODE!!!
    boolean timer(double period) {
        // used to reset the start time?
        if (period == 0) {
            st = runtime.milliseconds();
            return false;
        }
        if (runtime.milliseconds() - st > period) return true;
        return false;
    }//timer end

    public boolean timera(double period,int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;
    }

    public void preintake(){
        if (!flag[preintake]){
            move_extendSlide(extend_preintake_position , 3000);
            timera(0 , preintake);
            flag[preintake] = true;
        }
        if (timera(1500, preintake)) {
            move_rotateSlide(rotate_intakeSample, 2000);
            flag[preintake] = false;
        }

        drive_factor = 0.5;
    }

    public void grabSample(){

        moveSlide(array_rotate[sampleIntake_Position] += rotate_adjust_intake, (int) ((extend_intakeSample + extendSlide_intakeSample_Interval * sampleIntake_Position) * array_extend[sampleIntake_Position]), 0);
        move_rotateSlide(array_rotate[sampleIntake_Position] += rotate_adjust_intake , 3000);
        sleep(800);
        intakeOuttake(grab_close, hand_rotate_target, array_arm[sampleIntake_Position], 0);
        sleep(250);
        moveSlide(rotateSlide_rest_position, extend_postintake_position, 0);

    }

    public void sampleLift() {

        intakeOuttake(grab_close, hand_perpendicular, arm_sampleHigh, 0);
        moveSlide(rotate_sampleHigh, extend_sampleHigh, 0);

        drive_factor = 0.5;
    }
    public void intake_drop(){

        intakeOuttake(grab_open, hand_rotate_target, arm_rotate_target, 0);

    }

    public void preintake_specimen(){

        intakeOuttake(grab_open, hand_perpendicular, arm_lowered_SpecimenIntake, 0);
        moveSlide(rotate_intakeSpecimens, extend_intakeSpecimens, 0);
    }

    public void grabSpecimen(){


        move_grabServo(grab_close);
        sleep(500);
        intakeOuttake(grab_close, hand_perpendicular, arm_down, 0);
        moveSlide(rotate_specimenHigh, extend_specimenHigh, 0);
        sleep(800);
        moveDrive(-0.3);
        sleep(600);
        stopDrive();

        drive_factor = 1;
    }
    public void intake_drop_specimen(){

        intakeOuttake(grab_open, hand_perpendicular, arm_lowered_SpecimenIntake, 0);
        moveSlide(rotate_intakeSpecimens, extend_intakeSpecimens, 0);
    }
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

        frontLeftMotor.setPower(frontLeftPower * drive_factor);
        frontRightMotor.setPower(frontRightPower * drive_factor);
        backLeftMotor.setPower(backLeftPower * drive_factor);
        backRightMotor.setPower(backRightPower * drive_factor);
    }// controller drive end
    public void intakeOuttake(double grab, double hand, double arm, int sleepTime){

        grab_rotate_target = grab;
        hand_rotate_target = hand;
        arm_rotate_target = arm;

        grabServo.setPosition(grab_rotate_target);
        sleep(sleepTime);
        handServo.setPosition(hand_rotate_target);
        sleep(sleepTime);
        armServo.setPosition(arm_rotate_target);
        sleep(sleepTime);
    }// intake outtake end
    public void moveSlide(int rotateSlide, int extendSlide, int sleepTime){

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
    public void adjustIntake(double leftX, double rightY, int position) {
        if (!timera(150, adjustment)) return;

        currentSampleIntake_Position = (int) ((extendSlideMotor.getCurrentPosition() - 335)/465 + 0.5);

        if (rightY < -0.3) {
            sampleIntake_Position += 1;
        } else if (rightY > 0.3) {
            sampleIntake_Position -= 1;
        }
        if (sampleIntake_Position < 2) {
            sampleIntake_Position = 2;
        }
        if (sampleIntake_Position > 14) {
            sampleIntake_Position = 14;
        }

        move_handServo(hand_rotate_target += (leftX * 0.1));

        intakeOuttake(grab_rotate_target, hand_rotate_target, array_arm[position], 0);
        moveSlide(rotate_intakeSample, ((extend_intakeSample + extendSlide_intakeSample_Interval * position)), 0);
        timera(0, adjustment);

    }// adjust intake end
    public void adjustRotate(){
        if (!timera(200, adjustmentRotate)) return;

        if (gamepad2.left_trigger > 0.3){
            rotate_adjust_intake += 5;
        }
        else if (gamepad2.right_trigger > 0.3){
            rotate_adjust_intake -= 5;
        }
        if (rotate_adjust_intake > 20){
            rotate_adjust_intake = 20;
        }
        if (rotate_adjust_intake < -40){
            rotate_adjust_intake = -40;
        }

        timera(0, adjustmentRotate);
    }
    public void adjustGrab(){
        if (!timera(200, adjustmentGrab)) return;

        if (gamepad1.left_trigger > 0.3){
            grab_adjust += 0.01;
        }
        else if (gamepad1.right_trigger > 0.3){
            grab_adjust -= 0.01;
        }
        if (rotate_adjust_intake > 0.2){
            grab_adjust = 0.2;
        }
        if (rotate_adjust_intake < -0.3){
            grab_adjust = -0.3;
        }

        timera(0, adjustmentGrab);
    }
    public void specimenScore(){

        extendSlide_target_position = extend_specimenHigh_down;

        move_extendSlide(extendSlide_target_position, 3000);
        sleep(800);
        move_grabServo(grab_open);
        sleep(250);
        moveDrive(-0.3);
        sleep(500);
    }// specimen score end
    public void resetLinearSlide(){
        if (gamepad1.x || gamepad2.x){
            reset_RotateAmount += 10;
        }
        if (gamepad1.y || gamepad2.y){
            reset_RotateAmount -= 10;
        }
        if (reset_RotateAmount > 200){
            reset_RotateAmount = 200;
        }
        if (reset_RotateAmount < -400){
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

        int[] array_rotate    = new int[]   {0, -145 + reset_RotateAmount, -128 + reset_RotateAmount, -111 + reset_RotateAmount, -96 + reset_RotateAmount, -81  + reset_RotateAmount, -74  + reset_RotateAmount, -70  + reset_RotateAmount, -65  + reset_RotateAmount, -60  + reset_RotateAmount, -56  + reset_RotateAmount, -52  + reset_RotateAmount, -48  + reset_RotateAmount, -44  + reset_RotateAmount, -40   + reset_RotateAmount };

        sleep(100);
    }
    public void hang(){
        //steps in team b discord
        if (step == 13) return;

        if (step == 0) {
            //stop motors
            stopDrive();
            drive_factor = 0;
            //move servos so not in the way
            move_grabServo(grab_close);
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
            move_rotateSlide(530 + reset_RotateAmount,2000);
            step = 3;
        }
        if (step == 3 && rotateSlideMotor.getCurrentPosition() > 510 + reset_RotateAmount) {
            //4th
            move_extendSlide(-250,4000);
            move_rotateSlide(1000 + reset_RotateAmount,2000);
            //moveDrive(-0.2);
            step = 4;
        }
        if (step == 4 && extendSlideMotor.getCurrentPosition() < -200) {
            //5th
            //stopDrive();
            sleep(500);
            move_extendSlide(500,5000);
            step = 5;
        }
        if (step == 5 && extendSlideMotor.getCurrentPosition() > 480) {
            //6th
            move_rotateSlide(800 + reset_RotateAmount,2000);
            step = 6;
        }
        if (step == 6 && rotateSlideMotor.getCurrentPosition() > 780 + reset_RotateAmount) {
            //7th
            move_extendSlide(5900,5000);
            step = 7;
        }
        if (step == 7 && extendSlideMotor.getCurrentPosition() > 5880) {
            //8th hit the 2nd bar
            move_rotateSlide(1250 + reset_RotateAmount,2000);
            step = 8;
        }
        if (step == 8 && rotateSlideMotor.getCurrentPosition() > 1230 + reset_RotateAmount) {
            //9th
            move_extendSlide(4600,5000);
            step = 9;
        }
        if (step == 9 && extendSlideMotor.getCurrentPosition() < 4620) {
            //10th
            move_rotateSlide(550 + reset_RotateAmount,2000);
            step = 10;
        }
        if (step == 10 && rotateSlideMotor.getCurrentPosition() < 570 + reset_RotateAmount) {
            //11th
            move_extendSlide(-400,5000);
            step = 11;
        }
        if (step == 11 && extendSlideMotor.getCurrentPosition() < -350) {
            //12th
            sleep(300);
            move_rotateSlide(900 + reset_RotateAmount,2000);
            step = 12;
        }
        if (step == 12 && rotateSlideMotor.getCurrentPosition() > 880 + reset_RotateAmount) {
            //13th
            sleep(1500);
            move_extendSlide(600,5000);
            step = 13;
            flag[hang] = false;
        }
    }// hang end
    public void updateTelementry(){
        telemetry.addData("Elapsed time: ", runtime);
        telemetry.addData("out_Handling: ", out_handling);
        telemetry.addData("start time: ", st);
        telemetry.addData("time period ", timePeriod);
        telemetry.addLine();
        telemetry.addData("Flag Specimens", flag[specimen]);
        telemetry.addData("robot_Stage ", robot_Stage);
        telemetry.addData("Rotate Intake Adjust Intake",rotate_adjust_intake);
        telemetry.addData("Intake Position", sampleIntake_Position);
        telemetry.addData("Current Intake Position", currentSampleIntake_Position);
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
        telemetry.addData("Target of Grab Servo:", grab_rotate_target);
        telemetry.addData("Grab Servo:", grabServo.getPosition());
        telemetry.addLine();
//        telemetry.addData("1 Left Joystick X", gamepad1.left_stick_x);
//        telemetry.addData("1 Left Joystick Y", gamepad1.left_stick_y);
//        telemetry.addLine();
//        telemetry.addData("1 Right Joystick X", gamepad1.right_stick_x);
//        telemetry.addData("1 Right Joystick Y", gamepad1.right_stick_y);
//        telemetry.addLine();
//        telemetry.addData("2 Left Joystick X", gamepad2.left_stick_x);
//        telemetry.addData("2 Left Joystick Y", gamepad2.left_stick_y);
//        telemetry.addLine();
//        telemetry.addData("2 Right Joystick X", gamepad2.right_stick_x);
//        telemetry.addData("2 Right Joystick Y", gamepad2.right_stick_y);
//        telemetry.addLine();
//        telemetry.addData("2 Left Trigger ", gamepad2.left_trigger);
//        telemetry.addData("2 Right Trigger ", gamepad2.right_trigger);
//        telemetry.addLine();
        telemetry.update();
    }//update telementry end

} // class end