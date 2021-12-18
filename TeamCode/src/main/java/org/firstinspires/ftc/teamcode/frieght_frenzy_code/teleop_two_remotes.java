package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.noahbres.jotai.State;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop ff", group = "teleop")
public class teleop_two_remotes extends LinearOpMode {

    //imported hardware from "hardwareFF" public class:
    hardwareFF robot = new hardwareFF();

    //this is the timer used to create a toggle switch:
    ElapsedTime toggleBabyTimer = new ElapsedTime();
    ElapsedTime toggleCarousel = new ElapsedTime();
    ElapsedTime wait = new ElapsedTime();

    //this boolean keeps track of whether or not the toggle is on or off
    boolean babyMode = false;
    boolean carouselSpinning = false;
    boolean intakeOn = false;

    State currentState;

    private enum State {
        NOTHING,
        SET,
        WAIT,
        FINISH
    }


    
    public void runOpMode() {
        //initialization code goes here
        robot.init(hardwareMap);
        //robot.svoIntakeTilt.setPosition(0.5);
        currentState = State.NOTHING;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            //all teleop code after start button pressed goes here


            /***
             * GAMEPAD 1 CONTROLS
             */

            //make robot wheels go brrr
            if(gamepad1.left_bumper && !babyMode && toggleBabyTimer.seconds() > var.toggleWait){
                //activate baby slow mode when left bumper is pressed
                babyMode = true;
                toggleBabyTimer.reset();
            }
            if(gamepad1.left_bumper && babyMode && toggleBabyTimer.seconds() > var.toggleWait){
                //deactivate baby slow mode by pressing left bumper again
                babyMode = false;
                toggleBabyTimer.reset();
            }
            
            if (!babyMode) {
                robot.updateDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            if(babyMode){
                robot.updateDrive(gamepad1.left_stick_y*0.5, gamepad1.left_stick_x*0.5, gamepad1.right_stick_x*0.5);
            }

            if(gamepad1.a){
                carouselSpinning = true;
                if(robot.alliance_switch.getState() == true){
                    robot.svoCarousel.setPower(var.fullPower);
                }
                else{
                    robot.svoCarousel.setPower(-var.fullPower);
                }

            }
            if(gamepad1.b){
                carouselSpinning = false;
                robot.svoCarousel.setPower(var.stop);
            }
            if(gamepad1.x){
                robot.svoCarousel.setPower(-var.fullPower);
            }

            /***
             * GAMEPAD 2 CONTROLS
             */

            switch(currentState) {
                case NOTHING:
                    if(gamepad2.dpad_down){
                        robot.svoIntake.setPower(var.lessPower);
                        robot.svoIntakeTilt.setPosition(var.intakeTiltCollect);
                        robot.movearm(0.7,var.groundLvl);
                        currentState = State.SET;
                    }
                    if(gamepad2.dpad_left){
                        robot.svoIntakeTilt.setPosition(var.intakeTiltCollect);
                        robot.movearm(0.7,var.firstLvl);
                        currentState = State.SET;
                    }
                    if(gamepad2.dpad_up){
                        robot.svoIntakeTilt.setPosition(var.intakeTiltCollect);
                        robot.movearm(0.7,var.secondLvl);
                        currentState = State.SET;
                    }
                    if(gamepad2.dpad_right){
                        robot.svoIntakeTilt.setPosition(var.intakeTiltHigh);
                        robot.movearm(0.7,var.thirdLvl);
                        currentState = State.SET;
                    }
                    else if(robot.bottomLimit.isPressed() && gamepad2.left_stick_y > 0){
                        robot.mtrArm.setPower(0);
                    }
                    else if(robot.bottomLimit.isPressed() && gamepad2.left_stick_y < 0){
                        robot.mtrArm.setPower(gamepad2.left_stick_y);
                    }
                    else{
                        robot.mtrArm.setPower(gamepad2.left_stick_y);
                    }

                    break;
                case SET:
                        robot.mtrArm.setPower(0.7);
                        robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        currentState = State.WAIT;
                    break;
                case WAIT:
                    if(robot.mtrArm.isBusy()){

                    }else{
                        currentState = State.FINISH;
                    }
                    break;
                case FINISH:
                    robot.mtrArm.setPower(0);
                    robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currentState = State.NOTHING;
                    break;
            }


            //turret spin to da right
            if(robot.rightLimit.isPressed()){
                robot.mtrTurret.setPower(gamepad2.right_stick_x*0.3);
                telemetry.addLine("REEE");
            }else {
                robot.mtrTurret.setPower(gamepad2.right_stick_x);
            }
            //turret spin to da left
            if(robot.leftLimit.isPressed()){
                robot.mtrTurret.setPower(gamepad2.right_stick_x*0.3);
            }else{
               robot.mtrTurret.setPower(gamepad2.right_stick_x);
            }


            //servo tilt down
            if(gamepad2.left_bumper){
                robot.svoIntakeTilt.setPosition(var.intakeTiltMid);
            }

            //servo tilt up
            if(gamepad2.right_bumper){
                robot.svoIntakeTilt.setPosition(var.intakeTiltCollect);
            }

            /**
             * Intake Controls
             */

            //run intake
            if(gamepad2.a){
                robot.svoIntake.setPower(var.lessPower);
                intakeOn = false;
            }
            //reverse intake
            if(gamepad2.b){
                robot.svoIntake.setPower(-var.lessPower);
                intakeOn = false;
            }
            //stop intake
            if(gamepad2.x){
                robot.svoIntake.setPower(var.stop);
                intakeOn = false;
            }
            if(robot.bottomLimit.isPressed() && !intakeOn){
                intakeOn = true;
                robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.svoIntake.setPower(var.lessPower);
                robot.svoIntakeTilt.setPosition(var.intakeTiltMid);
            }


            //we usually add some telemetry at the end to tell us useful information during testing :)
            if(babyMode){
                telemetry.addLine("baby mode activated");
            }
            else{
                telemetry.addLine("baby mode inactive");
            }


            if(robot.alliance_switch.getState() == true) {
                telemetry.addLine("red alliance");
            }
            else {
                telemetry.addLine("blue alliance");
            }
            if(robot.position_switch.getState() == true) {
                telemetry.addLine("carousel side");
            }
            else {
                telemetry.addLine("warehouse side");
            }

            telemetry.addData("top limit status", robot.topLimit.isPressed());
            telemetry.addData("bottom limit status", robot.bottomLimit.isPressed());
            telemetry.addData("right limit status", robot.rightLimit.isPressed());
            telemetry.addData("left limit status", robot.leftLimit.isPressed());
            telemetry.addData("left stick value: ", gamepad2.left_stick_y);


            telemetry.update();
        }



    }

}
