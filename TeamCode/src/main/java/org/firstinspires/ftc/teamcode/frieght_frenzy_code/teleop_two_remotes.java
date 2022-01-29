package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "teleop ff", group = "teleop")
public class teleop_two_remotes extends LinearOpMode {

    //imported hardware from "hardwareFF" public class:
    hardwareFF robot = new hardwareFF();

    //this is the timer used to create a toggle switch:
    ElapsedTime toggleBabyTimer = new ElapsedTime();
    ElapsedTime toggleCarousel = new ElapsedTime();

    //this boolean keeps track of whether or not the toggle is on or off
    boolean babyMode = false;
    boolean carouselSpinning = false;
    boolean intakeOn = false;
    boolean freightCollected = false;
    boolean zeroPosSet = false;

    State currentState;
    Status intakeState;

    private enum Status{
        IN,
        OUT,
        STOPPED
    }

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
        intakeState = Status.STOPPED;
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
                carouselSpinning = true;
                if(robot.alliance_switch.getState() == true){
                    robot.svoCarousel.setPower(-var.fullPower);
                }
                else{
                    robot.svoCarousel.setPower(var.fullPower);
                }
            }

            /***
             * GAMEPAD 2 CONTROLS
             */

            if(robot.bottomLimit.isPressed() && !zeroPosSet){
                robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                zeroPosSet = true;
            }

            switch(currentState) {
                case NOTHING:
                    /**
                     * Don't need this 0 position because i just slam the arm down lol
                     */
                    /*
                    if(gamepad2.dpad_down){
                        //basic intake freight position
                        robot.svoIntake.setPower(var.lessPower);
                        robot.svoIntakeTilt.setPosition(var.intakeTiltCollect);
                        robot.movearm(0.7,var.groundLvl);
                        currentState = State.SET;
                    }
                     */
                    /**
                      Don't need these bc why would i ever do these other than auto
                     */
                    /*
                    if(gamepad2.dpad_left){
                        //first level of shipping hub (but why would you ever do this)
                        robot.svoIntakeTilt.setPosition(var.intakeTiltCollect);
                        robot.movearm(0.7,var.firstLvl);
                        currentState = State.SET;
                    }
                    if(gamepad2.dpad_up){
                        //second level of hub
                        robot.svoIntakeTilt.setPosition(var.intakeTiltCollect);
                        robot.movearm(0.7,var.secondLvl);
                        currentState = State.SET;
                    }
                     */
                    /**
                     * Begin automated to the top of hub function
                     */
                    if(gamepad2.dpad_right){
                        //third level of hub
                        robot.svoIntakeTilt.setPosition(var.intakeHigh);
                        robot.movearm(0.7,var.thirdLvl);
                        currentState = State.SET;
                    }
                    //TODO: - change the current dpad control to other available buttons
                    //      - add in shared hub functionality
                    //      - modify the levels to be right
                    //      - fix svoIntakeTilt to be right
                    //      - scrap the rest that's commented out lol

                    /**
                     * Normal 'manual' function :)
                     */
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

            //TODO: rewrite the logic when we get better limits :)

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

            /**
             * Tilt Controls
             */

            //TODO: this ALL is gonna be automated for christs sake this sucks

            /**
             * Intake Controls
             */

            if(!robot.intakeLimit.isPressed()){
                freightCollected = true;
            }else{
                freightCollected = false;
            }

            if(!freightCollected){
                if(robot.bottomLimit.isPressed() && intakeState != Status.IN){
                    robot.svoIntakeTilt.setPosition(var.intakeCollect);
                    robot.svoIntake.setPower(var.lessPower);
                    intakeState = Status.IN;
                }
                robot.LEDstrip.setPosition(var.green);
            }
            if(freightCollected){
                robot.svoIntake.setPower(var.stop);
                intakeState = Status.STOPPED;
                robot.LEDstrip.setPosition(var.red);
            }

            //run intake
            if(gamepad2.a){
                robot.svoIntake.setPower(var.lessPower);
                intakeState = Status.IN;
            }
            //reverse intake
            if(gamepad2.b){
                //might turn this into an output sequence
                robot.svoIntake.setPower(-var.lessPower);
                intakeState = Status.OUT;
            }
            //stop intake
            if(gamepad2.x){
                robot.svoIntake.setPower(var.stop);
                intakeState = Status.STOPPED;
            }

            /**
             * Telemetry yay
             */

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

            //telemetry.addData("top limit status", robot.topLimit.isPressed());
            telemetry.addData("bottom limit status", robot.bottomLimit.isPressed());
            //telemetry.addData("right limit status", robot.rightLimit.isPressed());
            //telemetry.addData("left limit status", robot.leftLimit.isPressed());
            //telemetry.addData("mid limit status", robot.midLimit.isPressed());
            telemetry.addData("intake limit status", robot.intakeLimit.isPressed());
            telemetry.addData("front range distance: ", "%.2f cm", robot.frontRange.getDistance(DistanceUnit.CM));
            telemetry.addData("back range distance: ", "%.2f cm", robot.backRange.getDistance(DistanceUnit.CM));
            telemetry.addData("right distance: ", String.format("%.01f cm", robot.rightDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("left distance: ", String.format("%.01f cm", robot.leftDistance.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}
