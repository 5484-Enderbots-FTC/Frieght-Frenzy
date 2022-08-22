package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Matt's attempt at teleop moment", group = "teleop")
public class teleop_matt_optimize extends LinearOpMode {

    //imported hardware from "hardwareFF" public class:
    hardwareFF robot = new hardwareFF();

    ElapsedTime runtime = new ElapsedTime();
    //this is the timer used to create a toggle switch:
    ElapsedTime toggleBabyTimer = new ElapsedTime();
    ElapsedTime toggleCarousel = new ElapsedTime();
    ElapsedTime togglePrecisionCap = new ElapsedTime();

    double precisionCap = 1;

    //this boolean keeps track of whether or not the toggle is on or off
    boolean babyMode = false;
    boolean carouselSpinning = false;
    boolean intakeOn = false;
    boolean freightCollected = false;
    boolean tapeDispensed = false;

    boolean zeroPosTapeSet = false;
    boolean zeroPosSet = false;

    State currentState;
    Status intakeState;

    private enum Status {
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
        robot.mtrTape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrTape.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            //all teleop code after start button pressed goes here


            /***
             * GAMEPAD 1 CONTROLS
             */

            //make robot wheels go brrr
            if (gamepad1.left_bumper && !babyMode && toggleBabyTimer.seconds() > vari.toggleWait) {
                //activate baby slow mode when left bumper is pressed
                babyMode = true;
                toggleBabyTimer.reset();
            }
            if (gamepad1.left_bumper && babyMode && toggleBabyTimer.seconds() > vari.toggleWait) {
                //deactivate baby slow mode by pressing left bumper again
                babyMode = false;
                toggleBabyTimer.reset();
            }

            if (!babyMode) {
                robot.updateDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            if (babyMode) {
                robot.updateDrive(gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
            }

            if (gamepad1.a) {
                carouselSpinning = true;
                if (robot.alliance_switch.getState() == true) {
                    robot.svoCarousel.setPower(vari.fullPower);
                } else {
                    robot.svoCarousel.setPower(-vari.fullPower);
                }

            }
            if (gamepad1.b) {
                carouselSpinning = false;
                robot.svoCarousel.setPower(vari.stop);
            }
            if (gamepad1.x) {
                carouselSpinning = true;
                if (robot.alliance_switch.getState() == true) {
                    robot.svoCarousel.setPower(-vari.fullPower);
                } else {
                    robot.svoCarousel.setPower(vari.fullPower);
                }
            }

            /***
             * GAMEPAD 2 CONTROLS
             */

            if (robot.bottomLimit.isPressed() && !zeroPosSet) {
                robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                zeroPosSet = true;
            }

            switch (currentState) {
                case NOTHING:
                    /**
                     * Begin automated to the top of hub function
                     */
                    /*
                    if(gamepad2.left_bumper && zeroPosSet){
                        //third level of hub
                        robot.svoIntakeTilt.setPosition(var.intakeCollect);
                        robot.movearm(0.7,var.groundLvl);
                        currentState = State.SET;
                    }
                    */

                    /*if(gamepad2.right_bumper && zeroPosSet){
                        //third level of hub
                        robot.svoIntakeTilt.setPosition(var.intakeHigh);
                        robot.movearm(1,var.thirdLvl);
                        currentState = State.SET;
                    }
*/
                    /**
                     * Normal 'manual' function :)
                     */
                    //else
                    if (robot.bottomLimit.isPressed() && gamepad2.left_stick_y > 0) {
                        robot.mtrArm.setPower(0);
                    } else if (robot.bottomLimit.isPressed() && gamepad2.left_stick_y < 0) {
                        robot.mtrArm.setPower(gamepad2.left_stick_y / precisionCap);
                    } else {
                        robot.mtrArm.setPower(gamepad2.left_stick_y / precisionCap);
                    }
                    break;
                case SET:
                    robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    currentState = State.WAIT;
                    break;
                case WAIT:
                    if (!robot.mtrArm.isBusy()) {
                        currentState = State.FINISH;
                    }else{
                        currentState = State.WAIT;
                    }
                    break;
                case FINISH:
                    robot.mtrArm.setPower(0);
                    robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currentState = State.NOTHING;
                    break;
            }

            //TODO: rewrite the logic when we get better limits :)

            if (robot.frontLimit.isPressed() && gamepad2.right_stick_x > 0) {
                robot.mtrTurret.setPower(0);
                telemetry.addLine("front limit stopping power");
            } else if (robot.backLimit.isPressed() && gamepad2.right_stick_x < 0) {
                robot.mtrTurret.setPower(0);
                telemetry.addLine("BACK limit stopping power");
            } else {
                robot.mtrTurret.setPower(gamepad2.right_stick_x / precisionCap);
            }
            /**
             * tilt controls
             */

            //TODO: fix collect position if it's too low normally :P
            if (robot.mtrTape.getCurrentPosition() < vari.tapeTimeIsNow) {
                //basically: if not TAPE TIME then do this
                if (robot.mtrArm.getCurrentPosition() >= -vari.armIntakeTiltSwitch) {
                    robot.svoIntakeTilt.setPosition(vari.intakeCollectTeleop);
                }
                if (robot.mtrArm.getCurrentPosition() < -vari.armIntakeTiltSwitch) {
                    robot.svoIntakeTilt.setPosition(vari.intakeHigh);
                }
            } else {
                //otherwise, set intake to init pls
                robot.LEDstrip.setPosition(vari.rainbowo);
                robot.svoIntakeTilt.setPosition(vari.intakeInit);
            }

            /**
             * Intake Controls
             */
            if (!robot.intakeLimit.isPressed()) {
                freightCollected = true;
            } else {
                freightCollected = false;
            }

            if (!freightCollected) {
                if (robot.bottomLimit.isPressed() && intakeState != Status.IN) {
                    robot.svoIntake.setPower(vari.lessPower);
                    intakeState = Status.IN;
                }
                if(runtime.seconds() < 90){
                    robot.LEDstrip.setPosition(vari.green);
                }

            }
            if (freightCollected) {
                if (intakeState != Status.OUT) {
                    robot.svoIntake.setPower(vari.stop);
                    intakeState = Status.STOPPED;
                    if(runtime.seconds() < 90){
                        robot.LEDstrip.setPosition(vari.red);
                    }

                }
            }


            //run intake
            if (gamepad2.a) {
                robot.svoIntake.setPower(vari.lessPower);
                intakeState = Status.IN;
            }
            //reverse intake
            if (gamepad2.b) {
                //might turn this into an output sequence
                intakeState = Status.OUT;
                robot.svoIntake.setPower(-vari.lessPower);
            }
            //stop intake
            if (gamepad2.x) {
                robot.svoIntake.setPower(vari.stop);
                intakeState = Status.STOPPED;
            }

            /**
             * MEASURE NOW
             */

            if (gamepad2.left_bumper && togglePrecisionCap.seconds() > vari.toggleWait) {
                precisionCap += 1;
                if (precisionCap == 4){
                    precisionCap = 1;
                }
                togglePrecisionCap.reset();
            }

            if(robot.mtrTape.getCurrentPosition() > 50){
                tapeDispensed = true;
            }else{
                tapeDispensed = false;
            }

            if(tapeDispensed){
                robot.mtrTape.setPower((gamepad2.right_trigger / precisionCap) - (gamepad2.left_trigger / precisionCap));
            }else{
                robot.mtrTape.setPower((gamepad2.right_trigger / precisionCap));
            }

            //adding trigger deadzones WEEE
            /*
            if(gamepad1.right_trigger > triggerDeadzone){
                robot.mtrTape.setPower(gamepad1.right_trigger);
            }else if(gamepad1.left_trigger > triggerDeadzone){
                robot.mtrTape.setPower(-gamepad1.left_trigger);
            }
                else{
                robot.mtrTape.setPower(0);
            }
             */

            /**
             * Telemetry yay
             */
//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
            //we usually add some telemetry at the end to tell us useful information during testing :)
            if (babyMode) {
                telemetry.addLine("baby mode activated");
            } else {
                telemetry.addLine("baby mode inactive");
            }
/*
            if (robot.alliance_switch.getState() == true) {
                telemetry.addLine("red alliance");
            } else {
                telemetry.addLine("blue alliance");
            }
            if (robot.position_switch.getState() == true) {
                telemetry.addLine("carousel side");
            } else {
                telemetry.addLine("warehouse side");
            }

 */
            telemetry.addData("precision cap reading", precisionCap);
            telemetry.addData("tape dispensed?", tapeDispensed);
            telemetry.addData("tape encoder reading: ", robot.mtrTape.getCurrentPosition());
            telemetry.addData("arm encoder reading: ", robot.mtrArm.getCurrentPosition());
            /*
            telemetry.addData("bottom limit status", robot.bottomLimit.isPressed());
            telemetry.addData("Servo current pos: ", robot.svoIntakeTilt.getPosition());
            telemetry.addData("right limit status", robot.frontLimit.isPressed());
            telemetry.addData("left limit status", robot.backLimit.isPressed());
            telemetry.addData("mid limit status", robot.midLimit.isPressed());
            telemetry.addData("intake limit status", robot.intakeLimit.isPressed());
            telemetry.addData("front range distance: ", "%.2f cm", robot.frontRange.getDistance(DistanceUnit.CM));
            telemetry.addData("back range distance: ", "%.2f cm", robot.backRange.getDistance(DistanceUnit.CM));
             */
            telemetry.update();
        }
    }
}
