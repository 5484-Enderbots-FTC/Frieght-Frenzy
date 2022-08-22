package org.firstinspires.ftc.teamcode.frieght_frenzy_code.tom_things;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.vari;

@TeleOp(name = "nek0 teleop ff", group = "teleop")
public class nek0_teleop_two_remotes extends LinearOpMode {

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

    boolean zeroPosTapeSet = false;
    boolean zeroPosSet = false;

    String nek0Neutral1 = "(^-u-^)";
    String nek0Neutral2 = "( ^-u-^)";
    String nek0Neutral3 = "(^-u-^ )";
    String nek0Approving = "(^‾u‾^)";
    String nek0Happy = "(^-U-^)";
    String nek0Awkward = "(^._.^)";
    String nek0Bad = "(^-n-^)";
    String nek0Right = "(͡° ͜ʖ ͡°)";
    String nek0ReallyRight = "(^°U°^)";
    String nek0Suspicous = "(^-_.^)";
    String nek0Dead1 = "(^*-*^)";
    String nek0Dead2 = "(^+-+^)";
    String nek0Readjusting = "(^...^)";
    String nek0Angry1 = "(^◣_◢^)";
    String nek0Angry2 = "(^◣o◢^)";
    String nek0Hurried = "(^°-°^)";
    String nek0OwO = "(^owo^)";
    String nek0UwU = "(^uwu^)";
    String nek0CurrentQuote = "";
    String nek0CurrentFace = "";
    Boolean nek0NeutralState = true;
    Boolean nek0ApprovingState = false;
    Boolean nek0HappyState = false;
    Boolean nek0AwkwardState = false;
    Boolean nek0BadState = false;
    Boolean nek0RightState = false;
    Boolean nek0ReallyRightState = false;
    Boolean nek0SusState = false;
    Boolean nek0DeadState = false;
    Boolean nek0ReadjustingState = false;
    Boolean nek0AngryState = false;
    Boolean nek0HurriedState = false;
    Boolean nek0OwOState = false;
    Boolean nek0UwUState = false;

    Boolean nek0CheerState1=false;
    Boolean nek0CheerStatekill1=false;
    Boolean nek0CheerState2=false;
    Boolean nek0CheerStatekill2=false;
    Boolean nek0CheerState3=false;
    Boolean nek0CheerStatekill3=false;
    double freightCollectionPointValue=0;
    Boolean freightResetValue=false;
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime nek0Time = new ElapsedTime();
    ElapsedTime nek0ResetTime = new ElapsedTime();
    Boolean nek0TimeReset = false;
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
/*everyti

/*
everytime i go to the robot room i feel more and more useless

i hate this

all my projects are just ways to distract myself from my own uselessness

whats the point
 */


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
                    } else {
                        currentState = State.WAIT;
                    }
                    break;
                case FINISH:
                    robot.mtrArm.setPower(0);
                    robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currentState = State.NOTHING;
                    break;
            }
            //this is where all the nek0 stuff goes
            if (nek0ResetTime.seconds()>10&& nek0TimeReset==true) {
                nek0Time.reset(); nek0ResetTime.reset();
            }
            if(nek0Time.seconds() > 8){nek0Time.reset();}
            //below is the idle animation code for the base state, no quotes, lil animation
            if (nek0NeutralState == true && nek0Time.seconds() > 3 && nek0Time.seconds() < 5) {
                nek0CurrentFace = nek0Neutral2;
            } else if (nek0NeutralState == true && nek0Time.seconds() > 5 && nek0Time.seconds() < 8) {
                nek0CurrentFace = nek0Neutral3;
            } else if (nek0NeutralState == true && nek0Time.seconds() < 3 | nek0Time.seconds() > 8) {
                nek0CurrentFace = nek0Neutral1;
            }
            // if you touch these limits nek0 gets a lil angry, and will play a little animation and tell you you to knock it off.
            if (robot.frontLimit.isPressed() | robot.backLimit.isPressed() | robot.bottomLimit.isPressed()) {
                nek0TimeReset=true;
                nek0AngryState = true;
                nek0CurrentQuote = "HEY STOP THAT";
            }
            if (nek0AngryState == true && Math.round(nek0Time.seconds()) % 2 == 0) {
                nek0CurrentFace = nek0Angry1;
            } else if (nek0AngryState == true && Math.round(nek0Time.seconds()) % 2 != 0) {
                nek0CurrentFace = nek0Angry2;
            }
            if (nek0AngryState == true && nek0Time.seconds() > 8 && robot.frontLimit.isPressed() == false && robot.backLimit.isPressed() == false && robot.bottomLimit.isPressed() == false) {
                nek0AngryState = false;
            }
            //below is some logic to switch to and from neutral state
            if (nek0NeutralState = true) {
                
            }
            if (nek0NeutralState = true &&
                    nek0ApprovingState == false &&
                    nek0HappyState == false &&
                    nek0AwkwardState == false &&
                    nek0BadState == false &&
                    nek0RightState == false &&
                    nek0ReallyRightState == false &&
                    nek0SusState == false &&
                    nek0DeadState == false &&
                    nek0ReadjustingState == false &&
                    nek0AngryState == false &&
                    nek0HurriedState == false &&
                    nek0OwOState == false &&
                    nek0UwUState == false) {
                nek0NeutralState = true;
            } else {
                nek0NeutralState = false;
            }
            //below this is the carousel scripting where when the carousel spins it gives congrats

            // IMPORTANT NOTE: while adding pieces, do not declare neutral as true, declare special face as false,
            // neutral sets in the event of no current face
            if(carouselSpinning==true){nek0HappyState=true;nek0CurrentQuote="GOOD JOB FELLAS, GET THIS DUB";nek0TimeReset=true;}
            if(nek0HappyState==true&&nek0Time.seconds()<=4){nek0CurrentFace=nek0Happy;}
            else if(nek0HappyState==true&&nek0Time.seconds()<4){nek0CurrentFace=nek0ReallyRight;}
            if(nek0HappyState=true&&nek0Time.seconds()>8){nek0HappyState=false;}
            //TODO: check if time is measured in dec, and then augment code accordingly
            telemetry.addData("time value for nek0",nek0Time.seconds());
            // a lil note fore i keep goin, the way i normally do face movements is either modular division or set times
            // the below lines of telemetry are where the actual data is stored
            if(freightCollectionPointValue==4&&nek0CheerStatekill1==false){nek0CheerState1=true;}
            if(nek0CheerState1==true){
                nek0CurrentQuote="wow stylin, keep it up";
                if(Math.round(nek0Time.seconds())%2==0){nek0CurrentFace=nek0UwU;}
            else if(Math.round(nek0Time.seconds())%2!=0){nek0CurrentFace=nek0ReallyRight;}

            }

            if(freightCollectionPointValue==6&&nek0CheerStatekill2==false){nek0CheerState2=true;}
            if(freightCollectionPointValue==10&&nek0CheerStatekill3==false){nek0CheerState3=true;}
            telemetry.addData("", nek0CurrentFace);
            telemetry.addData("", nek0CurrentQuote);


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
                if (runtime.seconds() < 90) {
                    robot.LEDstrip.setPosition(vari.green);
                }

            }
            if (freightCollected) {
                if (intakeState != Status.OUT) {
                    robot.svoIntake.setPower(vari.stop);
                    intakeState = Status.STOPPED;
                    if (runtime.seconds() < 90) {
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

                if(freightResetValue=true){
                freightCollectionPointValue+=1;
                freightResetValue=false;}
            }
            else{freightResetValue=true;}
            //stop intake
            if (gamepad2.x) {
                robot.svoIntake.setPower(vari.stop);
                intakeState = Status.STOPPED;
            }

            /**
             * MEASURE NOW
             */

            if (gamepad2.left_bumper && precisionCap == 1 && togglePrecisionCap.seconds() > vari.toggleWait) {
                precisionCap = 2;
                togglePrecisionCap.reset();
            }
            if (gamepad2.left_bumper && precisionCap == 2 && togglePrecisionCap.seconds() > vari.toggleWait) {
                precisionCap = 1;
                togglePrecisionCap.reset();
            }

            robot.mtrTape.setPower((gamepad2.right_trigger / precisionCap) - (gamepad2.left_trigger / precisionCap));

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
