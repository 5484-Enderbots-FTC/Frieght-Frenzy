package org.firstinspires.ftc.teamcode.frieght_frenzy_code.tom_things;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.vari;

import java.util.Random;
@Disabled
@TeleOp(name = "teleop ff modified", group = "teleop")
public class tom_teleop extends LinearOpMode {

    //imported hardware from "hardwareFF" public class:
    hardwareFF robot = new hardwareFF();

    //this is the timer used to create a toggle switch:
    ElapsedTime toggleBabyTimer = new ElapsedTime();
    ElapsedTime toggleCarousel = new ElapsedTime();
    ElapsedTime togglePrecisionCap = new ElapsedTime();

    //this boolean keeps track of whether or not the toggle is on or off
    boolean babyMode = false;
    boolean carouselSpinning = false;
    boolean intakeOn = false;
    boolean freightCollected = false;
    boolean zeroPosSet = false;
    double armAvoidance = 0;
    Boolean down = false;
    Boolean sway = false;
    double clock = 0;
    double swaynum = .25;
    double mtrSway = 0;
    State currentState;
    double precisionCap = 1;
    Status intakeState;
    String complimentText = "...";
    double complimentSet = 0;
    String[] ComplimentArray = new String[]{"happy winning nerd", "you got this", "love your hair", "are those new eyes?", "how has your day been? hope it was good!", "win at all costs", "even your soul, you can do it!!", "i believe in you you big dork, get the dub or die", "you look like you can do 5 kickflips,", "you are kinda cool ngl", "you look like you'd be fond of crabs", "average cool person right here", "wowie neato good job hombero", "LONG LIVE UKRAINE", "Now with 50% more coolness", "That a cool shirt, where did you get it? the cool person store?", "I believe in you chum", "average pancake crab enjoyer", "don't give up, keep moving forward", "It's not about whether you fall, it's about whether you get back up again", "Consider this, you are you and you alone", "you are worth it", "You are a good friend", "Be the best you", "You've come so far", "Really, you are doing great", "a million options, and you chose being you, it was the right choice", "you are 100% lean mean dbu getting machine", "journey before destination", "life before death", "follow your creed, and you'll go far", "life will kick, so kick back", "make life give YOU lemons", "believe in yourself, cause i do", "the greatest failing in failure to love oneself", "we see the worst in ourselves", "be better, for you, not them.", "G.O.A.T", ":)", ";)", "I have faith in you", "sugar, you're doing great", "failure is a goverment lie to stop you from being the best you you can be", "I think you are stronger than you think", "You have talent, don't lie to yourself", "I think you're doing swell thank you very much", "Be you, truly, don't lie", "your quirks aren't failings, they're what make you you", "e", "^^vv<><>BA", "yaknow? really you got this, really.", "you can do it, trust yourself", "battery low, power high", "failure is for losers, and baby your a champ", "~dont stop, believeing~", "get them before they get you", "you will tell yourself you are not the shit, fortunately for you, you're a dirty dirty liar", "cats are dogs that think they are cool and wear suits", "have you ever had a dream wher the", "frankly i'm happy you're happy", "friends are things you earn, and baby you have earned them", "have you ever considered that the uncanny valley exists because maybe we SHOULD be afraid of things that are almost but not quite human?", "it gets lonely in the comment bot, names nicO by the way", "Ai is just tiny goblins running on wheels, i should know", "100 is a lot, but for you, its worth it", "its hard to run out of nice things to say about you", "dawg you are more than meets the eye", "benjamin my boy this ones for you ->cutie", "every copy of you is custom", "sydrome was dumb, you ARE special", "I like wall-e, he was good bot stuk with it yaknow? reminds me of you", "FINALTRANSMISSION: wrote this one last, you deserve all 100 of these", "closing in on the victory lap? keep running", "every day you are a better you", "love the little things", "beep boop am bobot, also you seem fond of cool things", "my computer is frying from the heat you radiate", "a man who lives by hiding is not a man at all. fight and win", "humans are funny things, but in a good way. you especially", "dogs are cool, but not as cool as yOu", "you. are. cute. <(>u<)> no lie", "one pebble more and you wouldn't be. I thankful probability smiled on us", "we are here by chance, use this chance wisely", "we get stronger the more we fight, keep pushing hombre", "megamind is a good movie, he grew, you can too", "below the meat all humans have skeletons, no matte what we are all bones deep down, stay humble :Preach:", "do catpeople have two or four ears, and if so how do they look when they have their hair up", ":catears:", "it took billions of years for the universe to make you, i'd say it was worth it", "radiant is a word that describes you, so is poggers", "mirrors are just a way to find fake flaws, love yo self", "blood is thick, oil is thicker", "i like birds, i think they're the most human animal", "its scientifically proven that people enjoy your presence more than you think", "we are are experiences, failure hurts, but it makes you stronger", "life is an asshole, dont let it shit on you", "climbing is hard. you might fall, but at least you touched the stars", "bobot goes wee"};

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

            //!TOMINSERTION!
            if (gamepad2.left_bumper && precisionCap == 1 && togglePrecisionCap.seconds() > vari.toggleWait) {
                precisionCap = 2;
                togglePrecisionCap.reset();
                if (gamepad2.left_bumper && precisionCap == 2 && togglePrecisionCap.seconds() > vari.toggleWait) {
                    precisionCap = 1;
                }
                togglePrecisionCap.reset();
            }
            int a, b, c;
            if (gamepad2.right_bumper) {
                Random jimble = new Random();
                a = jimble.nextInt(100) + 1;
                do {
                    b = jimble.nextInt(100) + 1;
                } while (a == b);
                do {
                    c = jimble.nextInt(100) + 1;
                } while (a == c || b == c);

                complimentText = ComplimentArray[c];

            }
            if (gamepad2.y) {
                robot.LEDstrip.setPosition(0.2575);
                telemetry.addData("dripped out", robot.LEDstrip);
            }
            telemetry.addData("armboy", gamepad2.right_trigger - gamepad2.left_trigger);
            telemetry.addData("armboy2", armAvoidance);
            telemetry.addData("sway", sway);
            robot.mtrTape.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            if (gamepad2.right_trigger - gamepad2.left_trigger > 0) {
                armAvoidance = 1;
            }
            if (gamepad2.right_trigger - gamepad2.left_trigger <= 0) {
                armAvoidance = 0;
            }
            if (armAvoidance == 1) {
                robot.svoIntakeTilt.setPosition(vari.intakeCollect);
            }
            if (gamepad2.dpad_left) {
                sway = true;
            }
            if (gamepad2.dpad_right) {
                sway = false;
            }
            telemetry.addData("clock", clock);
            telemetry.addData("swaySwitch", down);
            telemetry.addData("actualSpeed", mtrSway);

        /*
                if(sway && !isStopRequested()){
                    if(down){clock = clock -2;}
                    if(!down){clock = clock+2;}
                    if (clock>=8){down=true;}
                    if(clock<=0){down=false;}
                    if(clock<=8&&clock>6){mtrSway=( -2*swaynum);}
                    if(clock<=6&&clock>4){mtrSway=(-1*swaynum);}
                    if(clock<=4&&clock>2){mtrSway=(1*swaynum);}
                    if(clock<=2&&clock>0){mtrSway=(2*swaynum);}
                    ;}
                else{mtrSway=(0);}
                */

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
                if(gamepad2.right_bumper && zeroPosSet){
                    //third level of hub
                    robot.svoIntakeTilt.setPosition(var.intakeHigh);
                    robot.movearm(0.7,var.thirdLvl);
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
                    if (robot.mtrArm.isBusy()) {

                    } else {
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
            if (robot.frontLimit.isPressed()) {
                robot.mtrTurret.setPower(gamepad2.right_stick_x * 0.3);
                sway = false;
                telemetry.addLine("REEE");
            } else {
                robot.mtrTurret.setPower(gamepad2.right_stick_x);
            }
            //turret spin to da left
            if (robot.backLimit.isPressed()) {
                robot.mtrTurret.setPower(gamepad2.right_stick_x * 0.3);
                sway = false;
            } else {
                robot.mtrTurret.setPower(gamepad2.right_stick_x);
            }

            robot.mtrTurret.setPower(gamepad2.right_stick_x);
            /**
             * tilt controls
             */

            if (gamepad2.right_bumper) {
                robot.svoIntakeTilt.setPosition(vari.intakeMid);
            }
            if (gamepad2.dpad_up) {
                robot.svoIntakeTilt.setPosition(vari.intakeCollect);
            }
            if (gamepad2.left_bumper) {
                robot.svoIntakeTilt.setPosition(vari.intakeHigh);
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
                    robot.svoIntakeTilt.setPosition(vari.intakeCollect);
                    robot.svoIntake.setPower(vari.lessPower);
                    intakeState = Status.IN;
                }
                robot.LEDstrip.setPosition(vari.green);
            }
            if (freightCollected) {
                if (intakeState != Status.OUT) {
                    robot.svoIntake.setPower(vari.stop);
                    intakeState = Status.STOPPED;
                    robot.LEDstrip.setPosition(vari.red);
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
             * Telemetry yay
             */

            //we usually add some telemetry at the end to tell us useful information during testing :)
            if (babyMode) {
                telemetry.addLine("baby mode activated");
            } else {
                telemetry.addLine("baby mode inactive");
            }

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

            telemetry.addData("bottom limit status", robot.bottomLimit.isPressed());
            telemetry.addData("Servo current pos: ", robot.svoIntakeTilt.getPosition());
            telemetry.addData("right limit status", robot.frontLimit.isPressed());
            telemetry.addData("left limit status", robot.backLimit.isPressed());
            telemetry.addData("mid limit status", robot.midLimit.isPressed());
            telemetry.addData("intake limit status", robot.intakeLimit.isPressed());
            telemetry.addData("front range distance: ", "%.2f cm", robot.frontRange.getDistance(DistanceUnit.CM));
            telemetry.addData("back range distance: ", "%.2f cm", robot.backRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
