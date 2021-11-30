package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.PI;
import static java.lang.Math.round;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.lang.Math;

@Autonomous(name = "HobolidarTestScript", group = "autonomous")

public class hoboLidar extends LinearOpMode {
    //starting with the lidar motor
    DistanceSensor sensorRange;
    DcMotor motorNEHL;
    BNO055IMU imu;
    Orientation angles;

    double side1map;
    double side2map;
    double side3map;
    double side4map;
    //these guy are the initial measurements    double side1turn60 = PI / 3;
    double side2turn120 = (2 * PI) / 3;
    double side2turn135 = (3 * PI) / 4;
    double side2turn150 = (5 * PI) / 6;
    double side3turn210 = (7 * PI) / 6;
    double side3turn225 = (5 * PI) / 4;
    double side3turn240 = (4 * PI) / 3;
    double side4turn300 = (5 * PI) / 3;
    double side4turn315 = (7 * PI) / 4;
    double side4turn330 = (11 * PI) / 6;
    //sideturns are radian values for math is measuring
    double lengthMeasuredSide1turn30;
    double lengthMeasuredSide1turn45;
    double lengthMeasuredSide1turn60;
    double lengthMeasuredSide2turn120;
    double lengthMeasuredSide2turn135;
    double lengthMeasuredSide2turn150;
    double lengthMeasuredSide3turn210;
    double lengthMeasuredSide3turn225;
    double lengthMeasuredSide3turn240;
    double lengthMeasuredSide4turn300;
    double lengthMeasuredSide4turn315;
    double lengthMeasuredSide4turn330;
    //length measured is from origin to wall.
    double side1alengthRaw1 = Math.pow(lengthMeasuredSide1turn45, 2);
    double side1alengthRaw2 = Math.pow(lengthMeasuredSide1turn30, 2);
    double side1alengthCalc = Math.sqrt(side1alengthRaw2 - side1alengthRaw1);

    double side1blengthRaw1 = Math.pow(lengthMeasuredSide1turn45, 2);
    double side1blengthRaw2 = Math.pow(side1map, 2);
    double side1blengthCalc = Math.sqrt(side1blengthRaw2 - side1blengthRaw1);

    double side1clengthRaw1 = Math.pow(lengthMeasuredSide1turn45, 2);
    double side1clengthRaw2 = Math.pow(lengthMeasuredSide4turn330, 2);
    double side1clengthCalc = Math.sqrt(side1clengthRaw2 - side1clengthRaw1);

    double side2alengthRaw1 = Math.pow(lengthMeasuredSide2turn135, 2);
    double side2alengthRaw2 = Math.pow(lengthMeasuredSide2turn120, 2);
    double side2alengthCalc = Math.sqrt(side2alengthRaw2 - side2alengthRaw1);

    double side2blengthRaw1 = Math.pow(lengthMeasuredSide2turn135, 2);
    double side2blengthRaw2 = Math.pow(side2map, 2);
    double side2blengthCalc = Math.sqrt(side2blengthRaw2 - side2blengthRaw1);

    double side2clengthRaw1 = Math.pow(lengthMeasuredSide2turn135, 2);
    double side2clengthRaw2 = Math.pow(lengthMeasuredSide1turn60, 2);
    double side2clengthCalc = Math.sqrt(side2clengthRaw2 - side2clengthRaw1);

    double side3alengthRaw1 = Math.pow(lengthMeasuredSide3turn225, 2);
    double side3alengthRaw2 = Math.pow(lengthMeasuredSide3turn210, 2);
    double side3alengthCalc = Math.sqrt(side3alengthRaw2 - side3alengthRaw1);

    double side3blengthRaw1 = Math.pow(lengthMeasuredSide3turn225, 2);
    double side3blengthRaw2 = Math.pow(side3map, 2);
    double side3blengthCalc = Math.sqrt(side3blengthRaw2 - side3blengthRaw1);

    double side3clengthRaw1 = Math.pow(lengthMeasuredSide3turn225, 2);
    double side3clengthRaw2 = Math.pow(lengthMeasuredSide2turn150, 2);
    double side3clengthCalc = Math.sqrt(side3clengthRaw2 - side3clengthRaw1);

    double side4alengthRaw1 = Math.pow(lengthMeasuredSide4turn315, 2);
    double side4alengthRaw2 = Math.pow(lengthMeasuredSide4turn300, 2);
    double side4alengthCalc = Math.sqrt(side4alengthRaw2 - side4alengthRaw1);

    double side4blengthRaw1 = Math.pow(lengthMeasuredSide4turn315, 2);
    double side4blengthRaw2 = Math.pow(side4map, 2);
    double side4blengthCalc = Math.sqrt(side4blengthRaw2 - side4blengthRaw1);

    double side4clengthRaw1 = Math.pow(lengthMeasuredSide4turn315, 2);
    double side4clengthRaw2 = Math.pow(lengthMeasuredSide3turn240, 2);
    double side4clengthCalc = Math.sqrt(side4clengthRaw2 - side4clengthRaw1);

    // alrightie positioning values time, each side should have 3 checks
    //each, and each one should have an exclusion and recalculating script
//raw 1 is 90 degree angle, straight on. raw 2 is hypotenuse calc is true adjacent
    // if checkvalues are needed, put them here
    double frontCheckValue1aDONTTOUCH = Math.pow(side1alengthCalc, 2);
    double frontCheckValue1a = Math.sqrt(side1alengthRaw2 - frontCheckValue1aDONTTOUCH);

    double frontCheckValue1bDONTTOUCH = Math.pow(side1blengthCalc, 2);
    double frontCheckValue1b = Math.sqrt(side1blengthRaw2 - frontCheckValue1bDONTTOUCH);

    double frontCheckValue1cDONTTOUCH = Math.pow(side1clengthCalc, 2);
    double frontCheckValue1c = Math.sqrt(side1clengthRaw2 - frontCheckValue1cDONTTOUCH);

    double frontCheckValue2aDONTTOUCH = Math.pow(side2alengthCalc, 2);
    double frontCheckValue2a = Math.sqrt(side2alengthRaw2 - frontCheckValue2aDONTTOUCH);

    double frontCheckValue2bDONTTOUCH = Math.pow(side2blengthCalc, 2);
    double frontCheckValue2b = Math.sqrt(side2blengthRaw2 - frontCheckValue2bDONTTOUCH);

    double frontCheckValue2cDONTTOUCH = Math.pow(side2clengthCalc, 2);
    double frontCheckValue2c = Math.sqrt(side2clengthRaw2 - frontCheckValue2cDONTTOUCH);

    double frontCheckValue3aDONTTOUCH = Math.pow(side3alengthCalc, 2);
    double frontCheckValue3a = Math.sqrt(side3alengthRaw2 - frontCheckValue3aDONTTOUCH);

    double frontCheckValue3bDONTTOUCH = Math.pow(side3blengthCalc, 2);
    double frontCheckValue3b = Math.sqrt(side3blengthRaw2 - frontCheckValue3bDONTTOUCH);

    double frontCheckValue3cDONTTOUCH = Math.pow(side3clengthCalc, 2);
    double frontCheckValue3c = Math.sqrt(side3clengthRaw2 - frontCheckValue3cDONTTOUCH);

    double frontCheckValue4aDONTTOUCH = Math.pow(side4alengthCalc, 2);
    double frontCheckValue4a = Math.sqrt(side4alengthRaw2 - frontCheckValue4aDONTTOUCH);

    double frontCheckValue4bDONTTOUCH = Math.pow(side4blengthCalc, 2);
    double frontCheckValue4b = Math.sqrt(side4blengthRaw2 - frontCheckValue4bDONTTOUCH);

    double frontCheckValue4cDONTTOUCH = Math.pow(side4clengthCalc, 2);
    double frontCheckValue4c = Math.sqrt(side4clengthRaw2 - frontCheckValue4cDONTTOUCH);
    //front, or side 1
    double frontlength = 0;
    //front length checks
    double side1aTrue = 0;
    double side1bTrue = 0;
    double side1cTrue = 0;

    double side2aTrue = 0;
    double side2bTrue = 0;
    double side2cTrue = 0;

    double side3aTrue = 0;
    double side3bTrue = 0;
    double side3cTrue = 0;

    double side4aTrue = 0;
    double side4bTrue = 0;
    double side4cTrue = 0;
    //the  double frontLength should end up near side2map, 45 degrees is straight on, and so adapt your stuff as such


    double side1CollectiveTruthFactor = side1aTrue + side1bTrue + side1cTrue;
    double side2CollectiveTruthFactor = side2aTrue + side2bTrue + side2cTrue;
    double side3CollectiveTruthFactor = side3aTrue + side3bTrue + side3cTrue;
    double side4CollectiveTruthFactor = side4aTrue + side4bTrue + side4cTrue;

    double side1TrustPriority = 0;
    double side2TrustPriority = 0;
    double side3TrustPriority = 0;
    double side4TrustPriority = 0;

    //front recalculating script


    //note: these are are not final values, just place holders, because they do not work if approaching a level object


    //for this each if statement above must be seperated, each one should add a poi nt to an accuracy scale which will weigh the order of losses

    //skibbity bop em dada heres some late documentation, 11/20/2021 Tom says hi, I've spaghettified the code, but i kinda get how my own code works again, so PROGRESS, less dumb me nyehehehe
    //front width checks
    //double frontwidth

    void composeTelemetry(){
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int motorGoal=0;
        int angleAdjustMath=round(angles.secondAngle);
        int angleAdjust=angleAdjustMath;
        double eps = 0.01;
        telemetry.log().setCapacity(12);
        telemetry.log().add("");
        telemetry.log().add("Please refer to the calibration instructions");
        telemetry.log().add("contained in the Adafruit IMU calibration");
        telemetry.log().add("sample opmode.");
        telemetry.log().add("");
        telemetry.log().add("When sufficient calibration has been reached,");
        telemetry.log().add("press the 'A' button to write the current");
        telemetry.log().add("calibration data to a file.");
        telemetry.log().add("");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.log().add("Waiting for start...");
        while (!isStarted()) {
            telemetry.update();
            idle();
        }
        telemetry.log().add("...started...");
        while (opModeIsActive()) {

            if (time<5) {

                // Get the calibration data
                BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

                // Save the calibration data to a file. You can choose whatever file
                // name you wish here, but you'll want to indicate the same file name
                // when you initialize the IMU in an opmode in which it is used. If you
                // have more than one IMU on your robot, you'll of course want to use
                // different configuration file names for each.
                String filename = "AdafruitIMUCalibration.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("saved to '%s'", filename);

                // Wait for the button to be released
                while (gamepad1.a) {
                    telemetry.update();
                    idle();
                }
            }

            telemetry.update();
        }

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        motorNEHL = hardwareMap.dcMotor.get("motorNEHL");
        double sensorINCH = sensorRange.getDistance(DistanceUnit.INCH);
        motorNEHL.setDirection(DcMotor.Direction.REVERSE);
        motorNEHL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorNEHL.setTargetPosition(motorGoal);
        motorNEHL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorNEHL.setPower(0.25);
        while (opModeIsActive() && motorNEHL.isBusy())   //motorNEHL.getCurrentPosition() <motorNEHL.getTargetPosition())
        {
            telemetry.addData("encoder-scootscoot-levels", motorNEHL.getCurrentPosition() + "  busy=" + motorNEHL.isBusy());
        }
        //time for a painful amount of work, also note: this is the exact moment i forgot how my own script works, i got mechanicus'd fellas, wish me luck
        //i pray that this works first try because if i have to figure out what i wrote to fix it ill be here for like 3 hours
        //for this, i need the wheel to spin at set amounts and correlate that to the appropiate values, so that's at least 12 new values, probably more.
        double extension = 0.5;
        double multiplier = 50;
        double angle_offset = 0;
        double inverse_angle_offset = -angle_offset;
        if (time >= 0 * multiplier && time <= (0 * multiplier) + extension) {
            motorGoal = 0+angleAdjust;
        } else if (time >= 1 * multiplier && time <= (1 * multiplier) + extension) {
            motorGoal = 30+angleAdjust;
            lengthMeasuredSide1turn45 = sensorINCH;
        } else if (time >= 2 * multiplier && time <= (2 * multiplier) + extension) {
            motorGoal = 45+angleAdjust;
            lengthMeasuredSide1turn60 = sensorINCH;
        } else if (time >= 3 * multiplier && time <= (3 * multiplier) + extension) {
            motorGoal = 60+angleAdjust;
            side2map = sensorINCH;
        }else if (time >= 4 * multiplier && time <= (4 * multiplier) + extension) {
            motorGoal = 90+angleAdjust;
            lengthMeasuredSide2turn120 = sensorINCH;
        } else if (time >= 5 * multiplier && time <= (5 * multiplier) + extension) {
            motorGoal = 120+angleAdjust;
            lengthMeasuredSide2turn135 = sensorINCH;
        } else if (time >= 6 * multiplier && time <= (6 * multiplier) + extension) {
            motorGoal = 135+angleAdjust;
            lengthMeasuredSide2turn150 = sensorINCH;
        } else if (time >= 7 * multiplier && time <= (7 * multiplier) + extension) {
            motorGoal = 150+angleAdjust;
            lengthMeasuredSide3turn210 = sensorINCH;
        }else if (time >= 8 * multiplier && time <= (8 * multiplier) + extension) {
            motorGoal = 180+angleAdjust;
            side3map = sensorINCH;
        } else if (time >= 9 * multiplier && time <= (9 * multiplier) + extension) {
            motorGoal = 210+angleAdjust;
            lengthMeasuredSide3turn225 = sensorINCH;
        } else if (time >= 10 * multiplier && time <= (10 * multiplier) + extension) {
            motorGoal = 225+angleAdjust;
            lengthMeasuredSide3turn240 = sensorINCH;
        } else if (time >= 11 * multiplier && time <= (11 * multiplier) + extension) {
            motorGoal = 240+angleAdjust;
            lengthMeasuredSide4turn300 = sensorINCH;
        } else if (time >= 12 * multiplier && time <= (12 * multiplier) + extension) {
            motorGoal = 300+angleAdjust;
            lengthMeasuredSide4turn315 = sensorINCH;
        } else if (time >= 13 * multiplier && time <= (13 * multiplier) + extension) {
            motorGoal = 315+angleAdjust;
            lengthMeasuredSide4turn330 = sensorINCH;
        } else if (time >= 14 * multiplier && time <= (14 * multiplier) + extension) {
            motorGoal = 330+angleAdjust;
            side1map = sensorINCH;
        } else if (time >= 15 * multiplier && time <= (15 * multiplier) + extension) {
            motorGoal = 0+angleAdjust;
            lengthMeasuredSide1turn30 = sensorINCH;
        } else if (time >= 16 * multiplier && time <= (16 * multiplier) + extension) {
            motorNEHL.setDirection(DcMotor.Direction.FORWARD);
        } else if (time >= 17 * multiplier && time <= (17 * multiplier) + extension) {
            motorGoal = 0+angleAdjust;
            lengthMeasuredSide1turn30=sensorINCH;
        } else if (time >= 18 * multiplier && time <= (18 * multiplier) + extension) {
            motorGoal = 330+angleAdjust;
            side1map=sensorINCH;
        } else if (time >= 19 * multiplier && time <= (19 * multiplier) + extension) {
            motorGoal = 315+angleAdjust;
            lengthMeasuredSide4turn330=sensorINCH;
        } else if (time >= 20 * multiplier && time <= (20 * multiplier) + extension) {
            motorGoal = 300+angleAdjust;
            lengthMeasuredSide4turn315=sensorINCH;
        } else if (time >= 21 * multiplier && time <= (21 * multiplier) + extension) {
            motorGoal = 240+angleAdjust;
            lengthMeasuredSide4turn300=sensorINCH;
        } else if (time >= 22 * multiplier && time <= (22 * multiplier) + extension) {
            motorGoal = 225+angleAdjust;
            lengthMeasuredSide3turn240=sensorINCH;
        } else if (time >= 23 * multiplier && time <= (23 * multiplier) + extension) {
            motorGoal = 210+angleAdjust;
            lengthMeasuredSide3turn225=sensorINCH;
        } else if (time >= 24 * multiplier && time <= (24 * multiplier) + extension) {
            motorGoal = 150+angleAdjust;
            lengthMeasuredSide3turn210=sensorINCH;
        } else if (time >= 25 * multiplier && time <= (25 * multiplier) + extension) {
            motorGoal = 135+angleAdjust;
            lengthMeasuredSide2turn150=sensorINCH;
        } else if (time >= 26 * multiplier && time <= (26 * multiplier) + extension) {
            motorGoal = 120+angleAdjust;
            lengthMeasuredSide2turn135=sensorINCH;
        } else if (time >= 27 * multiplier && time <= (27 * multiplier) + extension) {
            motorGoal = 60+angleAdjust;
            lengthMeasuredSide2turn120=sensorINCH;
        } else if (time >= 28 * multiplier && time <= (28 * multiplier) + extension) {
            motorGoal = 45+angleAdjust;
            lengthMeasuredSide1turn60=sensorINCH;
        } else if (time >= 29 * multiplier && time <= (29 * multiplier) + extension) {
            motorGoal = 30+angleAdjust;
            lengthMeasuredSide1turn45=sensorINCH;
        } else if (time >= 30 * multiplier && time <= (30 * multiplier) + extension) {
            motorGoal = 0+angleAdjust;
            lengthMeasuredSide1turn30=sensorINCH;
        } else if (time >= 31 * multiplier && time <= (31 * multiplier) + extension) {
            motorNEHL.setDirection(DcMotor.Direction.REVERSE);
            side1map=sensorINCH;
            time = 0;
        }
        if (Math.abs(frontCheckValue1a - lengthMeasuredSide1turn45) < eps) {
            // eps is accuracy value
            side1aTrue = 1;
        } else {
            side1aTrue = 0;
        }
        if (Math.abs(frontCheckValue1b - lengthMeasuredSide1turn45) < eps) {
            // eps is accuracy value
            side1bTrue = 1;
        } else {
            side1bTrue = 0;
        }
        if (Math.abs(frontCheckValue1c - lengthMeasuredSide1turn45) < eps) {
            // eps is accuracy value
            side1cTrue = 1;
        } else {
            side1cTrue = 0;
        }
        if (Math.abs(frontCheckValue2a - lengthMeasuredSide2turn135) < eps) {
            // eps is accuracy value
            side2aTrue = 1;
        } else {
            side2aTrue = 0;
        }
        if (Math.abs(frontCheckValue2b - lengthMeasuredSide2turn135) < eps) {
            // eps is accuracy value
            side2bTrue = 1;
        } else {
            side2bTrue = 0;
        }
        if (Math.abs(frontCheckValue2c - lengthMeasuredSide2turn135) < eps) {
            // eps is accuracy value
            side2cTrue = 1;
        } else {
            side2cTrue = 0;
        }
        if (Math.abs(frontCheckValue3a - lengthMeasuredSide3turn225) < eps) {
            // eps is accuracy value
            side3aTrue = 1;
        } else {
            side3aTrue = 0;
        }
        if (Math.abs(frontCheckValue3b - lengthMeasuredSide3turn225) < eps) {
            // eps is accuracy value
            side3bTrue = 1;
        } else {
            side3bTrue = 0;
        }
        if (Math.abs(frontCheckValue3c - lengthMeasuredSide3turn225) < eps) {
            // eps is accuracy value
            side3cTrue = 1;
        } else {
            side3cTrue = 0;
        }
        if (Math.abs(frontCheckValue4a - lengthMeasuredSide4turn315) < eps) {
            // eps is accuracy value
            side4aTrue = 1;
        } else {
            side4aTrue = 0;
        }
        if (Math.abs(frontCheckValue4b - lengthMeasuredSide4turn315) < eps) {
            // eps is accuracy value
            side4bTrue = 1;
        } else {
            side4bTrue = 0;
        }
        if (Math.abs(frontCheckValue4c - lengthMeasuredSide4turn315) < eps) {
            // eps is accuracy value
            side4cTrue = 1;
        } else {
            side4cTrue = 0;
        }
        double priorityTest1 = Math.max(side1CollectiveTruthFactor, side2CollectiveTruthFactor);
        double priorityTest2 = Math.max(side3CollectiveTruthFactor, side4CollectiveTruthFactor);
        double highPriority = Math.max(priorityTest1, priorityTest2);

        double priorityTest3 = Math.min(side1CollectiveTruthFactor, side2CollectiveTruthFactor);
        double priorityTest4 = Math.min(side3CollectiveTruthFactor, side4CollectiveTruthFactor);
        double lowPriority = Math.min(priorityTest3, priorityTest4);

        boolean side1Untrustworthy = false;
        boolean side2Untrustworthy = false;
        boolean side3Untrustworthy = false;
        boolean side4Untrustworthy = false;
        boolean uberTrust1 = false;
        boolean uberTrust2 = false;
        boolean uberTrust3 = false;
        boolean uberTrust4 = false;
        boolean lidarDead = false;
        boolean uberCharge = true;
        if (side1CollectiveTruthFactor >= 4 || side2CollectiveTruthFactor == 4 || side3CollectiveTruthFactor == 4 || side4CollectiveTruthFactor == 4) {
            if (side1CollectiveTruthFactor == highPriority) {
                uberTrust1 = true;
            }
            if (side2CollectiveTruthFactor == highPriority) {
                uberTrust2 = true;
            }
            if (side3CollectiveTruthFactor == highPriority) {
                uberTrust3 = true;
            }
            if (side4CollectiveTruthFactor == highPriority) {
                uberTrust4 = true;
            }
            if (uberTrust1 == true && uberTrust2 == true && uberTrust3 == true && uberTrust4) {
                uberCharge = true;
            }
        }
        if (side1CollectiveTruthFactor <= 1 || side2CollectiveTruthFactor <= 1 || side3CollectiveTruthFactor <= 1 || side4CollectiveTruthFactor <= 1) {
            if (side1CollectiveTruthFactor == lowPriority) {
                side1Untrustworthy = true;
            }
            if (side2CollectiveTruthFactor == lowPriority) {
                side2Untrustworthy = true;
            }
            if (side3CollectiveTruthFactor == lowPriority) {
                side3Untrustworthy = true;
            }
            if (side4CollectiveTruthFactor == lowPriority) {
                side4Untrustworthy = true;
            }
            if (side1Untrustworthy == true && side2Untrustworthy == true && side3Untrustworthy == true && side4Untrustworthy == true) {
                lidarDead = true;
            }

        }
        //field position measurements after all the checks have run, its time to get you positioning, note, the uncertaintaity value will increase with time
        // each turn will slowly push that value up, which will adjust EPS to account for it, after two minutes it should still be fine, but we'll see
        //these scripts will weight checker values against one another, and if they add to what others checkers do, EPS stays low, EPS will have a hard cap,
        //which is currently unknown, which will kill the lidar, There should be six values that should have atleast 3 sharing the same value before EPS shoots up
        double fullLength1 = lengthMeasuredSide1turn45 + lengthMeasuredSide3turn225;
        double fullWidth1 = lengthMeasuredSide2turn135 + lengthMeasuredSide4turn315;
        double fullLength2 = frontCheckValue1a + frontCheckValue3a;
        double fullWidth2 = frontCheckValue2a + frontCheckValue4a;
        double fullLength3 = frontCheckValue1b + frontCheckValue3b;
        double fullWidth3 = frontCheckValue2b + frontCheckValue4b;
        double trueFullLength;
        double trueFullWidth;
        boolean set1x = false;
        boolean set2x = false;
        boolean set3x = false;
        boolean set1y = false;
        boolean set2y = false;
        boolean set3y = false;
        if (Math.abs(fullLength1 - fullLength2) < eps && fullLength1 != fullLength3) {
            trueFullLength = 1;
            set1x = true;
        }
        if (Math.abs(fullLength2 - fullLength3) < eps && fullLength1 != fullLength3) {
            trueFullLength = 2;
            set1x = true;
        }
        if (Math.abs(fullLength3 - fullLength1) < eps && fullLength2 != fullLength3) {
            trueFullLength = 3;
            set1x = true;
        }
        double currentPositionx1 = fullLength1 - fullLength2;
        double currentPositionx2 = fullLength2 - fullLength3;
        double currentPositionx3 = fullLength3 - fullLength1;
        if (Math.abs(fullWidth1 - fullWidth2) < eps && fullWidth1 != fullWidth3) {
            trueFullWidth = 1;
            set1y = true;
        } else if (Math.abs(fullWidth2 - fullWidth3) < eps && fullWidth1 != fullWidth3) {
            trueFullWidth = 2;
            set1y = true;
        } else if (Math.abs(fullWidth3 - fullWidth1) < eps && fullWidth2 != fullWidth3) {
            trueFullWidth = 3;
            set1y = true;
        }
        double currentPositiony1 = fullWidth1 - fullWidth2;
        double currentPositiony2 = fullWidth2 - fullWidth3;
        double currentPositiony3 = fullWidth3 - fullWidth1;
        if (set1x == true) {
            trueFullLength = fullLength1 - fullLength2;
        } else if (set2x == true) {
            trueFullLength = fullLength2 - fullLength3;
        } else if (set2x == true) {
            trueFullLength = fullLength3 - fullLength1;
        }

        telemetry.update();

    }
}
