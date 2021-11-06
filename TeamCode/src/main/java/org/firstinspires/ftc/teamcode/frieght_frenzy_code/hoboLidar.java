package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.lang.Math;


public class hoboLidar {
    double side1map;
    double side2map;
    double side3map;
    double side4map;
    //these guy are the initial measurements
    double side1turn60 = PI / 3;
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
    double lengthMeasuredSide1turn30 = 5;
    double lengthMeasuredSide1turn45 = 6;
    double lengthMeasuredSide1turn60 = 5;
    double lengthMeasuredSide2turn120 = 5;
    double lengthMeasuredSide2turn135 = 6;
    double lengthMeasuredSide2turn150 = 5;
    double lengthMeasuredSide3turn210 = 5;
    double lengthMeasuredSide3turn225 = 6;
    double lengthMeasuredSide3turn240 = 5;
    double lengthMeasuredSide4turn300 = 5;
    double lengthMeasuredSide4turn315 = 6;
    double lengthMeasuredSide4turn330 = 5;
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

    double eps = 0.00001;
    double fullLength=lengthMeasuredSide1turn45+lengthMeasuredSide3turn225;
    double fullWidth=lengthMeasuredSide2turn135+lengthMeasuredSide4turn315;
    //note: these are are not final values, just place holders, because they do not work if approaching a level object


    //for this each if statement above must be seperated, each one should add a point to an accuracy scale which will weigh the order of losses

    //front width checks
    //double frontwidth
    public void init() {
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
        if (side1CollectiveTruthFactor == 4 || side2CollectiveTruthFactor == 4 || side3CollectiveTruthFactor == 4 || side4CollectiveTruthFactor == 4) {
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
                if (side1Untrustworthy == false && side2Untrustworthy == false && side3Untrustworthy == false && side4Untrustworthy == false) {
                    lidarDead = true;
                }



            }
        }
    }

