package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

public class var {

    //commonly used static variables can be stored here and used by multiple programs
    //mostly for servo positions that need to be edited once in a while when a builder screws up :}

    //the way you use them in other programs is by typing: var.[variableName]
    //ez pz :)

    //example:
    public static double violet = 0.7545;
    public static double rainbowo = 0.2275;
    public static double confetti = 0.2575;
    public static double ocean = 0.3845;
    public static double green = 0.7145;
    public static double red = 0.6695;

    public static double toggleWait = 1; //(seconds)

    public static double intakeInit = 1;
    public static double intakeCollect = 0.39;
    public static double intakeLow = 0.4;
    public static double intakeMid = 0.5;
    public static double intakeHigh = 0.16;

    public static double fullPower = 1;
    public static double lessPower = 0.3;

    //auto parking encoder counts
    public static double stop = 0;
    public static int parkStrafe = 1200;
    public static int parkBack = -900;

    //teleop encoder counts from ground as 0
    public static int groundLvl = 0;
    public static int firstLvl = 800;
    public static int secondLvl = 2400;
    public static int thirdLvl = 3850;
    public static int thirdLvlTeleOp = 4100;

    //deinit arm
    public static int deinitArm = 400;

    //strafing align
    public static int turnDirection = 1;


}
