package frc.robot;

/** Values for setting up Gang-Show robot opperation */
public class GANG_SHOW_CONSTANTS 
{
    // ------------------- //
    // > Robot controlls < //
    // ------------------- //
    
    //  //  "Start" (right button on controller face) --> Zero headding
    //  //  "Back"  (left button on controller face) --> Zero position
    //  //  D-pad up/down/left/right --> Shift the virtual fence a small amount in the given direction
    //  //  
    //  //  Right trigger --> Brake
    //  //  Right shoulder button --> Change brake to be accelerate while held
    //  //  Left shoulder button --> Ignore virtual fence
    //  //  
    //  //  Left stick --> Drive
    //  //  Right stick --> Rotate
    
    
    // ----------------------- //
    // > How the robot moves < //
    // ----------------------- //

    /** Normal robot speed, relative to maximum uncapped speed */
    public static final double baseSpeed = 0.5;
    /** Minimum robot speed when braking, relative to maximum uncapped speed */
    public static final double minSpeed = 0.1;
    /** Maximum robot speed when accelerated, relative to maximum uncapped speed */
    public static final double maxSpeed = 0.8;

    /** Maximum robot rotation speed */
    public static final double maxSpin = 1.3;

    /** How fast to twist the wheels to a new direction, increase if changing direction is sluggish */
    public static final double wheelTwistRate = 15;


    // -------------------------- //
    // > Where the robot can go < //
    // -------------------------- //

    // Relative to the centre of the robot, in direction the robot is facing
    // These values are the distance in metres to the virtual wall the robot will stop at
    // 0 means the wall is running through the middle of the robot
    // negative distances will have the robot start outside the area, and can only move into it
    /** Metres the robot can travel left */
    public static final double stageLeft = 0.6;

    /** Metres the robot can travel right */
    public static final double stageRight = 10;

    /** Metres the robot can travel forwards */
    public static final double stageFront = 1.9;

    /** Metres the robot can travel back */
    public static final double stageBack = 2.4;    

    // This value is how close in metres the robot can get to the virtual wall before it starts to slow down when moving towards it
    /** Metres buffer distance to the wall */
    public static final double wallBuffer = 1;

    // This value is how close in metres the centre of the robot can get to the wall before it stops
    /** Metres maximum distance from robot centre to robot perimeter */
    public static final double robotRadius = 0.4;

    // This value is how far the virtual position moves when using the D-Pad
    /** Metres odometry shift distance */
    public static final double  odoShift = 0.25;


    // --------------------------------- //
    // > Position cues and checkpoints < //
    // --------------------------------- //

    // Based on the positioning of the virtual fence
    // When the robot is on a known point, you can press the corresponding button to recentre the positioning

    // (A)
    public static final double xCueA = 0;
    public static final double yCueA = 0;

    // (B)
    public static final double xCueB = 0;
    public static final double yCueB = 0;

    // (X)
    public static final double xCueX = 0;
    public static final double yCueX = 0;

    // (Y)
    public static final double xCueY = 0;
    public static final double yCueY = 0;
}
