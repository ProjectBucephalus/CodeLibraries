package frc.robot.Swerve.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Swerve.util.COTSTalonFXSwerveConstants;
import frc.robot.Swerve.util.SwerveModuleConstants;

import frc.robot.Swerve.util.GeoFenceObject;

public final class Constants 
{
    
    public static final class ControllConstants
    {
        public static final double stickDeadband = 0.1;
        /** Maximum robot speed when accelerated, relative to maximum uncapped speed */
        public static final double speedMax = 0.8;
        /** Normal robot speed, relative to maximum uncapped speed */
        public static final double speedBase = 0.5;
        /** Minimum robot speed when braking, relative to maximum uncapped speed */
        public static final double speedMin = 0.1;
        public static final double speedAngle = 5;
        /** Maximum robot rotation speed */
        public static final double speedRot = 1.3; // 0.02 for angle chasing version
        
        
    }
    
    public final class GeoFencing
    {   
        // Relative to the centre of the robot, in direction the robot is facing
        // These values are the distance in metres to the virtual wall the robot will stop at
        // 0 means the wall is running through the middle of the robot
        // negative distances will have the robot start outside the area, and can only move into it
        /** Metres the robot can travel left */
        public static final double fieldLeft = 0.0;

        /** Metres the robot can travel right */
        public static final double fieldRight = 16.5;

        /** Metres the robot can travel forwards */
        public static final double fieldFront = 8.2;

        /** Metres the robot can travel back */
        public static final double fieldBack = 0;    

        public static final GeoFenceObject field = new GeoFenceObject
        (
            -fieldBack, 
            -fieldRight, 
            fieldBack + fieldFront, 
            fieldLeft + fieldRight, 
            GeoFenceObject.ObjectTypes.walls,
            1.0,
            0.0
        );
        
        public static final GeoFenceObject[] fieldGeoFence = {field};
        
        /** Radius from robot centre in metres where geofence is triggered */
        public static final double robotBuffer = 0.4;
        
        //public static OdometryReadout fieldReadout = new OdometryReadout(0.5);
        //fieldReadout.addRectangle(fieldGeoFence[0].getObject);
    }

    public static final class Swerve
    {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X2_10);
        
        /* Drivetrain Constants */
        public static final double trackWidth = 0.28;
        public static final double wheelBase = 0.28;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        //public static final double angleGearRatioAlt = 12.17; // ERROR: The physical gearing on the specific robot is built wrong, remove this if all swerve modules are built correctly! Should be ~13.37

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 35;
        public static final int angleCurrentThreshold = 60;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.8; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32 / 12; 
        public static final double driveKV = 1.51 / 12;
        public static final double driveKA = 0.27 / 12;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 1; // ERROR: Unsure if this value works as it should
        /** Radians per Second */
        public static final double maxAngularVelocity = 3;
        
        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        // # 5985 Fix: With the additiona of persistant calibration, the angleOffset values should not need to be changed # //
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0
        { 
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1
        { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2
        { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3
        { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(270.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants // TODO: The below constants are used in the example auto, and must be tuned to specific robot
    {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
