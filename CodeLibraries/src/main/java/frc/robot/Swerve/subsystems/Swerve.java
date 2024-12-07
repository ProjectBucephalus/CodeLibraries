package frc.robot.Swerve.subsystems;

import frc.robot.Swerve.util.SwerveModule;
import frc.robot.Swerve.constants.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public double MaxDriveSpeed = Constants.Swerve.maxSpeed;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    }

    // ------------------------------------------------------------------------------------------ //
    // | # 5985 Additional drive functions to provide more customisable driving functionality # | //
    // | #                                                                                    # | //


    private static double speedBase = Constants.ControlConstants.speedBase;
    private static double speedMax = Constants.ControlConstants.speedMax;
    private static double speedMin = Constants.ControlConstants.speedMin;
    private static double speedAngle = Constants.ControlConstants.speedAngle;
    private static double speedRot = Constants.ControlConstants.speedRot;
    private static double targetAngle = 0;
    private static double robotRadius = Constants.GeoFencing.robotBuffer;
    private boolean manualAngleFlag = false;

    public void setSpeed(double newBaseSpeed)
        {speedBase = newBaseSpeed;}

    public void setSpeedMax(double newMaxSpeed)
        {speedMax = newMaxSpeed;}

    public void setSpeedMin(double newMinSpeed)
        {speedMin = newMinSpeed;}

    public void setSpeedAngle(double newRotationSpeed)
        {speedAngle = newRotationSpeed;}

    public double getTarget()
        {return targetAngle;}

    public void setTarget(double newTargetAngle)
        {targetAngle = newTargetAngle;}

    /** Zero robot headding and reset target angle */
    public void zeroHeading(double targetAngle)
    {
        zeroHeading();
        setTarget(0);
    }
    
    /** Zero robot position */
    public void zeroPose(double targetAngle)
    {
        setPose(new Pose2d(new Translation2d(), getHeading()));
    }

    public void shiftPose(double xShift, double yShift)
    {
        setPose(new Pose2d(new Translation2d(getPose().getX() - xShift, getPose().getY() - yShift), getHeading()));
    }

    public void setPoseX(double xPos)
    {
        setPose(new Pose2d(new Translation2d(xPos, getPose().getY()), getHeading()));
    }

    public void setPoseY(double yPos)
    {
        setPose(new Pose2d(new Translation2d(getPose().getX(), yPos), getHeading()));
    }

    public void setPoseXY(double xPos, double yPos)
    {
        setPose(new Pose2d(new Translation2d(xPos, yPos), getHeading()));
    }

    /**
     * Converts assorted inputs into a tuneable drive profile
     * @param translationVal [-1..1] forward drive axis
     * @param strafeVal [-1..1] sideways drive axis
     * @param targetDelta [-1..1] rotation axis, changes the target angle for the robot to face
     * @param brakeVal [0..1] brake/accelerate axis, modifies the stick input between max and min speeds
     * @param invertBrake BOOLEAN switch brake axis from normal->min to normal->max
     * @param fieldRelative BOOLEAN drive relative to field, false to drive relative to robot front
     * @param fenced BOOLEAN keep robot within defined geofence
     * @author 5985
     */
    public void drive(double translationVal, double strafeVal, double targetDelta, double brakeVal, boolean invertBrake, boolean fieldRelative)
    {
        Translation2d stickInput = new Translation2d(translationVal, strafeVal);
        
        if (targetDelta != 0 && !manualAngleFlag)
        {
            manualAngleFlag = true;
        }

        /*
        targetAngle = ((targetAngle+180) % 360)-180;
        double targetOffset = targetAngle - getHeading().getDegrees();
        if (targetOffset > 180)
        { targetOffset -= 360; }
        else if (targetOffset < -180)
        { targetOffset += 360; }
        
        if (targetDelta == 0 && manualAngleFlag)
        {
            manualAngleFlag = false;
            targetOffset = targetOffset/2;
            targetAngle = targetAngle - targetOffset;
        }
        else
        {
            targetAngle += targetDelta * speedAngle;
        }

        double rotationVal = Math.max(Math.min(targetOffset * speedRot, 1), -1);
        */

        double rotationVal = targetDelta * speedRot;
        targetAngle = getHeading().getDegrees();

        if(brakeVal != 0)
        {
            if(!invertBrake)
            {
                stickInput = stickInput.times(speedBase - ((speedBase - speedMin) * brakeVal));
                //SmartDashboard.putNumber("Throttle:", speedBase - ((speedBase - speedMin) * brakeVal));
                rotationVal *= (speedBase - ((speedBase - speedMin) * brakeVal))/speedBase;
            }
            else
            {
                stickInput = stickInput.times(speedBase + ((speedMax - speedBase) * brakeVal));
                //rotationVal *= (speedBase + ((speedMax - speedBase) * brakeVal))/speedBase;
            }
        }
        else
        {
            stickInput = stickInput.times(speedBase);
        }

        SmartDashboard.putNumber("Stick:", stickInput.getNorm());
        SmartDashboard.putNumber("Headding:", getHeading().getDegrees());
        //SmartDashboard.putNumber("Target", targetAngle);
        //SmartDashboard.putNumber("Delta:", targetDelta);
        //SmartDashboard.putNumber("Rotation:", rotationVal);
        SmartDashboard.putNumber("Brake:", brakeVal);

        drive
        (
            stickInput, 
            rotationVal,
            fieldRelative, 
            true
        );
    }    

    /**
     * Converts assorted inputs into a tuneable drive profile
     * @param translationVal [-1..1] forward drive axis
     * @param strafeVal [-1..1] sideways drive axis
     * @param targetDelta [-1..1] rotation axis, changes the target angle for the robot to face
     * @param brakeVal [0..1] brake/accelerate axis, modifies the stick input between max and min speeds
     * @param invertBrake BOOLEAN switch brake axis from normal->min to normal->max
     * @param fieldRelative BOOLEAN drive relative to field, false to drive relative to robot front
     * @param fenced BOOLEAN keep robot within defined geofence
     * @author 5985
     */
    public void drive(double translationVal, double strafeVal, double targetDelta, double brakeVal, boolean invertBrake, boolean fieldRelative, boolean fenced)
    {
        Translation2d motionXY = new Translation2d(translationVal,strafeVal);
        if (fenced)
        {
            for (int i = 0; i < Constants.GeoFencing.fieldGeoFence.length; i++)
            {
                motionXY = Constants.GeoFencing.fieldGeoFence[i].dampMotion(getPose().getTranslation(), motionXY, robotRadius);
            }
        }
        drive(motionXY.getX(), motionXY.getY(), targetDelta, brakeVal, invertBrake, true);
    }

    /**
     * Converts assorted inputs into a tuneable drive profile
     * @param translationVal [-1..1] forward drive axis
     * @param strafeVal [-1..1] sideways drive axis
     * @param targetDelta [-1..1] rotation axis, changes the target angle for the robot to face
     * @param brakeVal [0..1] brake/accelerate axis, modifies the stick input between max and min speeds
     * @param invertBrake BOOLEAN switch brake axis from normal->min to normal->max
     * @param fieldRelative BOOLEAN drive relative to field, false to drive relative to robot front
     * @param fenced BOOLEAN keep robot within defined geofence
     * @author 5985
     */
    public void drive(double translationVal, double strafeVal, double targetDelta, double brakeVal, boolean invertBrake)
    {
        drive(translationVal, strafeVal, targetDelta, brakeVal, invertBrake, true);
    }

    /**
     * Converts assorted inputs into a tuneable drive profile
     * @param translationVal [-1..1] forward drive axis
     * @param strafeVal [-1..1] sideways drive axis
     * @param targetDelta [-1..1] rotation axis, changes the target angle for the robot to face
     * @param brakeVal [0..1] brake/accelerate axis, modifies the stick input between max and min speeds
     * @param invertBrake BOOLEAN switch brake axis from normal->min to normal->max
     * @param fieldRelative BOOLEAN drive relative to field, false to drive relative to robot front
     * @param fenced BOOLEAN keep robot within defined geofence
     * @author 5985
     */
    public void drive(double translationVal, double strafeVal, double targetDelta, double brakeVal)
    {
        drive(translationVal, strafeVal, targetDelta, brakeVal, false, true);
    }


    // | #                                                                                    # | //
    // | # 5985 Additional drive functions to provide more customisable driving functionality # | //
    // ------------------------------------------------------------------------------------------ //


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
    {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MaxDriveSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MaxDriveSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public static Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        SmartDashboard.putNumber("X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Y", getPose().getTranslation().getY());

        for(SwerveModule mod : mSwerveMods){
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        //MaxDriveSpeed = SmartDashboard.getNumber("Max Drive Speed (m/s)",Constants.Swerve.maxSpeed);
    }
}