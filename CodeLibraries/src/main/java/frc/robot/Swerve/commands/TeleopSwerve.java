package frc.robot.Swerve.commands;

import frc.robot.Swerve.constants.Constants;
import frc.robot.Swerve.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier brakeSup;
    private BooleanSupplier fieldCentricSup;
    private BooleanSupplier fencedSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier brakeSup, BooleanSupplier fieldCentricSup, BooleanSupplier fencedSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.brakeSup = brakeSup;
        this.fieldCentricSup = fieldCentricSup;
        this.fencedSup = fencedSup;
    }

    @Override
    public void execute() 
    {
        /* Get Values, Deadband*/
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.ControlConstants.stickDeadband);
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double brakeVal = brakeSup.getAsDouble();
        if (Math.sqrt(Math.pow(translationSup.getAsDouble(), 2) + Math.pow(strafeSup.getAsDouble(), 2)) <= Constants.ControlConstants.stickDeadband) 
        {
            translationVal = 0;
            strafeVal = 0;   
        }

        if (fieldCentricSup.getAsBoolean())
        {
            s_Swerve.driveFenced
            (
                translationVal,
                strafeVal, 
                rotationVal, 
                brakeVal,
                fencedSup.getAsBoolean()
            );
        }
        else
        {
            s_Swerve.drive
            (
                translationVal, 
                strafeVal, 
                rotationVal, 
                brakeVal, 
                fieldCentricSup.getAsBoolean()
            );
        }
    }
}