package frc.robot.Swerve.commands;

import frc.robot.Constants;
import frc.robot.GANG_SHOW_CONSTANTS;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier brakeSup;
    private BooleanSupplier brakeInvertSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier fencedSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier brakeSup, BooleanSupplier brakeInvertSup, BooleanSupplier robotCentricSup, BooleanSupplier fencedSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.brakeSup = brakeSup;
        this.brakeInvertSup = brakeInvertSup;
        this.robotCentricSup = robotCentricSup;
        this.fencedSup = fencedSup;
    }

    @Override
    public void execute() 
    {
        /* Get Values, Deadband*/
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.ControllConstants.stickDeadband);
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double brakeVal = brakeSup.getAsDouble();
        boolean brakeInvert = brakeInvertSup.getAsBoolean();
        if (Math.sqrt(Math.pow(translationSup.getAsDouble(), 2) + Math.pow(strafeSup.getAsDouble(), 2)) <= Constants.ControllConstants.stickDeadband) 
        {
            translationVal = 0;
            strafeVal = 0;   
        }

        s_Swerve.drive
        (
            translationVal,
            strafeVal, 
            rotationVal, 
            brakeVal,
            brakeInvert,
            true,
            !fencedSup.getAsBoolean()
        );
    }
}