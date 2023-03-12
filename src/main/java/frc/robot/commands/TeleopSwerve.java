package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private SlewRateLimiter m_strafeValRateLimiter;
    private SlewRateLimiter m_translationValRateLimiter;
    private SlewRateLimiter m_rotationValRateLimiter;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        m_strafeValRateLimiter = new SlewRateLimiter(2.5);
        m_translationValRateLimiter = new SlewRateLimiter(2.6);
        m_rotationValRateLimiter = new SlewRateLimiter(2.5);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        double rotationValSquared =rotationVal > 0 ?rotationVal *rotationVal :rotationVal *rotationVal * -1;
        double strafeValSquared =strafeVal > 0 ?strafeVal *strafeVal :strafeVal *strafeVal * -1;
        double translationValSquared =translationVal > 0 ?translationVal *translationVal :translationVal *translationVal * -1;

        double yAxisFiltered = m_translationValRateLimiter.calculate(translationValSquared);
        double xAxisFiltered = m_strafeValRateLimiter.calculate(strafeValSquared);
        double zAxisFiltered = m_rotationValRateLimiter.calculate(rotationValSquared);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(yAxisFiltered, xAxisFiltered     ).times(Constants.Swerve.maxSpeed), 
            zAxisFiltered * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}