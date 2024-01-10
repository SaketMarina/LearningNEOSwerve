package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
  private Swerve swerve;
  private double translation;
  private double strafe;
  private double rotation;
  private boolean robotCentric;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public TeleopSwerve(
      Swerve swerve,
      double translation,
      double strafe,
      double rotation, 
      boolean robotCentric) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translation = translation;
    this.strafe = strafe;
    this.rotation = rotation;
    this.robotCentric = robotCentric;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(translation, Constants.SwerveConstants.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafe, Constants.SwerveConstants.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotation, Constants.SwerveConstants.stickDeadband));

    /* Drive */
    swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed),
        rotationVal * Constants.SwerveConstants.maxAngularVelocity,
        !robotCentric,
        true);
  }
}