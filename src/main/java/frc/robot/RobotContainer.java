package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(1);

  private final Swerve drive = new Swerve();
  //private final AutoOptions autoOptions = new AutoOptions(drive);

  public RobotContainer() {
    drive.setDefaultCommand(new TeleopSwerve(drive, controller.getLeftX(), controller.getLeftY(), controller.getRightX(), true));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    controller.button(1).onTrue(new InstantCommand(() -> drive.zeroGyro()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}