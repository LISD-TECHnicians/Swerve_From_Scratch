package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive.SwerveSubsystem;

public class SetDriveCoastCmd extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;

  public SetDriveCoastCmd(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.setDriveCoast();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
