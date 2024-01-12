package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drive.SwerveSubsystem;

public class SetDriveBrakeCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;

  public SetDriveBrakeCmd(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    // addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.setDriveBrake();
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
