package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive.Swerve_Subsystem;

public class Reset_Pose_Cmd extends CommandBase {
  private final Swerve_Subsystem SwerveSubsystem;

  public Reset_Pose_Cmd(Swerve_Subsystem SwerveSubsystem) {
    this.SwerveSubsystem = SwerveSubsystem;

    addRequirements(SwerveSubsystem);
  }

  @Override
  public void initialize() {
    SwerveSubsystem.Reset_Pose();
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
