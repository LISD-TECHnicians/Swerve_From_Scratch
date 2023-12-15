package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive.Swerve_Subsystem;

import edu.wpi.first.math.geometry.Pose2d;

public class Set_Pose_Cmd extends CommandBase {
  private final Swerve_Subsystem SwerveSubsystem;

  private Pose2d Pose = new Pose2d();

  public Set_Pose_Cmd(Swerve_Subsystem SwerveSubsystem, Pose2d Pose) {
    this.SwerveSubsystem = SwerveSubsystem;

    this.Pose = Pose;

    addRequirements(SwerveSubsystem);
  }

  @Override
  public void initialize() {
    SwerveSubsystem.Set_Pose(Pose);
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
