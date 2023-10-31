package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive.Swerve_Subsystem;

import java.util.function.DoubleSupplier;

public class Swerve_Velocity_Cmd extends CommandBase {
  private final Swerve_Subsystem SwerveSubsystem;

  private final DoubleSupplier X_Speed; // m/s
  private final DoubleSupplier Y_Speed; // m/s
  private final DoubleSupplier Rotation_Speed; // rad/s

  public Swerve_Velocity_Cmd(Swerve_Subsystem SwerveSubsystem, DoubleSupplier X_Speed, DoubleSupplier Y_Speed, DoubleSupplier Rotation_Speed) {
    this.SwerveSubsystem = SwerveSubsystem;
    this.X_Speed = X_Speed;
    this.Y_Speed = Y_Speed;
    this.Rotation_Speed = Rotation_Speed;

    addRequirements(SwerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SwerveSubsystem.Run_Swerve(X_Speed.getAsDouble(), Y_Speed.getAsDouble(), Rotation_Speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
