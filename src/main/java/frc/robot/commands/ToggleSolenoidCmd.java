package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.PneumaticsSubsystem;

public class ToggleSolenoidCmd extends CommandBase {
  private final PneumaticsSubsystem pneumaticsSubsystem;

  public ToggleSolenoidCmd(PneumaticsSubsystem pneumaticsSubsystem) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;

    addRequirements(pneumaticsSubsystem);
  }

  @Override
  public void initialize() {
    pneumaticsSubsystem.toggleSolenoid();
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
