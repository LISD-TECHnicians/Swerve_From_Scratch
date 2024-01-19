package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.PneumaticsSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class ToggleSolenoidCmd extends Command {
  private final PneumaticsSubsystem pneumaticsSubsystem;

  private final Timer delayTimer = new Timer();

  public ToggleSolenoidCmd(PneumaticsSubsystem pneumaticsSubsystem) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;

    addRequirements(pneumaticsSubsystem);
  }

  @Override
  public void initialize() {
    delayTimer.reset(); 
    delayTimer.start();
  }

  @Override
  public void execute() {
    if (3 > delayTimer.get() && delayTimer.get() > 2) {
      pneumaticsSubsystem.toggleSolenoid();
    }
  }

  @Override
  public void end(boolean interrupted) {
    delayTimer.reset();
  }

  @Override
  public boolean isFinished() {
    return delayTimer.get() > 8;
  }
}
