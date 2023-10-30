package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics_Subsystem;

public class Toggle_Solenoid_Cmd extends CommandBase {
  private final Pneumatics_Subsystem PneumaticsSubsystem;

  public Toggle_Solenoid_Cmd(Pneumatics_Subsystem PneumaticsSubsystem) {
    this.PneumaticsSubsystem = PneumaticsSubsystem;

    addRequirements(PneumaticsSubsystem);
  }

  @Override
  public void initialize() {
    PneumaticsSubsystem.Toggle_Solenoid();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
