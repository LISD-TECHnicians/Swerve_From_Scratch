package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Pneumatics_Constants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics_Subsystem extends SubsystemBase {
  private final Solenoid LaBimba = new Solenoid(PneumaticsModuleType.REVPH, Pneumatics_Constants.LaBimba_Port);

  public Pneumatics_Subsystem() {
  }

  public void Toggle_Solenoid() {
    LaBimba.toggle();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
