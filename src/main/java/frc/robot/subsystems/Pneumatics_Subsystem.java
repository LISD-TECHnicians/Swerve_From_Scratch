package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics_Subsystem extends SubsystemBase {
  private final Solenoid LaBimba = new Solenoid(PneumaticsModuleType.REVPH, 0);

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
