package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PneumaticsConstants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticsSubsystem extends SubsystemBase {
  private final Solenoid laBimba = new Solenoid(PneumaticsModuleType.REVPH, PneumaticsConstants.LA_BIMBA_PORT);

  public PneumaticsSubsystem() {}

  public void toggleSolenoid() {
    laBimba.toggle();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
