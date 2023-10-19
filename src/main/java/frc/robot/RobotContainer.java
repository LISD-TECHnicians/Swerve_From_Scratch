package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Drive.Swerve_Subsystem;
import frc.robot.Constants.Drive_Constants;
import frc.robot.commands.Swerve_Velocity_Cmd;

public class RobotContainer {
  private final CommandPS4Controller Controller = new CommandPS4Controller(0);

  public final Swerve_Subsystem SwerveSubsystem = new Swerve_Subsystem();

  private final Swerve_Velocity_Cmd JoystickSwerve = new Swerve_Velocity_Cmd(
    SwerveSubsystem, 
    () -> Controller.getLeftX() * Drive_Constants.Max_Drive_Speed, 
    () -> Controller.getLeftY() * Drive_Constants.Max_Drive_Speed, 
    () -> Controller.getRightX() * Drive_Constants.Max_Rotation_Speed * Drive_Constants.Rotation_Speed_Scale_Factor);

  public RobotContainer() {
    configureBindings();

    SwerveSubsystem.setDefaultCommand(JoystickSwerve);
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
