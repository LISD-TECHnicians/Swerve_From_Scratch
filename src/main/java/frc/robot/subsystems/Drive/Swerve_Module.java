package frc.robot.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Drive_Constants;

public class Swerve_Module {
  private final WPI_TalonFX Drive_Motor;
  private final CANSparkMax Rotation_Motor;

  private final CANcoder Rotation_Encoder;
  private final Double Angle_Offset;

  private final SlewRateLimiter Drive_Limiter;

  private final PIDController Rotation_PID;

  public Swerve_Module(int Drive_Motor_ID, int Rotation_Motor_ID, int Rotation_Encoder_ID, double Angle_Offset) {
    Drive_Motor = new WPI_TalonFX(Drive_Motor_ID, "rio");
    Rotation_Motor = new CANSparkMax(Rotation_Motor_ID, MotorType.kBrushless);

    Drive_Motor.configFactoryDefault();
    Rotation_Motor.restoreFactoryDefaults();

    Drive_Motor.setVoltage(Drive_Constants.Operating_Voltage);
    Rotation_Motor.setVoltage(Drive_Constants.Operating_Voltage);

    Rotation_Encoder = new CANcoder(Rotation_Encoder_ID, "rio");
    this.Angle_Offset = Angle_Offset;

    Drive_Limiter = new SlewRateLimiter(Drive_Constants.Max_Drive_Set_Acceleration);

    Rotation_PID = new PIDController(Drive_Constants.Rotation_P, Drive_Constants.Rotation_I, Drive_Constants.Rotation_D);
    Rotation_PID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double Get_Drive_Position() {
    return Drive_Motor.getSelectedSensorPosition() / Drive_Constants.Drive_Gear_Ratio; // Returns meters
  }

  public double Get_Drive_Velcoity() {
    return Drive_Motor.getSelectedSensorVelocity() * 10 / Drive_Constants.Drive_Gear_Ratio; // Returns meters per second
  }

  public double Get_Rotation_Position() {
    return Rotation_Encoder.getAbsolutePosition().getValue() / Drive_Constants.Rotation_Gear_Ratio; // Returns radians
  }

  public double Get_Rotation_Velocity() {
    return Rotation_Encoder.getVelocity().getValue() * 10 / Drive_Constants.Rotation_Gear_Ratio; // Returns radians per second
  }

  public SwerveModuleState Get_Swerve_State() {
    return new SwerveModuleState(Get_Drive_Velcoity(), new Rotation2d(Get_Rotation_Position()));
  }

  public void Set_Swerve_State(SwerveModuleState Swerve_Module_State) {
    Swerve_Module_State = SwerveModuleState.optimize(Swerve_Module_State, Get_Swerve_State().angle);

    double Drive_Speed = Drive_Limiter.calculate(Swerve_Module_State.speedMetersPerSecond);
    Drive_Speed = Drive_Speed / Drive_Constants.Max_Drive_Speed;

    double Rotation_Speed = Rotation_PID.calculate(Get_Rotation_Position(), Swerve_Module_State.angle.getRadians() + Angle_Offset);
    Rotation_Speed = MathUtil.clamp(Rotation_Speed, -Drive_Constants.Max_Rotation_Motor_Set_Speed, Drive_Constants.Max_Rotation_Motor_Set_Speed);

    Drive_Motor.set(Drive_Speed);
    Rotation_Motor.set(Rotation_Speed);
  }
}