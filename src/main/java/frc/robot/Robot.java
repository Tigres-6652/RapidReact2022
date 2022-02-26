
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Motores;
import frc.robot.Constants.Neumatica;
import frc.robot.Constants.statusrobot;

public class Robot extends TimedRobot {

  // Chasis

  WPI_TalonSRX MOTORD1 = new WPI_TalonSRX(Motores.KMOTORD1);
  WPI_TalonSRX MOTORD2 = new WPI_TalonSRX(Motores.KMOTORD2);
  WPI_TalonSRX MOTORD3 = new WPI_TalonSRX(Motores.KMOTORD3);

  WPI_TalonSRX MOTORI4 = new WPI_TalonSRX(Motores.KMOTORI4);
  WPI_TalonSRX MOTORI5 = new WPI_TalonSRX(Motores.KMOTORI5);
  WPI_TalonSRX MOTORI6 = new WPI_TalonSRX(Motores.KMOTORI6);

  MotorControllerGroup MOTSI = new MotorControllerGroup(MOTORD1, MOTORD2, MOTORD3);
  MotorControllerGroup MOTSD = new MotorControllerGroup(MOTORI4, MOTORI5, MOTORI6);

  DifferentialDrive chasis = new DifferentialDrive(MOTSI, MOTSD);

  Solenoid PISTCHASIS = new Solenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTCHASIS);

  // Neumática

  Compressor COMPRESOR = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // CONTROLES

  Joystick JoystickDriver1 = new Joystick(Controllers.kJoystickDriver1);
  // Joystick JoystickDriver2 = new Joystick (Controllers.KJoystickDriver2);

  // INTAKE
  Solenoid PISTINTAKE = new Solenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTINTAKE);

  WPI_TalonSRX MOTORINTAKE = new WPI_TalonSRX(Motores.KMOTORIN);

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {

    // IMPRIME LOS VALORES EN EL SMARTDASHBOARD
    SmartDashboard.putBoolean("Intake", statusrobot.IntakeState);
    SmartDashboard.putBoolean("Compresor", statusrobot.compresorState);

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    // Movimiento del chasis con control Xbox
    statusrobot.velocidad = JoystickDriver1.getRawAxis(3) - JoystickDriver1.getRawAxis(2);
    chasis.arcadeDrive(statusrobot.velocidad, JoystickDriver1.getRawAxis(0));

    // ACCIONAMIENTO DE INTAKE CON BOTON "A"

    if (JoystickDriver1.getRawButton(0)) {
      if (statusrobot.IntakeState = true) {
        PISTINTAKE.set(true);
        MOTORINTAKE.set(0.2);
      } else {
        PISTINTAKE.set(false);
        MOTORINTAKE.set(0);

      }
      statusrobot.IntakeState = !statusrobot.IntakeState;
    }

    // SE PRENDE EL COMPRESOR CON EL BOTON "B"

    // Mas adelante cambiar esto al driver secundario
    if (JoystickDriver1.getRawButton(1)) {
      if (statusrobot.compresorState = true) {
        PISTINTAKE.set(true);
      } else {
        PISTINTAKE.set(false);
      }
      statusrobot.compresorState = !statusrobot.compresorState;
    }

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
