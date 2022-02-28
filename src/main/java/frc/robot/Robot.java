
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; //librerias
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControlarMecanismos;
import frc.robot.Constants.Controles;
import frc.robot.Constants.Kxbox;
import frc.robot.Constants.Motores;
import frc.robot.Constants.Neumatica;
import frc.robot.Constants.statusrobot;
import frc.robot.Constants.VelocidadChasis;

public class Robot extends TimedRobot {

  // CHASIS //

  WPI_TalonSRX MOTORD1ENC = new WPI_TalonSRX(Motores.Chasis.KMOTORD1);
  WPI_TalonSRX MOTORD2 = new WPI_TalonSRX(Motores.Chasis.KMOTORD2);
  WPI_TalonSRX MOTORD3 = new WPI_TalonSRX(Motores.Chasis.KMOTORD3);
  WPI_TalonSRX MOTORI4ENC = new WPI_TalonSRX(Motores.Chasis.KMOTORI4);
  WPI_TalonSRX MOTORI5 = new WPI_TalonSRX(Motores.Chasis.KMOTORI5);
  WPI_TalonSRX MOTORI6 = new WPI_TalonSRX(Motores.Chasis.KMOTORI6);
  MotorControllerGroup MOTSI = new MotorControllerGroup(MOTORD1ENC, MOTORD2, MOTORD3);
  MotorControllerGroup MOTSD = new MotorControllerGroup(MOTORI4ENC, MOTORI5, MOTORI6);
  DifferentialDrive chasis = new DifferentialDrive(MOTSI, MOTSD);
  Solenoid PISTCHASIS = new Solenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTCHASIS);

  // Neumática // (los pistones estan en su respectivo mecanismo)
  Compressor COMPRESOR = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // CONTROLES //
  Joystick JoystickDriver1 = new Joystick(Controles.kJoystickDriver1);
  Joystick JoystickDriver2 = new Joystick(Controles.KJoystickDriver2);

  // INTAKE //
  Solenoid PISTINTAKE = new Solenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTINTAKE);
  WPI_TalonSRX MOTORINTAKE = new WPI_TalonSRX(Motores.Intake.KMOTORINTAKE);

  // SHOOTER //
  WPI_TalonSRX MOTORSHOOTERLEFT = new WPI_TalonSRX(Motores.Shooter.KMOTORSLeft);
  WPI_TalonSRX MOTORSHOOTERRIGHT = new WPI_TalonSRX(Motores.Shooter.KMOTORSRight);

  // INDEXER //
  WPI_TalonSRX MOTORINDEXER = new WPI_TalonSRX(Motores.Indexer.KMOTORINDEXER);

  // CAPUCHA //
  WPI_TalonSRX MOTORCAPUCHA = new WPI_TalonSRX(Motores.Capucha.KMOTORCAPUCHA);

  // CLIMBER //
  WPI_TalonSRX MOTORCLIMBER = new WPI_TalonSRX(Motores.Climber.KMOTORCLIMBER);

  // LIMELIGHT //

  // ENCODERS //

  // Navx //
  AHRS navx = new AHRS(SPI.Port.kMXP);

  /*
   *
   * SEPARACION DE INSTANCIAS
   *
   */

  @Override
  public void robotInit() {
    reiniciarSensores();
  }

  @Override
  public void robotPeriodic() {

    ImprimirEnSmartDashboardRobotPeriodic();

  }

  @Override
  public void autonomousInit() {
    reiniciarSensores();
  }

  @Override
  public void autonomousPeriodic() { // Autonomo
    AutonomoTaxi();
  }

  @Override
  public void teleopInit() {
    reiniciarSensores();
  }

  @Override
  public void teleopPeriodic() { // Teleoperado

    manejarchasis();
    compresorbotonB();
    IntakeBotA();
    cambiosShifter();

  }

  @Override
  public void disabledInit() {
    reiniciarSensores();

  }

  @Override
  public void disabledPeriodic() {
    desactivartodo();
    reiniciarSensores();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {

  }

  /*
   *
   *
   * // SEPARACION DE PERIODOS//
   *
   *
   */

  public void manejarchasis() {

    // Movimiento del chasis con control Xbox
    double velocidad;
    velocidad = JoystickDriver1.getRawAxis(Kxbox.AXES.RB) - JoystickDriver1.getRawAxis(Kxbox.AXES.LB);
    chasis.arcadeDrive(VelocidadChasis.velocidadX * velocidad,
        VelocidadChasis.velocidadgiro * JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_derecho_eje_X));

  }

  public void compresorbotonB() {

    // SE PRENDE EL COMPRESOR CON EL BOTON "B"
    // Mas adelante cambiar esto al driver secundario
    if (JoystickDriver1.getRawButton(ControlarMecanismos.compresor)) {
      if (statusrobot.compresorState = true) {
        PISTINTAKE.set(true);
      } else {
        PISTINTAKE.set(false);
      }
      statusrobot.compresorState = !statusrobot.compresorState;
    }
  }

  public void IntakeBotA() {

    // ACCIONAMIENTO DE INTAKE CON BOTON "A"

    if (JoystickDriver1.getRawButton(ControlarMecanismos.intake)) {
      if (statusrobot.IntakeState = true) {
        PISTINTAKE.set(true);
        MOTORINTAKE.set(0.2);
      } else {
        PISTINTAKE.set(false);
        MOTORINTAKE.set(0);

      }
      statusrobot.IntakeState = !statusrobot.IntakeState;
    }

  }

  public void desactivartodo() {

    // Desactiva totalmente todo, incluso si ya estaba desactivado antes
    chasis.arcadeDrive(0, 0);
    PISTCHASIS.set(false);
    PISTINTAKE.set(false);

  }

  public void reiniciarSensores() {

    // Reset de sensores de encoders, navx.
    MOTORD1ENC.setSelectedSensorPosition(0);
    MOTORI4ENC.setSelectedSensorPosition(0);
    navx.reset();

  }

  public void cambiosShifter() {

    if (JoystickDriver1.getPOV() == ControlarMecanismos.shifter1) {
      PISTCHASIS.set(false);
    }
    if (JoystickDriver1.getPOV() == ControlarMecanismos.shifter2) {
      PISTCHASIS.set(true);
    }
  }

  public void AutonomoTaxi() {

    double encoIzq = MOTORI4ENC.getSelectedSensorPosition();
    SmartDashboard.putNumber("Econder izquierdo", encoIzq);
    double encoDer = MOTORD1ENC.getSelectedSensorPosition();
    double testencodermenos = -1 * encoDer;
    SmartDashboard.putNumber("Econder derecho", testencodermenos);

    // encoizq

    double vuelta = encoIzq / 4096 / 4.17;
    double distanciainches = vuelta * 6.1 * Math.PI; // Units.inchesToMeters(3.2 );
    double distmeters = Units.inchesToMeters(distanciainches);

    SmartDashboard.putNumber("distancia?", distmeters);

    if (distmeters <= 3) {
      chasis.arcadeDrive(0, 0.5);
    }

    if (distmeters >= 3 && distmeters <= 2.05) {
      chasis.arcadeDrive(0, 0);
    }

    if (distmeters >= 3.1) {
      chasis.arcadeDrive(0, -0.3);
    }

    double direccionx = navx.getDisplacementX();
    double direcciony = navx.getDisplacementX();
    double angulo = navx.getAngle();

    SmartDashboard.putNumber("Coordenada x", direccionx);
    SmartDashboard.putNumber("Coordenada y", direcciony);
    SmartDashboard.putNumber("angulo", angulo);

    if (distmeters <= 3 && angulo <= 5) {
      chasis.arcadeDrive(-0.5, 0.7);
    }
    if (distmeters <= 3 && angulo >= 5) {
      chasis.arcadeDrive(0.4, 0.7);
    }

    if (distmeters <= 3 && angulo <= 5 && angulo >= -5) {
      chasis.arcadeDrive(-0, 0.7);
    }

  }

  public void ImprimirEnSmartDashboardRobotPeriodic() {

    // IMPRIME LOS VALORES EN EL SMARTDASHBOARD
    SmartDashboard.putBoolean("Intake", statusrobot.IntakeState);
    SmartDashboard.putBoolean("Compresor", statusrobot.compresorState);

  }

}