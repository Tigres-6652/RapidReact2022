
package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; //librerias
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
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
import frc.robot.Constants.VelocidadChasis;
import frc.robot.Constants.statusrobot;

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
  WPI_VictorSPX MOTORINTAKE = new WPI_VictorSPX(Motores.Intake.KMOTORINTAKE);
  boolean motints = false;

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

  // TEST
  /* String for output */
  StringBuilder _sb = new StringBuilder();

  /* Loop tracker for prints */
  int _loops = 0;

  WPI_TalonFX _talon = new WPI_TalonFX(15);

  /*
   *
   * SEPARACION DE INSTANCIAS
   *
   */

  @Override
  public void robotInit() {
    reiniciarSensores();
    desactivartodo();

  }

  @Override
  public void robotPeriodic() {

    ImprimirEnSmartDashboardRobotPeriodic();

  }

  @Override
  public void autonomousInit() {
    reiniciarSensores();
    desactivartodo();

  }

  @Override
  public void autonomousPeriodic() { // Autonomo
    AutonomoTaxi();
  }

  @Override
  public void teleopInit() {
    reiniciarSensores();
    desactivartodo();

  }

  @Override
  public void teleopPeriodic() { // Teleoperado

    manejarchasis();
    compresorbotonB();
    IntakeBotA();
    cambiosShifter();
    motorintake();

  }

  @Override
  public void disabledInit() {
    reiniciarSensores();
    desactivartodo();

  }

  @Override
  public void disabledPeriodic() {
    desactivartodo();
    reiniciarSensores();
  }

  @Override
  public void testInit() {
    testinitshooterpid();
  }

  @Override
  public void testPeriodic() {
    testTeleopshooterPID();
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
    chasis.arcadeDrive(-VelocidadChasis.velocidadgiro * JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_izquierdo_eje_X),
        -VelocidadChasis.velocidadX * velocidad);

  }

  public void compresorbotonB() {

    // SE PRENDE EL COMPRESOR CON EL BOTON "B"
    // Mas adelante cambiar esto al driver secundario
    if (JoystickDriver1.getRawButtonPressed(ControlarMecanismos.compresor)) {
      if (statusrobot.compresorState) {
        COMPRESOR.enableDigital();
        statusrobot.compresorState = false;
      } else {
        COMPRESOR.disable();
        statusrobot.compresorState = true;
      }
    }
  }

  public void motorintake() {

    // SE PRENDE EL COMPRESOR CON EL BOTON "B"
    // Mas adelante cambiar esto al driver secundario
    if (JoystickDriver1.getRawButtonPressed(Kxbox.BOTONES.X)) {
      if (motints) {
        MOTORINTAKE.set(0.5);
        motints = false;

      } else {
        MOTORINTAKE.set(0);
        motints = true;
      }
    }
  }

  public void IntakeBotA() {

    // ACCIONAMIENTO DE INTAKE CON BOTON "A"

    if (JoystickDriver1.getRawButtonPressed(ControlarMecanismos.intake)) {
      if (statusrobot.IntakeState) {
        PISTINTAKE.set(true);
        // MOTORINTAKE.set(0.4);
        statusrobot.IntakeState = false;
      } else {
        PISTINTAKE.set(false);
        // MOTORINTAKE.set(0);
        statusrobot.IntakeState = true;

      }
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
    COMPRESOR.disable();
    statusrobot.IntakeState = false;
    statusrobot.compresorState = false;

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

    boolean statusSmartcompr;
    statusSmartcompr = !statusrobot.compresorState;
    SmartDashboard.putBoolean("Intake", statusrobot.IntakeState);
    SmartDashboard.putBoolean("Compresor", statusSmartcompr);
    SmartDashboard.putBoolean("prueba compresor", statusrobot.compresorState);

  }

  public void testinitshooterpid() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    _talon.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    _talon.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    /* Config the peak and nominal outputs */
    _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    _talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
    _talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
    _talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
    _talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // _talon.setSensorPhase(true);

  }

  public void testTeleopshooterPID() {
    /* Get gamepad axis */
    double leftYstick = -1 * JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_derecho_eje_Y);

    /* Get Talon/Victor's current output percentage */
    double motorOutput = _talon.getMotorOutputPercent();

    /* Prepare line to print */
    _sb.append("\tout:");
    /* Cast to int to remove decimal places */
    _sb.append((int) (motorOutput * 100));
    _sb.append("%"); // Percent

    _sb.append("\tspd:");
    _sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
    _sb.append("u"); // Native units

    /**
     * When button 1 is held, start and run Velocity Closed loop.
     * Velocity Closed Loop is controlled by joystick position x500 RPM, [-500, 500]
     * RPM
     */
    if (JoystickDriver1.getRawButton(Kxbox.BOTONES.RB)) {
      /* Velocity Closed Loop */

      /**
       * Convert 2000 RPM to units / 100ms.
       * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
       * velocity setpoint is in units/100ms
       */
      double targetVelocity_UnitsPer100ms = leftYstick * 2000.0 * 2048.0 / 600.0;
      /* 2000 RPM in either direction */
      _talon.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

      /* Append more signals to print when in speed mode. */
      _sb.append("\terr:");
      _sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
      _sb.append("\ttrg:");
      _sb.append(targetVelocity_UnitsPer100ms);
    } else {
      /* Percent Output */

      _talon.set(TalonFXControlMode.PercentOutput, leftYstick);
    }

    /* Print built string every 10 loops */
    if (++_loops >= 10) {
      _loops = 0;
      System.out.println(_sb.toString());
    }
    /* Reset built string */
    _sb.setLength(0);

  }
}