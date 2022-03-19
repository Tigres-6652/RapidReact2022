
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; //librerias
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControlarMecanismos;
import frc.robot.Constants.Controles;
import frc.robot.Constants.KPIDShooter;
import frc.robot.Constants.Kxbox;
import frc.robot.Constants.Motores;
import frc.robot.Constants.Neumatica;
import frc.robot.Constants.VelocidadChasis;
import frc.robot.Constants.statusrobot;
import frc.robot.Constants.Motores.Shooter;

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
  DoubleSolenoid PISTCHASIS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTCHASIS1,
      Neumatica.KPISTCHASIS2);

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
  WPI_TalonFX MOTORSHOOTERLEFT = new WPI_TalonFX(Motores.Shooter.KMOTORSLeft);
  WPI_TalonFX MOTORSHOOTERRIGHT = new WPI_TalonFX(Motores.Shooter.KMOTORSRight);

  StringBuilder _sb = new StringBuilder(); /* String for output(PID) */

  // INDEXER //
  WPI_VictorSPX MOTORINDEXER = new WPI_VictorSPX(Motores.Indexer.KMOTORINDEXER);

  // CAPUCHA //
  WPI_TalonSRX MOTORCAPUCHA = new WPI_TalonSRX(Motores.Capucha.KMOTORCAPUCHA);
  boolean _lastButton1 = false;
  double targetPositionRotations;

  // CLIMBER //
  WPI_TalonSRX MOTORCLIMBER = new WPI_TalonSRX(Motores.Climber.KMOTORCLIMBER);

  // LIMELIGHT //////
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  double targetOffsetAngle_Vertical = ty.getDouble(0.0);

  // ¿Cuántos grados hacia atrás gira su centro de atención desde la posición
  // perfectamente vertical?
  double limelightMountAngleDegrees = 25.0;

  // distancia desde el centro de la lente Limelight hasta el suelo
  double limelightHeightInches = 20.0;

  // distancia del objetivo al suelo
  double goalHeightInches = 103.9; // distancia hub 103.9 in

  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  // Navx /////
  AHRS navx = new AHRS(SPI.Port.kMXP);

  // ESTRATEGIA AUTOAPUNTADO //
  double[] rpmVal = new double[7];

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

    // Calculos
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)
        / Math.tan(angleToGoalRadians);
    boolean statusSmartcompr;
    double velocidadtest = MOTORSHOOTERLEFT.getSelectedSensorVelocity() / 4096 * 10 * 60 * 2;
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double distancia_metros_limelight_a_hub = distanceFromLimelightToGoalInches * 2.54;

    double capuchavalor = MOTORCAPUCHA.getSelectedSensorPosition();
    double capucha_angulo = capuchavalor / 4096 * 360;
    // IMPRIME LOS VALORES EN EL SMARTDASHBOARD
    SmartDashboard.putNumber("distancia a HUB", distancia_metros_limelight_a_hub);
    SmartDashboard.putBoolean("Intake", !statusrobot.IntakeState);
    SmartDashboard.putBoolean("Compresor", statusrobot.compresorState);
    /*
     * SmartDashboard.putNumber("velocidad", velocidadtest);
     * SmartDashboard.putNumber("LL X Value", x);
     * SmartDashboard.putNumber("LL Y Value", y);
     * SmartDashboard.putNumber("LL X Area", area);
     */

    SmartDashboard.putNumber("capucha angulo", capucha_angulo);
    SmartDashboard.putNumber("capucha", capuchavalor);

  }

  @Override
  public void autonomousInit() {
    reiniciarSensores();
    desactivartodo();

    rpmVal[0] = 600;
    rpmVal[1] = 1200;
    rpmVal[2] = 2500;
    rpmVal[3] = 3500;
    rpmVal[4] = 4300;
    rpmVal[5] = 5000;
    rpmVal[6] = 6500;

  }

  @Override
  public void autonomousPeriodic() { // Autonomo
    AutonomoTaxi();
  }

  @Override
  public void teleopInit() {

    falconpidConfig();
    reiniciarSensores();
    desactivartodo();

  }

  @Override
  public void teleopPeriodic() { // Teleoperado

    double velocidad = JoystickDriver1.getRawAxis(Kxbox.AXES.RT) - JoystickDriver1.getRawAxis(Kxbox.AXES.LT);
    chasis.arcadeDrive(
        -VelocidadChasis.velocidadgiro * JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_izquierdo_eje_X),
        -VelocidadChasis.velocidadX * -velocidad);

    cambiosShifter();

    // Intake
    compresorbotonB();
    IntakeBotA();

    if (JoystickDriver2.getPOV()==Kxbox.POV.derecha) {

ShooterPID(-5600);
    }
    if (JoystickDriver2.getPOV()==Kxbox.POV.abajo) {

      ShooterPID(-4650);
          }
          if (JoystickDriver2.getPOV()==Kxbox.POV.izquierda) {

            ShooterPID(-5900);
                }
                if (JoystickDriver2.getRawAxis(Kxbox.AXES.RT)>=0.5) {

                  MOTORINDEXER.set(0.5);
                } else  if (JoystickDriver2.getRawAxis(Kxbox.AXES.RT)<=0.5 ){
                  MOTORINDEXER.set(0);
}

if(JoystickDriver2.getRawButton(Kxbox.BOTONES.LB)==true){

MOTORCAPUCHA.setSelectedSensorPosition(0);

}

if(JoystickDriver2.getPOV()==-1){

MOTORSHOOTERLEFT.set(0);
MOTORSHOOTERRIGHT.set(0);


}


MOTORCAPUCHA.set(0.4* -JoystickDriver2.getRawAxis(Kxbox.AXES.joystick_derecho_eje_Y));
  
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
  }

  @Override
  public void testPeriodic() {

    double x = tx.getDouble(0.0);
    double ajusteGiro = 0.0f;
    float min_command = 0.05f;

    if (x > 1.0) {

      ajusteGiro = Constants.LimeLight.kp * x - min_command;

    } else if (x < 1.0) {

      ajusteGiro = Constants.LimeLight.kp * x + min_command;

    }
    chasis.arcadeDrive(ajusteGiro, 0);

  }

  /*
   *
   *
   * // SEPARACION DE PERIODOS//
   *
   *
   */

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

  public void IntakeBotA() {

    if (JoystickDriver1.getRawButtonPressed(ControlarMecanismos.intake)) {
      if (statusrobot.IntakeState) {
        PISTINTAKE.set(true);
         MOTORINTAKE.set(-0.4);
        statusrobot.IntakeState = false;
      } else {
        PISTINTAKE.set(false);
         MOTORINTAKE.set(0);
        statusrobot.IntakeState = true;

      }
    }
    }


  public void desactivartodo() {

    // Desactiva totalmente todo, incluso si ya estaba desactivado antes
    chasis.arcadeDrive(0, 0);
    PISTCHASIS.set(Value.kOff);
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
      PISTCHASIS.set(Value.kForward);
    }
    if (JoystickDriver1.getPOV() == ControlarMecanismos.shifter2) {
      PISTCHASIS.set(Value.kReverse);
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

  public void falconpidConfig() { // no moverle a esto por favor👍

    /* Factory Default all hardware to prevent unexpected behaviour */
    MOTORSHOOTERLEFT.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    MOTORSHOOTERLEFT.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    MOTORSHOOTERLEFT.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.KPIDShooter.kPIDLoopIdx,
        Constants.KPIDShooter.kTimeoutMs);

    /* Config the peak and nominal outputs */
    MOTORSHOOTERLEFT.configNominalOutputForward(0, Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.configNominalOutputReverse(0, Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.configPeakOutputForward(1, Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.configPeakOutputReverse(-1, Constants.KPIDShooter.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    MOTORSHOOTERLEFT.config_kF(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kF,
        Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.config_kP(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kP,
        Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.config_kI(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kI,
        Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.config_kD(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kD,
        Constants.KPIDShooter.kTimeoutMs);

        MOTORSHOOTERLEFT.configOpenloopRamp(0.4);
        MOTORSHOOTERRIGHT.configOpenloopRamp(0.4);



    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#

  }

  public void ShooterPID(double rpmtotal) { // 6380 maximo

    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
    double rpmconv = KPIDShooter.torpm * rpmtotal;
    double valor = -1 * rpmconv;// JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_derecho_eje_Y);

    SmartDashboard.putNumber("conv", rpmconv);

    double targetVelocity_UnitsPer100ms = valor * 3000 * 2048.0 / 600.0;
    MOTORSHOOTERLEFT.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
    _sb.setLength(0);
    MOTORSHOOTERRIGHT.set(TalonFXControlMode.Velocity, -targetVelocity_UnitsPer100ms);
  }

  public void ajustedegiro() {

    double x = tx.getDouble(0.0);
    double ajusteGiro = 0.0f;
    float min_command = 0.05f;

    if (x > 1.0) {

      ajusteGiro = Constants.LimeLight.kp * x - min_command;

    } else if (x < 1.0) {

      ajusteGiro = Constants.LimeLight.kp * x + min_command;

    }
    chasis.arcadeDrive(ajusteGiro, 0);

  }

  public void chasis_shoot_Adjust() {
    double x = tx.getDouble(0.0);
    double ajusteGiro = 0.0f;
    float min_aim_command = 0.05f;

    double heading_error = -tx.getDouble(0.0);
    double distance_error = -ty.getDouble(0.0);

    if (x > 1.0) {

      ajusteGiro = Constants.LimeLight.kp * heading_error * x - min_aim_command;

    } else if (x < 1.0) {

      ajusteGiro = Constants.LimeLight.kp * heading_error * x + min_aim_command;

    }

    double distance_adjust = Constants.LimeLight.kp * distance_error;

    chasis.arcadeDrive(ajusteGiro, 0);

  }

  public void capucha(double angulo) {

    double leftYstick = JoystickDriver2.getY();
    boolean button1 = JoystickDriver2.getRawButton(1); // X-Button
    boolean button2 = JoystickDriver2.getRawButton(2); // A-Button

    double motorOutput = MOTORCAPUCHA.getMotorOutputPercent();

    /* Deadband gamepad */
    if (Math.abs(leftYstick) < 0.10) {
      /* Within 10% of zero */
      leftYstick = 0;
    }

    /* Prepare line to print */
    _sb.append("\tout:");
    /* Cast to int to remove decimal places */
    _sb.append((int) (motorOutput * 100));
    _sb.append("%"); // Percent

    _sb.append("\tpos:");
    _sb.append(MOTORCAPUCHA.getSelectedSensorPosition(0));
    _sb.append("u"); // Native units

    /**
     * When button 1 is pressed, perform Position Closed Loop to selected position,
     * indicated by Joystick position x10, [-10, 10] rotations
     */
    if (!_lastButton1 && button1) {
      /* Position Closed Loop */

      /* 10 Rotations * 4096 u/rev in either direction */
      targetPositionRotations = leftYstick * 10.0 * 4096;
      MOTORCAPUCHA.set(ControlMode.Position, targetPositionRotations);
    }

    /* When button 2 is held, just straight drive */
    if (button2) {
      /* Percent Output */

      MOTORCAPUCHA.set(ControlMode.PercentOutput, leftYstick);
    }

    /* If Talon is in position closed-loop, print some more info */
    if (MOTORCAPUCHA.getControlMode() == ControlMode.Position) {
      /* ppend more signals to print when in speed mode. */
      _sb.append("\terr:");
      _sb.append(MOTORCAPUCHA.getClosedLoopError(0));
      _sb.append("u"); // Native Units

      _sb.append("\ttrg:");
      _sb.append(targetPositionRotations);
      _sb.append("u"); /// Native Units
    }

    /**
     * Print every ten loops, printing too much too fast is generally bad
     * for performance.
     */

    /* Reset built string for next loop */
    _sb.setLength(0);

    /* Save button state for on press detect */
    _lastButton1 = button1;

  }

  public void capuchainit() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    MOTORCAPUCHA.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    MOTORCAPUCHA.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        Constants.KPIDCapucha.kPIDLoopIdx,
        Constants.KPIDCapucha.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    MOTORCAPUCHA.setSensorPhase(Constants.KPIDCapucha.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be.
     * This does not affect sensor phase.
     */
    MOTORCAPUCHA.setInverted(Constants.KPIDCapucha.kMotorInvert);

    /* Config the peak and nominal outputs, 12V means full */
    MOTORCAPUCHA.configNominalOutputForward(0, Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.configNominalOutputReverse(0, Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.configPeakOutputForward(1, Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.configPeakOutputReverse(-1, Constants.KPIDCapucha.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range. See Table in Section 17.2.1 for native
     * units per rotation.
     */
    MOTORCAPUCHA.configAllowableClosedloopError(0, Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    MOTORCAPUCHA.config_kF(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains.kF,
        Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.config_kP(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains.kP,
        Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.config_kI(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains.kI,
        Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.config_kD(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains.kD,
        Constants.KPIDCapucha.kTimeoutMs);

    /**
     * Grab the 360 degree position of the MagEncoder's absolute
     * position, and intitally set the relative sensor to match.
     */
    int absolutePosition = MOTORCAPUCHA.getSensorCollection().getPulseWidthPosition();

    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    if (Constants.KPIDCapucha.kSensorPhase) {
      absolutePosition *= -1;
    }
    if (Constants.KPIDCapucha.kMotorInvert) {
      absolutePosition *= -1;
    }

    /* Set the quadrature (relative) sensor to match absolute */
    MOTORCAPUCHA.setSelectedSensorPosition(absolutePosition, Constants.KPIDCapucha.kPIDLoopIdx,
        Constants.KPIDCapucha.kTimeoutMs);
  }

  public void climbler() {
    if (JoystickDriver2.getPOV() == Kxbox.POV.arriba) {

      MOTORCLIMBER.set(0.5);
    }
    if (JoystickDriver2.getPOV() == Kxbox.POV.abajo) {

      MOTORCLIMBER.set(0.5);
    } else if (JoystickDriver2.getPOV() == Kxbox.POV.abajo && JoystickDriver2.getPOV() == Kxbox.POV.arriba) {

    }

  }
}
