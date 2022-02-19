// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //Auto Selector
  private static final String kDefaultAuto = "Default";
  private static final String kLowAuto = "Low Auto";
  private static final String kHighAuto = "High Auto";
  private static final String kHighAutoPickup= "High Auto Pickup";
  private static final String kCenterHighAuto = "TCup Center High Auto";
  private static final String kCenterLowAuto = "TCup Center Low Auto";
  private static final String kLRHighAuto = "TCup L/R High Auto"; 
  private static final String kCenterHighW5Auto = "TCup Center High W5 Auto";

  private String m_autoSelected;
  
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  //Motors
  private final WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(RobotMap.m_leftMotor1);
  private final WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(RobotMap.m_leftMotor2);
  private final WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(RobotMap.m_rightMotor1);
  private final WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(RobotMap.m_rightMotor2);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

  private final DifferentialDrive m_driveTrain = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final WPI_TalonFX firstMotor = new WPI_TalonFX(RobotMap.m_leftShooter);
  private final WPI_TalonFX secondMotor = new WPI_TalonFX(RobotMap.m_rightShooter);

  private final WPI_VictorSPX armMotor = new WPI_VictorSPX(RobotMap.m_arm);
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(RobotMap.m_intake);

  private final WPI_VictorSPX panelMotor = new WPI_VictorSPX(RobotMap.m_panelMotor);
  private final WPI_VictorSPX spinnerMotor = new WPI_VictorSPX(RobotMap.m_spinnerMotor);

  private final DifferentialDrive m_shooter = new DifferentialDrive(firstMotor, secondMotor);
  private final WPI_VictorSPX m_transition = new WPI_VictorSPX(RobotMap.m_transition);

  private final WPI_VictorSPX climbMotor = new WPI_VictorSPX(RobotMap.m_climb);
  //private final WPI_TalonFX winchMotor = new WPI_TalonFX(RobotMap.m_winch);

 // double shooterSpeed1 = firstMotor.getSelectedSensorVelocity();
  //double shooterSpeed2 = secondMotor.getSelectedSensorVelocity();

  private final XboxController m_driverController = new XboxController(RobotMap.DRIVER_CONTROLLER);
  private final XboxController m_operatorController = new XboxController(RobotMap.OPERATOR_CONTROLLER);

  private final Timer m_timer = new Timer();

  private final Encoder leftEncoders = new Encoder(0, 1, false);
  private final Encoder rightEncoders = new Encoder (6,7,true);

  private final DigitalInput armSwitch = new DigitalInput(RobotMap.m_armSwitch);
  private final DigitalInput armDownSwitch = new DigitalInput(RobotMap.m_armSwitch2);

  private double m_voltage;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Low Auto", kLowAuto);
    m_chooser.addOption("High Auto", kHighAuto);
    m_chooser.addOption("High Auto Pickup", kHighAutoPickup);
    m_chooser.addOption("TCup Center High Auto", kCenterHighAuto);
    m_chooser.addOption("TCup Center High W5 Auto", kCenterHighW5Auto);
    m_chooser.addOption("TCup Center Low Auto", kCenterLowAuto);
    m_chooser.addOption("TCup L/R High Auto", kLRHighAuto);
    SmartDashboard.putData("Auto choices", m_chooser);


   CameraServer.startAutomaticCapture();
   //CameraServer.startAutomaticCapture("Climb Cam", 1).setResolution(80, 40);

   
    leftEncoders.reset();
    rightEncoders.reset();
   
    m_rightMotors.setInverted(true);
    secondMotor.setInverted(true);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").setNumber(3);


    m_timer.reset();

    m_timer.start();

    leftEncoders.reset();
    rightEncoders.reset();

    //m_voltage = m_PDP.getVoltage();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      /*********************************************************************************** */
      //Low goal auto, may be set in several places on initiation line and aimed towards goal
      /*********************************************************************************** */
            case kLowAuto:
              if(m_timer.get() < 4.0){
              if(leftEncoders.getDistance() < 1000){
                m_driveTrain.arcadeDrive(0.5, 0);
                
              } else{
                m_driveTrain.stopMotor();}
              } else if(m_timer.get() < 4.3 && m_timer.get() > 4.0){
                armMotor.set(0.3);
              } else if(m_timer.get() < 4.5 && m_timer.get() > 4.3){
                armMotor.set(0.0);
              } else if(m_timer.get() < 10.0 && m_timer.get() > 4.5){
                intakeMotor.set(0.4);
                m_shooter.arcadeDrive(-.1817, 0, false);
                m_transition.set(-0.8);
              } else{
                intakeMotor.set(0.0);
                m_shooter.arcadeDrive(0, 0, false);
                m_transition.set(0);
              }
              break;
      /********************************************************* */
      //High Auto, facing backward, pickup up extra cell then aim.
      /********************************************************* */
            case kHighAutoPickup:
           // double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
            if(m_timer.get() < 1.0){
              armMotor.set(0.6);
              NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      
            } else if(m_timer.get() < 4.0 && m_timer.get() > 1.0){
              armMotor.set(0.0);
              intakeMotor.set(0.75);
              if(leftEncoders.getDistance() < 1000){
                m_driveTrain.arcadeDrive(0.55, 0);
              } else{
                m_driveTrain.stopMotor();
              }
            } else if(m_timer.get() > 4.0 && m_timer.get() < 4.9){
              m_driveTrain.tankDrive(0, 0);
              leftEncoders.reset();
            } else if(m_timer.get() < 8.0 && m_timer.get() > 4.9){
              if(leftEncoders.getDistance() < 650){
              m_driveTrain.tankDrive(0.72, -0.6);
              } else{
              m_driveTrain.tankDrive(0, 0);
              }
              intakeMotor.set(0.0);
            } else if(m_timer.get() < 8.2 && m_timer.get() > 8.0){
              m_driveTrain.tankDrive(0, 0);
            } else if(m_timer.get() > 8.2 && m_timer.get() < 8.7){
              NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
              double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); 
              double heading_error = -tx; 
              double kPAim = -.27854 + 0.014615 * m_voltage;
              double min_command = 0.01;
              double steering_adjust = 0.0;
              double left_command = 0.0;
              double right_command = 0.0;
                 if( tx > 0) {
                   steering_adjust = kPAim * heading_error - min_command;
                  }
                  else if( tx < 0) {
                    steering_adjust = kPAim * heading_error + min_command;
                    }
                left_command += steering_adjust;
               right_command-=steering_adjust; 
            m_driveTrain.tankDrive(left_command, right_command);
            } else if(m_timer.get() > 8.2 && m_timer.get() < 8.7){
              m_shooter.arcadeDrive(-RobotMap.SHOOTER_SPEED, 0, false);
            } else if(m_timer.get() > 8.7 && m_timer.get() < 14.0){
              m_shooter.arcadeDrive(-RobotMap.SHOOTER_SPEED, 0, false);
              intakeMotor.set(0.4);
              m_transition.set(-0.9);
              NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            } else if(m_timer.get() > 14.0 && m_timer.get() < 14.5){
              m_transition.set(0.0);
              intakeMotor.set(0.0);
              m_shooter.arcadeDrive(0, 0, false);
            }
            break;
      /************************************************* */
      //High Auto, facing forward, 3 balls MAX
      /************************************************* */
            case kHighAuto:
              if(m_timer.get() < 4.0){
                if(leftEncoders.getDistance() > -1000){
                  m_driveTrain.arcadeDrive(-0.55, 0);
                } else{
                  m_driveTrain.stopMotor();
                }
              } else if(m_timer.get() < 4.5 && m_timer.get() > 4.0){
                armMotor.set(0.5);
              } else if(m_timer.get() < 4.9 && m_timer.get() > 4.5){
                armMotor.set(0);
                m_driveTrain.tankDrive(-0.6, 0.6);
              } else if(m_timer.get() > 5.5 && m_timer.get() < 6.9){
                m_driveTrain.tankDrive(0, 0);
              /*
              } else if(m_timer.get() > 5.8 && m_timer.get() < 6.8){
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
                double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); 
                
                double heading_error = -tx; 
                double kPAim = -.27854 + 0.014615 * m_voltage;
                double min_command = 0.01;
                double steering_adjust = 0.0;
                double left_command = 0.0;
                double right_command = 0.0;
       
                   if( tx > 0) {
                     steering_adjust = kPAim * heading_error - min_command;
                    }
                    else if( tx < 0) {
                      steering_adjust = kPAim * heading_error + min_command;
                      }
                  left_command += steering_adjust;
                 right_command-=steering_adjust; 
              m_driveTrain.tankDrive(left_command, right_command);
              */
              } else if(m_timer.get() > 6.9 && m_timer.get() < 7.8){
                m_shooter.arcadeDrive(-RobotMap.SHOOTER_SPEED, 0, false);
              } else if(m_timer.get() > 7.8 && m_timer.get() < 14.0){
                m_shooter.arcadeDrive(-RobotMap.SHOOTER_SPEED, 0, false);
                m_transition.set(-0.9);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
              } else if(m_timer.get() > 14.0 && m_timer.get() < 14.5){
                m_transition.set(0.0);
                m_shooter.arcadeDrive(0, 0, false);
              }
            break;
       /*****************************************************************/
      //Shoot from the center towards the high goal and then move forward
      /*****************************************************************/
            case kCenterHighAuto:
          
            if(m_timer.get() < 4){
              m_shooter.arcadeDrive(-.4,0,false);
              m_transition.set(-RobotMap.TRANSITION_SPEED);
            } else if (m_timer.get() <5 && m_timer.get() > 4){
              m_shooter.arcadeDrive(0,0,false);
              m_transition.set(0);
            } else if (m_timer.get() < 8 && m_timer.get() > 5){
              m_driveTrain.arcadeDrive(.5,0);
            } else if (m_timer.get() < 9 && m_timer.get() > 8){
              m_driveTrain.arcadeDrive(0, 0);
            }
            break;
      /******************************************************************/
      //Shoot from the center towards the low goal and then move forward
      /******************************************************************/
            case kCenterLowAuto:
            if(m_timer.get() < 4.0){
              if(leftEncoders.getDistance() < 1000){
                m_driveTrain.arcadeDrive(0.51, 0.13); //old number is (0.51, 0.1)
                
              } else{
                m_driveTrain.stopMotor();}
              } else if(m_timer.get() < 4.3 && m_timer.get() > 4.0){
                armMotor.set(0.2);
              } else if(m_timer.get() < 4.5 && m_timer.get() > 4.3){
                armMotor.set(0.0);
              } else if(m_timer.get() < 10.0 && m_timer.get() > 4.5){
                intakeMotor.set(0.45);
                m_shooter.arcadeDrive(-.1817, 0, false);
                m_transition.set(-0.8);
              } else{
                intakeMotor.set(0.0);
                m_shooter.arcadeDrive(0, 0, false);
                m_transition.set(0);
              }
              break;
      /******************************************************************/
      //Shoot from the right towards the high goal and then move forward
      /******************************************************************/  
             case kLRHighAuto:
             if(m_timer.get() < 4){
              m_shooter.arcadeDrive(-RobotMap.SHOOTER_SPEED + .01,0,false);
              m_transition.set(-RobotMap.TRANSITION_SPEED);
            } else if (m_timer.get() <5 && m_timer.get() > 4){
              m_shooter.arcadeDrive(0,0,false);
              m_transition.set(0);
            } else if (m_timer.get() < 8 && m_timer.get() > 5){
              m_driveTrain.arcadeDrive(.5,0);
            } else if (m_timer.get() < 9 && m_timer.get() > 8){
              m_driveTrain.arcadeDrive(0, 0);
            }
            break;
      /***********************************************************************/
      //Wait 5 Shoot from the center towards the High goal and then move forward
      /************************************************************************/
      case kCenterHighW5Auto:
          
      if(m_timer.get() < 9 && m_timer.get() >5 ){
        m_shooter.arcadeDrive(-RobotMap.SHOOTER_SPEED,0,false);
        m_transition.set(-RobotMap.TRANSITION_SPEED);
      } else if (m_timer.get() <9 && m_timer.get() > 8){
        m_shooter.arcadeDrive(0,0,false);
        m_transition.set(0);
      } else if (m_timer.get() < 12 && m_timer.get() > 9){
        m_driveTrain.arcadeDrive(.5,0);
      } else if (m_timer.get() < 13 && m_timer.get() > 12){
        m_driveTrain.arcadeDrive(0, 0);
      }
      break;
            

            default:
              if(m_timer.get() < 3.0){
                m_driveTrain.arcadeDrive(0.5, 0);
              } else {
                m_driveTrain.stopMotor();
              }
              break;
            }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


    if(DriverStation.getMatchTime() <30 && DriverStation.getMatchTime() > 29){
      m_driverController.setRumble(RumbleType.kRightRumble, 1.0);
      m_driverController.setRumble(RumbleType.kLeftRumble, 1.0);
    } else{
      m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
      m_driverController.setRumble(RumbleType.kRightRumble, 0.0);
    }
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").setNumber(0);

    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
/*
    SmartDashboard.putNumber("ShooterSpeed1", -firstMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("ShooterSpeed2", -secondMotor.getSelectedSensorVelocity());
*/
  //*****DRIVETRAIN CODE******* */

    //double left_command;
    //double right_command;


      double triggerVal =
        (m_driverController.getRightTriggerAxis()
        - m_driverController.getLeftTriggerAxis())
        * RobotMap.DRIVING_SPEED;

      double stick =
       (m_driverController.getLeftX())
       * RobotMap.TURNING_RATE;

      //left_command = (triggerVal + stick) * RobotMap.DRIVING_SPEED;
      //right_command = (triggerVal - stick) * RobotMap.DRIVING_SPEED; 

     // m_driveTrain.tankDrive(left_command, right_command);

      m_driveTrain.tankDrive(triggerVal + stick, triggerVal - stick);

    //****************DRIVER Y************* */
    /****************SHOOT FAR************ */
    if(m_driverController.getYButton()){
      m_shooter.arcadeDrive(-0.25, 0.0, false);
     // intakeMotor.set(RobotMap.ROLLER_SPEED/2);
    //****************DRIVER A************* */
    /****************SHOOT LOW************ */
    } else if(m_driverController.getAButton()){
      m_shooter.arcadeDrive(-(RobotMap.LOW_SPEED), 0.0, false);
      intakeMotor.set(RobotMap.ROLLER_SPEED);
    //****************DRIVER B************* */
    /****************SHOOT CLOSE************ */
    } else if(m_driverController.getBButton()){
      m_shooter.arcadeDrive(-0.41, 0.0, false);
    //****************OPERATOR X************* */
    /*****************ROLLER IN************** */
    } else if(m_operatorController.getXButton()){
      intakeMotor.set(RobotMap.ROLLER_SPEED);
    //****************OPERATOR B************* */
    /****************ROLLER OUT************ */
    } else if(m_operatorController.getBButton()){
      intakeMotor.set(-RobotMap.ROLLER_SPEED);
    } else{
      intakeMotor.set(0);
      m_shooter.arcadeDrive(0, 0);
    }
    //************DRIVER AND OPERATOR STICK************* */
    /****************TRANSITION************ */
   // m_transition.set(m_driverController.getRightY()-0.05);
    m_transition.set(m_operatorController.getRightY()-0.05);


    //m_transition.set(m_driverController.getRightY()*.7);
    //**************OPERATOR Y*********************** */
    /****************ARM UP************************** */
    if(m_operatorController.getYButton() && armSwitch.get()){
      armMotor.set(RobotMap.ARM_SPEED);
    //**************OPERATOR A*********************** */
    /****************ARM DOWN************************ */
    } else if(m_operatorController.getAButton() && armDownSwitch.get()){
      armMotor.set(-RobotMap.ARM_SPEED);
    } else{
      armMotor.set(0.0);
    }


    //Limelight 
   // double kP = -0.07; //-0.1328
   // double min_command = 0.035; //0.001
   // double steering_adjust = 0.0;

   // double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); 
    //double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); 

/*
    if(m_driverController.getRightBumper()){ 
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

      double heading_error = -tx;  
      if( tx > 1) {
        steering_adjust = kP * heading_error - min_command;
      }
      else if( tx < 1) {
        steering_adjust = kP * heading_error + min_command;
      }
        left_command+=steering_adjust;
        right_command-=steering_adjust; 
        m_driveTrain.tankDrive(left_command, right_command);
    } else{

    }
  

    if(m_driverController.getLeftBumperPressed()){
      m_driveTrain.setMaxOutput(0.6);
    }
    if(m_driverController.getLeftBumperReleased()){
      m_driveTrain.setMaxOutput(1.0);
    }
*/
    //*****************OPERATOR TRIGGERS*************** */
    /*******************TURN SPINNER******************* */
    spinnerMotor.set(
        m_operatorController.getRightTriggerAxis()
         - m_operatorController.getLeftTriggerAxis());
  
    //****************OPERATOR BUMPERS***************** */
    /*****************DEPLOY SPINNER******************* */
    if(m_operatorController.getRightBumper()){
      panelMotor.set(RobotMap.DEPLOY_SPEED);
    } else if(m_operatorController.getLeftBumper()){
      panelMotor.set(-RobotMap.DEPLOY_SPEED);
    } else{
      panelMotor.set(0.0);
    }
/*
    if(m_operatorController.getBackButton()){
      climbMotor.set(0.85);
    } else{
      climbMotor.set(0.0);
    }
*/

    if(m_operatorController.getStartButton()){
      climbMotor.set(-RobotMap.CLIMB_SPEED);
    } else if(m_operatorController.getBackButton()){
      climbMotor.set(RobotMap.CLIMB_SPEED);
    } else{
      climbMotor.set(0.0);
    }
/*
    if(m_driverController.getStartButton()){
      winchMotor.set(RobotMap.WINCH_SPEED);
    } else{
      winchMotor.set(0.0);
    }
*/
    
 
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
