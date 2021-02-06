
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import java.lang.ProcessBuilder.Redirect;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;




import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Robot extends TimedRobot {
  // JOYSTICKS
  private Joystick driveStick;
  private Joystick shootStick;
  // MOTORS
  //private final VictorSP m_leftMotor = new VictorSP(0);
  //private final VictorSP m_rightMotor = new VictorSP(1);
  private CANSparkMax leftMotorF;
  private CANSparkMax leftMotorB;
  private CANSparkMax rightMotorF;
  private CANSparkMax rightMotorB;


  //private final Spark leftShooter = new Spark(8);
  //private final Spark rightShooter = new Spark(9);
  //private final Victor collector = new Victor(3);
  private Talon flopper;
  private Talon aimer;
  private CANSparkMax indexer;
  private CANSparkMax leftShooter;
  private CANSparkMax rightShooter;
  private CANSparkMax collector;

  // ENCODERS
  private Encoder aimerEncoder;
  private CANEncoder indexEncoder;
  private CANEncoder leftEncoderF;
  private CANEncoder leftEncoderB;
  private CANEncoder rightEncoderF;
  private CANEncoder rightEncoderB;
  private CANEncoder collectorEncoder;

  //PID CONTROLLERS 

  private CANPIDController leftMotorFPID;
  private CANPIDController leftMotorBPID; 
  private CANPIDController rightMotorFPID;
  private CANPIDController rightMotorBPID;
  private CANPIDController collectorPID;
  public double drive_kP ; 
  public double drive_kI ;
  public double drive_kD ; 
  public double drive_kIz; 
  public double drive_kFF; 
  public double drive_kMaxOutput; 
  public double drive_kMinOutput;

  double drive_state = 0;
  double drive_leftEncoderFFinalPosition = 0; 
  double drive_leftEncoderBFinalPosition = 0; 
  double drive_rightEncoderFFinalPosition = 0; 
  double drive_rightEncoderBFinalPosition = 0; 
  double drive_ticksPerDegree = 100;
  double drive_ticksPerInch = 100;
  double drive_encoderError = 100;

  public double collector_kP ; 
  public double collector_kI ;
  public double collector_kD ; 
  public double collector_kIz; 
  public double collector_kFF; 
  public double collector_kMaxOutput; 
  public double collector_kMinOutput;
  double collector_encoderError = 100;

  // LIMIT SWITCHES
  DigitalInput upSwitch, downSwitch;

  // DRIVETRAIN
  //private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;
  private DifferentialDrive m_robotDrive;
  // NETWORK TABLES
  NetworkTableInstance inst;
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  //ADDITIONAL SENSORS
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.424, 0.023);
  private final Color kOverYellow = ColorMatch.makeColor(0.361, 0.4, 0.2);
  private double IR = m_colorSensor.getIR();

  /*private Rev2mDistanceSensor distOnboard; 
  private Rev2mDistanceSensor distOnboard; 
  Gyro euro = new AnalogGyro(0);*/
  // VARIABLES
  public boolean isCollectorOn;
  public boolean isIndexerOn;
  public boolean isCollectorBackwards;
  public boolean isIndexerBackwards;
  public boolean readyToIndex;
  public double wantedIndex;
  public double shootIndex;
  public double ballCount;

  public double state;
  final Timer t = new Timer();

  @Override
  public void robotInit() {
    
  }

  @Override
  public void autonomousInit(){
    owenInit();
  }

  @Override
  public void teleopInit(){
    owenInit();
  }

  @Override
public void autonomousPeriodic() {
  SmartDashboard.putNumber("State", state);
}

  public void teleopPeriodic() {
    //REPEATEDLY GET THESE VALUES
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double distanceAngle = Math.toRadians(y);
    double distance = 74.25 / Math.tan(0.401426 + distanceAngle);
    double aimerEncoderValue = (aimerEncoder.getDistance());
    double indexPosition = indexEncoder.getPosition();

    //More Sensing
   // rangeSensor();
    colorSensor();

    // SMART DASHBOARD
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("Aimer", aimerEncoder.getDistance());
    SmartDashboard.putNumber("IndexCoder", indexPosition);
    SmartDashboard.putNumber("wanted index", wantedIndex);
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("Left Front Encoder", leftEncoderF.getPosition());
    SmartDashboard.putNumber("Left Back Encoder", leftEncoderB.getPosition());
    SmartDashboard.putNumber("Right Front Encoder", rightEncoderF.getPosition());
    SmartDashboard.putNumber("Right Back Encoder", rightEncoderB.getPosition());
    SmartDashboard.putNumber("Drive Left Front Position", leftEncoderF.getPosition());
    SmartDashboard.putNumber("Drive Left Back Position", leftEncoderB.getPosition());
    SmartDashboard.putNumber("Drive Right Front Position", rightEncoderF.getPosition());
    SmartDashboard.putNumber("Drive Right Back Position", rightEncoderB.getPosition());
    SmartDashboard.putNumber("Drive Left Front Target", drive_leftEncoderFFinalPosition);
    SmartDashboard.putNumber("Drive Left Back Target", drive_leftEncoderBFinalPosition);
    SmartDashboard.putNumber("Drive Right Front Target", drive_rightEncoderFFinalPosition);
    SmartDashboard.putNumber("Drive Right Back Target", drive_rightEncoderBFinalPosition);



  //RUN PARTS OF THE ROBOT WHEN BUTTONS PRESSED
  countBalls();
  manualAim();
  heShootsHeScores(indexPosition);


  if(collectorRun())
  runCollector(.4);
  if(collectorBack())
  runCollector(-.75);
  if(collectorStop() || collectorBackStop())
  runCollector(0);
  if(indexerBack())
  runIndexer(-.5);
  if (indexerBackStop())
  runIndexer(0);
  if(collectorStop() || collectorBackStop())
  runCollector(0);
  if (pleaseStop())
  stopThePress();
  if (shooterPrimedPressed()){
    resetIndexer();
    runShooter(-1);}
  if (fwoooopTimePressed()){
    resetIndexer();
    ballCount = ballCount + 1;
  }
  fwoooop(indexPosition);
  if (fire()){
    runIndexer(.75);
    runCollector(1);}
  if(driveStick.getTriggerReleased()){
    runIndexer(0);
    runShooter(0);
    runCollector(0);
    ballCount = 0;
  }
  if (shootStick.getRawButton(12))
  resetAimer();
  if(stopShooter()){
    runShooter(0);
  }
  if (stopIndexer()){
    runIndexer(0);
  }

  flopIt();
  if (aimingEngaged()) {
    aim(x, aimerEncoderValue, distance);
  } 
  else drive(-driveStick.getY(), driveStick.getX(), driveStick.getRawButton(2));
}
  //FUNCTIONS

  public void heShootsHeScores(double indexPosition) {
    if(shooterPrimed()){
      wantedIndex = -.11;
    }
    IndexBackwards(wantedIndex, 1, indexPosition);}

  public void fwoooop(double indexPosition) {
    if(fwoooopTime()) {
      wantedIndex = .40+((ballCount-1)*.25);}
    indexForward(wantedIndex, 1, indexPosition);
  }


  public void flopIt() {
    if(shootStick.getRawButton(6) && (upSwitch.get()))
      flopper.set(-1);
    else if(shootStick.getRawButton(4) && (downSwitch.get()))
      flopper.set(.25);
    else flopper.set(0);
  }

  public void aim(double h, double aimerEncoderValue, double distance) {
    // Heading Error
    double x = h;
    double steering_adjust = 0;
    if (x > .32)
      steering_adjust = .235 + (.015) * Math.log(x);
    if (x < -.32)
      steering_adjust = -(.235 + (.015) * Math.log(-x));
    m_robotDrive.tankDrive(steering_adjust, -steering_adjust);
    if ((x < -.32) && (x > .32))
    {
      //adjust(distance);
    }
  }

  public void adjust(double distance) {
    double currentWormDistance = (aimerEncoder.getDistance()/ 2.62);
    //needs Math
    double currentAngle = currentWormDistance;
    double wantedAngle = (.1733 * ((distance*distance)/144) - 4.8*(distance/12) + 58.667);
    if(currentAngle < wantedAngle)
    aimUp(wantedAngle, currentAngle);
    if(currentAngle > wantedAngle)
    aimDown(wantedAngle, currentAngle);
  }

  public void aimDown(double wantedAngle, double currentAngle) {
    //needs Math
  double aimerSpeed = wantedAngle - currentAngle;
  aimer.set(aimerSpeed);
  }

  public void aimUp(double wantedAngle, double currentAngle) {
    //needs Math
    double aimerSpeed = wantedAngle - currentAngle;
    aimer.set(-aimerSpeed);
  }

  public void drive(double speed, double turn, boolean quickTurn) {
    m_robotDrive.curvatureDrive(speed, turn, quickTurn);
  }

  /*public void rangeSensor(){
    if(distOnboard.isRangeValid()) {
      SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
      SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
    }
  }*/

  public void colorSensor(){
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
  
    if (match.color == kYellowTarget) {
      colorString = "Yes";
    } else if(match.color == kOverYellow) {
      colorString = "Over";
    } else {
      colorString = "No";
    }

    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Index Now", colorString);
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
  }

  public void indexForward(double wantedIndexIn, double indexSpeedMagnitude, double indexPosition){
    double wantedIndexForward = wantedIndexIn;
    if(wantedIndexForward > 0){
    double indexSpeed = indexSpeedMagnitude;
    if (indexPosition < wantedIndexForward){
      indexer.set(indexSpeed);
  }
   if(indexPosition >= wantedIndexForward){ 
     indexer.set(0);
     wantedIndexForward = 0;
    }
  }
  }

  public void IndexBackwards(double wantedIndexIn, double indexSpeedMagnitude, double indexPosition){
    double wantedIndexBackwards = (wantedIndexIn);
    if(wantedIndexBackwards < 0){
    double indexSpeed = (-indexSpeedMagnitude);
    if(indexPosition > wantedIndexBackwards){
      indexer.set(indexSpeed);
  }
    if(indexPosition < wantedIndexBackwards && wantedIndexBackwards != 0){ 
      wantedIndexBackwards = 0;
      indexer.set(0);
    }
  }
    SmartDashboard.putNumber("wanted Index back", wantedIndex);
  }

  public void runIndexer(double indexerSpeed){
    indexer.set(indexerSpeed);
  }

  public void runShooter(double shootSpeed){
    leftShooter.set(shootSpeed);
    rightShooter.set(-shootSpeed);
  }

  public void runCollector(double collectorSpeed){
    collector.set(collectorSpeed);
  }

  public void resetIndexer(){
    indexEncoder.setPosition(0);
  }

  public void resetDriveEncoders(){
    leftEncoderF.setPosition(0);
    leftEncoderB.setPosition(0);
    rightEncoderF.setPosition(0);
    rightEncoderB.setPosition(0);
  }

  public void resetAimer(){
    aimerEncoder.reset();
  }

  public void checkButtons(){
    aimingEngaged();
    shooterPrimed();
    shooterPrimedPressed();
    fwoooopTime();
    fwoooopTimePressed();
    fire();
    stopShooter();
    goingUp();
    goingDown();
    stopIndexer();
    pleaseStop();
  }

  public boolean aimingEngaged() {
    return shootStick.getRawButton(2);
  }

  public boolean shooterPrimed() {
    return shootStick.getRawButton(7);
  }
  public boolean shooterPrimedPressed() {
    return shootStick.getRawButtonPressed(7);
  }
  public boolean fwoooopTime() {
    return shootStick.getTrigger();
  }
  public boolean fwoooopTimePressed() {
    return shootStick.getTriggerPressed();
  }
  public boolean fire() {
    return driveStick.getTrigger();
  }
  public boolean stopShooter() {
    return shootStick.getRawButtonPressed(8);
  }
  public boolean goingUp(){
   return driveStick.getRawButtonPressed(5);
  }
  public boolean goingDown(){
    return driveStick.getRawButtonPressed(3);
  }
  public boolean stopIndexer() {
    return shootStick.getRawButtonPressed(10);
  }
  public boolean pleaseStop(){
    return driveStick.getRawButton(7);
  }
  public void stopThePress(){
    indexer.set(0);
    collector.set(0);
    runShooter(0);
    drive(0, 0, false);
    flopper.set(0);
    aimer.set(0);
  }

  public void manualAim(){
    if(shootStick.getY() >= .5 || shootStick.getY() <= -.5)
    aimer.set(-shootStick.getY());
    else aimer.set(0);
  }

  public void countBalls(){
    if(ballCount >= 5) ballCount = 5;
    if(ballCount < 0) ballCount = 0;
    if(shootStick.getRawButtonPressed(11)) ballCount = ballCount -1;
  } 

    


  public void turnDegrees(double turnDegrees, int slot){
    double leftEncoderFValue = leftEncoderF.getPosition();
    double leftEncoderBValue = leftEncoderB.getPosition();
    double rightEncoderFValue = rightEncoderF.getPosition();
    double rightEncoderBValue = rightEncoderB.getPosition();
    drive_leftEncoderFFinalPosition = leftEncoderFValue + (drive_ticksPerDegree * turnDegrees);
    drive_leftEncoderBFinalPosition = leftEncoderBValue - (drive_ticksPerDegree * turnDegrees);
    drive_rightEncoderFFinalPosition = rightEncoderFValue + (drive_ticksPerDegree * turnDegrees);
    drive_rightEncoderBFinalPosition = rightEncoderBValue - (drive_ticksPerDegree * turnDegrees);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition, slot);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition, slot);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition, slot);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition, slot);
    drive_state = 1;
  }

  public boolean driveComplete(){
    double leftEncoderFValue = leftEncoderF.getPosition();
    double leftEncoderBValue = leftEncoderB.getPosition();
    double rightEncoderFValue = rightEncoderF.getPosition();
    double rightEncoderBValue = rightEncoderB.getPosition();
    boolean drivePositionreached = true;
    if (Math.abs(drive_leftEncoderFFinalPosition - leftEncoderFValue) > drive_encoderError){
      drivePositionreached = false;
    }
    if (Math.abs(drive_leftEncoderBFinalPosition - leftEncoderBValue) > drive_encoderError){
      drivePositionreached = false;
    }
    if (Math.abs(drive_rightEncoderFFinalPosition - rightEncoderFValue) > drive_encoderError){
      drivePositionreached = false;
    }
    if (Math.abs(drive_rightEncoderBFinalPosition - rightEncoderBValue) > drive_encoderError){
      drivePositionreached = false;
    }
    if (drivePositionreached){
      drive_state = 2;
    }
    return drivePositionreached;
  }

  public void driveDistance(double driveDistanceInch, int slot){
    double leftEncoderFValue = leftEncoderF.getPosition();
    double leftEncoderBValue = leftEncoderB.getPosition();
    double rightEncoderFValue = rightEncoderF.getPosition();
    double rightEncoderBValue = rightEncoderB.getPosition();
    drive_leftEncoderFFinalPosition = leftEncoderFValue + (drive_ticksPerInch * driveDistanceInch);
    drive_leftEncoderBFinalPosition = leftEncoderBValue - (drive_ticksPerInch * driveDistanceInch);
    drive_rightEncoderFFinalPosition = rightEncoderFValue + (drive_ticksPerInch * driveDistanceInch);
    drive_rightEncoderBFinalPosition = rightEncoderBValue - (drive_ticksPerInch * driveDistanceInch);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition,slot);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition, slot);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition, slot);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition, slot);
    drive_state = 1;
  }

  public boolean collectorRun(){
    return shootStick.getRawButton(3);
  }
  public boolean collectorStop(){
    return shootStick.getRawButtonReleased(3);
  }
  public boolean collectorBack(){
    return shootStick.getRawButton(5);
  }
  public boolean indexerBack(){
    return shootStick.getRawButton(9);
  }
  public boolean indexerBackStop(){
    return shootStick.getRawButtonReleased(9);
  }
  public boolean collectorBackStop(){
    return shootStick.getRawButtonReleased(5);
  }

  public void stateMachine(){
 
    if(state == 0){
      state++;
    }
    if(state == 1){
      state++;
    }
    if(state == 2){
      state++;
    }
    if(state == 3){
      state++;
    }
    if(state == 4){
      state++;
    }
    if(state == 5){
      state++;
    }
    if(state == 6){
      state++;
    }
    if(state == 7){
      state++;
    }
    if(state == 8){
      state++;
    }
  }

  public void owenInit(){


    // JOYSTICKS
    driveStick = new Joystick(0);
    shootStick = new Joystick(1);
    flopper = new Talon(2);
    aimer = new Talon(5);
    
    // ENCODERS
    aimerEncoder = new Encoder(0, 1);

    // MOTORS
    //private final VictorSP m_leftMotor = new VictorSP(0);
    //private final VictorSP m_rightMotor = new VictorSP(1);
    leftMotorF = new CANSparkMax(5, MotorType.kBrushless);
    leftMotorB = new CANSparkMax(6, MotorType.kBrushless);
    rightMotorF = new CANSparkMax(7, MotorType.kBrushless);
    rightMotorB = new CANSparkMax(8, MotorType.kBrushless);


    //private final Spark leftShooter = new Spark(8);
    //private final Spark rightShooter = new Spark(9);
    //private final Victor collector = new Victor(3)
    indexer = new CANSparkMax(1, MotorType.kBrushless);
    leftShooter = new CANSparkMax(2, MotorType.kBrushed);
    rightShooter = new CANSparkMax(3, MotorType.kBrushed);
    collector = new CANSparkMax(4, MotorType.kBrushless);

    // ENCODERS
    indexEncoder = indexer.getEncoder(EncoderType.kHallSensor, 1);
    leftEncoderF = leftMotorF.getEncoder(EncoderType.kHallSensor, 1);
    leftEncoderB = leftMotorB.getEncoder(EncoderType.kHallSensor, 1);
    rightEncoderF = rightMotorF.getEncoder(EncoderType.kHallSensor, 1);
    rightEncoderB = rightMotorB.getEncoder(EncoderType.kHallSensor, 1);
    collectorEncoder = collector.getEncoder(EncoderType.kHallSensor, 1);

    //PID CONTROLLERS 
    leftMotorFPID = leftMotorF.getPIDController();
    leftMotorBPID = leftMotorB.getPIDController(); 
    rightMotorFPID = rightMotorF.getPIDController();
    rightMotorBPID = rightMotorB.getPIDController();
    collectorPID = collector.getPIDController();
    drive_kP = 0.1; 
    drive_kI = 1e-4;
    drive_kD = 1; 
    drive_kIz = 0; 
    drive_kFF = 0; 
    drive_kMaxOutput = .25; 
    drive_kMinOutput = -.25;

    // DRIVETRAIN
    //private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    leftMotors = new SpeedControllerGroup(leftMotorF, leftMotorB);
    rightMotors = new SpeedControllerGroup(rightMotorF, rightMotorB);
    m_robotDrive = new DifferentialDrive(leftMotors, rightMotors);
    // NETWORK TABLES
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

        //SENSORS
    CameraServer.getInstance().startAutomaticCapture();
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kOverYellow);
    /*distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    distOnboard.setRangeProfile(RangeProfile.kHighSpeed);
    distOnboard.setRangeProfile(RangeProfile.kHighAccuracy);
    distOnboard.setRangeProfile(RangeProfile.kLongRange);
    distOnboard.setRangeProfile(RangeProfile.kDefault);
    //enable the Distance Sensor background Thread
    distOnboard.setAutomaticMode(true);*/
    //CAN AND ENCODERS
    collector.restoreFactoryDefaults();
    aimerEncoder.setDistancePerPulse(1.0/650);
    resetIndexer();
    resetDriveEncoders();

    ballCount = 0;
    //LIMIT SWITCHES
    upSwitch = new DigitalInput(2);
    downSwitch = new DigitalInput(3);

    leftMotorFPID.setP(drive_kP,0);
    leftMotorFPID.setI(drive_kI,0);
    leftMotorFPID.setD(drive_kD,0);
    leftMotorFPID.setIZone(drive_kIz,0);
    leftMotorFPID.setFF(drive_kFF,0);
    leftMotorFPID.setSmartMotionAllowedClosedLoopError(drive_encoderError,0);
    leftMotorFPID.setOutputRange(drive_kMinOutput, drive_kMaxOutput,0);
    
    leftMotorBPID.setP(drive_kP,0);
    leftMotorBPID.setI(drive_kI,0);
    leftMotorBPID.setD(drive_kD,0);
    leftMotorBPID.setIZone(drive_kIz,0);
    leftMotorBPID.setFF(drive_kFF,0);
    leftMotorBPID.setSmartMotionAllowedClosedLoopError(drive_encoderError,0);
    leftMotorBPID.setOutputRange(drive_kMinOutput, drive_kMaxOutput,0);

    rightMotorFPID.setP(drive_kP,0);
    rightMotorFPID.setI(drive_kI,0);
    rightMotorFPID.setD(drive_kD,0);
    rightMotorFPID.setIZone(drive_kIz,0);
    rightMotorFPID.setFF(drive_kFF,0);
    rightMotorFPID.setSmartMotionAllowedClosedLoopError(drive_encoderError,0);
    rightMotorFPID.setOutputRange(drive_kMinOutput, drive_kMaxOutput,0);

    rightMotorBPID.setP(drive_kP,0);
    rightMotorBPID.setI(drive_kI,0);
    rightMotorBPID.setD(drive_kD,0);
    rightMotorBPID.setIZone(drive_kIz,0);
    rightMotorBPID.setFF(drive_kFF,0);
    rightMotorBPID.setSmartMotionAllowedClosedLoopError(drive_encoderError,0);
    rightMotorBPID.setOutputRange(drive_kMinOutput, drive_kMaxOutput,0);

    collector_kP = 0.1; 
    collector_kI = 1e-4;
    collector_kD = 1; 
    collector_kIz = 0; 
    collector_kFF = 0; 
    collector_kMaxOutput = .25; 
    collector_kMinOutput = -.25;
    
    collectorPID.setP(collector_kP,0);
    collectorPID.setI(collector_kI,0);
    collectorPID.setD(collector_kD,0);
    collectorPID.setIZone(collector_kIz,0);
    collectorPID.setFF(collector_kFF,0);
    collectorPID.setSmartMotionAllowedClosedLoopError(collector_encoderError,0);
    collectorPID.setOutputRange(collector_kMinOutput, collector_kMaxOutput);

    SmartDashboard.putNumber("Drive P Gain", drive_kP);
    SmartDashboard.putNumber("Drive I Gain", drive_kI);
    SmartDashboard.putNumber("Drive D Gain", drive_kD);
    SmartDashboard.putNumber("Drive I Zone", drive_kIz);
    SmartDashboard.putNumber("Drive Feed Forward", drive_kFF);
    SmartDashboard.putNumber("Drive Max Output", drive_kMaxOutput);
    SmartDashboard.putNumber("Drive Min Output", drive_kMinOutput);
    SmartDashboard.putNumber("Drive Error", drive_encoderError);
    SmartDashboard.putNumber("Drive Left Front Position", 0);
    SmartDashboard.putNumber("Drive Left Back Position", 0);
    SmartDashboard.putNumber("Drive Right Front Position", 0);
    SmartDashboard.putNumber("Drive Right Back Position", 0);
    SmartDashboard.putNumber("Drive Left Front Target", 0);
    SmartDashboard.putNumber("Drive Left Back Target", 0);
    SmartDashboard.putNumber("Drive Right Front Target", 0);
    SmartDashboard.putNumber("Drive Right Back Target", 0);

    SmartDashboard.putNumber("Collector P Gain", collector_kP);
    SmartDashboard.putNumber("Collector I Gain", collector_kI);
    SmartDashboard.putNumber("Collector D Gain", collector_kD);
    SmartDashboard.putNumber("Collector I Zone", collector_kIz);
    SmartDashboard.putNumber("Collector Feed Forward", collector_kFF);
    SmartDashboard.putNumber("Collector Max Output", collector_kMaxOutput);
    SmartDashboard.putNumber("Collector Min Output", collector_kMinOutput);
    SmartDashboard.putNumber("Collector Error", collector_encoderError);
    SmartDashboard.putNumber("Collector Position", 0);
    SmartDashboard.putNumber("Collector Target", 0);

    state = 0;

    t.start();
    //this is right
  }
  @Override
  public void testInit(){
    owenInit();
  }
  @Override
  public void testPeriodic(){
    if(state == 0){
      driveDistance(12, 0);
      state++;
    }
    if(state == 1){
      if(driveComplete() == true){
      state++;
      }
    }
    SmartDashboard.putNumber("State", state);
    SmartDashboard.putNumber("Drive State", drive_state);
    SmartDashboard.putNumber("Left Front Encoder", leftEncoderF.getPosition());
    SmartDashboard.putNumber("Left Back Encoder", leftEncoderB.getPosition());
    SmartDashboard.putNumber("Right Front Encoder", rightEncoderF.getPosition());
    SmartDashboard.putNumber("Right Back Encoder", rightEncoderB.getPosition());
    SmartDashboard.putNumber("Drive Left Front Position", leftEncoderF.getPosition());
    SmartDashboard.putNumber("Drive Left Back Position", leftEncoderB.getPosition());
    SmartDashboard.putNumber("Drive Right Front Position", rightEncoderF.getPosition());
    SmartDashboard.putNumber("Drive Right Back Position", rightEncoderB.getPosition());
    SmartDashboard.putNumber("Drive Left Front Target", drive_leftEncoderFFinalPosition);
    SmartDashboard.putNumber("Drive Left Back Target", drive_leftEncoderBFinalPosition);
    SmartDashboard.putNumber("Drive Right Front Target", drive_rightEncoderFFinalPosition);
    SmartDashboard.putNumber("Drive Right Back Target", drive_rightEncoderBFinalPosition);
  
  }

}