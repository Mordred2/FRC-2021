
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
  double drive_ticksPerDegree = 0.11;
  double drive_ticksPerInch = .57;
  double drive_encoderError = .4;

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
  public boolean isArcRunning;
  public double wantedIndex;
  public double shootIndex;
  public double ballCount;
  public double wheelWidth;

  public double state;
  final Timer t = new Timer();

  @Override
  public void robotInit() {
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
 
     
     collectorPID = collector.getPIDController();
     
 
     
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

 
     ballCount = 0;
     //LIMIT SWITCHES
     upSwitch = new DigitalInput(2);
     downSwitch = new DigitalInput(3);
 
     
 
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
 
     
 
    //  SmartDashboard.putNumber("Collector P Gain", collector_kP);
    //  SmartDashboard.putNumber("Collector I Gain", collector_kI);
    //  SmartDashboard.putNumber("Collector D Gain", collector_kD);
    //  SmartDashboard.putNumber("Collector I Zone", collector_kIz);
    //  SmartDashboard.putNumber("Collector Feed Forward", collector_kFF);
    //  SmartDashboard.putNumber("Collector Max Output", collector_kMaxOutput);
    //  SmartDashboard.putNumber("Collector Min Output", collector_kMinOutput);
    //  SmartDashboard.putNumber("Collector Error", collector_encoderError);
    //  SmartDashboard.putNumber("Collector Position", 0);
    //  SmartDashboard.putNumber("Collector Target", 0);
 
 
     t.start();
     //this is right
   
  }



  @Override
  public void autonomousInit(){
    drivePidTestInit();
    
  }

  @Override
  public void autonomousPeriodic() {
    //drivePidTestPeriodic();
    slalomRun();
  }

  @Override
  public void teleopInit(){
    // DRIVETRAIN
     SpeedControllerGroup leftMotors;
     SpeedControllerGroup rightMotors;
     leftMotors = new SpeedControllerGroup(leftMotorF, leftMotorB);
     rightMotors = new SpeedControllerGroup(rightMotorF, rightMotorB);
     m_robotDrive = new DifferentialDrive(leftMotors, rightMotors);

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
    drive_leftEncoderBFinalPosition = leftEncoderBValue + (drive_ticksPerDegree * turnDegrees);
    drive_rightEncoderFFinalPosition = rightEncoderFValue + (drive_ticksPerDegree * turnDegrees);
    drive_rightEncoderBFinalPosition = rightEncoderBValue + (drive_ticksPerDegree * turnDegrees);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition, slot);
    leftMotorBPID.setReference(drive_leftEncoderBFinalPosition, ControlType.kPosition, slot);
    rightMotorFPID.setReference(drive_rightEncoderFFinalPosition, ControlType.kPosition, slot);
    rightMotorBPID.setReference(drive_rightEncoderBFinalPosition, ControlType.kPosition, slot);
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
    /*if(drivePositionreached=true){
      isArcRunning=false;
    }*/
    if(drivePositionreached){
      setDrivePids(0,drive_kP, drive_kI, drive_kD, drive_kIz, drive_kFF, drive_encoderError, drive_kMaxOutput, drive_kMinOutput);
    }
    return drivePositionreached;
  }

  public void driveDistance(double driveDistanceInch, int slot){
    double leftEncoderFValue = leftEncoderF.getPosition();
    double leftEncoderBValue = leftEncoderB.getPosition();
    double rightEncoderFValue = rightEncoderF.getPosition();
    double rightEncoderBValue = rightEncoderB.getPosition();
    drive_leftEncoderFFinalPosition = leftEncoderFValue + (drive_ticksPerInch * driveDistanceInch);
    drive_leftEncoderBFinalPosition = leftEncoderBValue + (drive_ticksPerInch * driveDistanceInch);
    drive_rightEncoderFFinalPosition = rightEncoderFValue - (drive_ticksPerInch * driveDistanceInch);
    drive_rightEncoderBFinalPosition = rightEncoderBValue - (drive_ticksPerInch * driveDistanceInch);
    leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition,slot);
    leftMotorBPID.setReference(drive_leftEncoderBFinalPosition, ControlType.kPosition, slot);
    rightMotorFPID.setReference(drive_rightEncoderFFinalPosition, ControlType.kPosition, slot);
    rightMotorBPID.setReference(drive_rightEncoderBFinalPosition, ControlType.kPosition, slot);
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
  public void setDrivePids(int slot, double kP ,double kI , double kD,  double kIz,  double kFF, double encoderError,  double kMaxOutput,  double kMinOutput){
     leftMotorFPID.setP(kP,slot);
    leftMotorFPID.setI(kI,slot);
    leftMotorFPID.setD(kD,slot);
    leftMotorFPID.setIZone(kIz,slot);
    leftMotorFPID.setFF(kFF,slot);
    leftMotorFPID.setSmartMotionAllowedClosedLoopError(encoderError,slot);
    leftMotorFPID.setOutputRange(kMinOutput, kMaxOutput,slot);
    
    leftMotorBPID.setP(kP,slot);
    leftMotorBPID.setI(kI,slot);
    leftMotorBPID.setD(kD,slot);
    leftMotorBPID.setIZone(kIz,slot);
    leftMotorBPID.setFF(kFF,slot);
    leftMotorBPID.setSmartMotionAllowedClosedLoopError(encoderError,slot);
    leftMotorBPID.setOutputRange(kMinOutput, kMaxOutput,slot);

    rightMotorFPID.setP(kP,slot);
    rightMotorFPID.setI(kI,slot);
    rightMotorFPID.setD(kD,slot);
    rightMotorFPID.setIZone(kIz,slot);
    rightMotorFPID.setFF(kFF,slot);
    rightMotorFPID.setSmartMotionAllowedClosedLoopError(drive_encoderError,slot);
    rightMotorFPID.setOutputRange(kMinOutput, kMaxOutput,slot);

    rightMotorBPID.setP(kP,slot);
    rightMotorBPID.setI(kI,slot);
    rightMotorBPID.setD(kD,slot);
    rightMotorBPID.setIZone(kIz,slot);
    rightMotorBPID.setFF(kFF,slot);
    rightMotorBPID.setSmartMotionAllowedClosedLoopError(drive_encoderError,slot);
    rightMotorBPID.setOutputRange(kMinOutput, kMaxOutput,slot);
  }

  public void setLeftPids(int slot, double kP ,double kI , double kD,  double kIz,  double kFF, double encoderError,  double kMaxOutput,  double kMinOutput){
    leftMotorFPID.setP(kP,slot);
   leftMotorFPID.setI(kI,slot);
   leftMotorFPID.setD(kD,slot);
   leftMotorFPID.setIZone(kIz,slot);
   leftMotorFPID.setFF(kFF,slot);
   leftMotorFPID.setSmartMotionAllowedClosedLoopError(encoderError,slot);
   leftMotorFPID.setOutputRange(kMinOutput, kMaxOutput,slot);
   
   leftMotorBPID.setP(kP,slot);
   leftMotorBPID.setI(kI,slot);
   leftMotorBPID.setD(kD,slot);
   leftMotorBPID.setIZone(kIz,slot);
   leftMotorBPID.setFF(kFF,slot);
   leftMotorBPID.setSmartMotionAllowedClosedLoopError(encoderError,slot);
   leftMotorBPID.setOutputRange(kMinOutput, kMaxOutput,slot);
 }

 public void setRightPids(int slot, double kP ,double kI , double kD,  double kIz,  double kFF, double encoderError,  double kMaxOutput,  double kMinOutput){

 rightMotorFPID.setP(kP,slot);
 rightMotorFPID.setI(kI,slot);
 rightMotorFPID.setD(kD,slot);
 rightMotorFPID.setIZone(kIz,slot);
 rightMotorFPID.setFF(kFF,slot);
 rightMotorFPID.setSmartMotionAllowedClosedLoopError(drive_encoderError,slot);
 rightMotorFPID.setOutputRange(kMinOutput, kMaxOutput,slot);

 rightMotorBPID.setP(kP,slot);
 rightMotorBPID.setI(kI,slot);
 rightMotorBPID.setD(kD,slot);
 rightMotorBPID.setIZone(kIz,slot);
 rightMotorBPID.setFF(kFF,slot);
 rightMotorBPID.setSmartMotionAllowedClosedLoopError(drive_encoderError,slot);
 rightMotorBPID.setOutputRange(kMinOutput, kMaxOutput,slot);
}

  @Override
  public void testInit(){
    //PID CONTROLLERS 
     
       
  }
  @Override
  public void testPeriodic(){
    
  }


  double drivePidTestInches = 12;

  public void allAuton(){
    state = SmartDashboard.getNumber("State Tester", state);
    double p = SmartDashboard.getNumber("P Gain", drive_kP);
    double i = SmartDashboard.getNumber("I Gain", drive_kI);
    double d = SmartDashboard.getNumber("D Gain", drive_kD);
    double iz = SmartDashboard.getNumber("I Zone", drive_kIz);
    double ff = SmartDashboard.getNumber("Feed Forward", drive_kFF);
    double max = SmartDashboard.getNumber("Max Output", drive_kMaxOutput);
    double min = SmartDashboard.getNumber("Min Output", drive_kMinOutput);
    double encoderError = SmartDashboard.getNumber("Encoder Error", drive_encoderError);
    double inches = SmartDashboard.getNumber("Inches", drivePidTestInches);
    double newState = SmartDashboard.getNumber("New State", 1);
    
    // if PID coefficients on SmartDashboard have changed, write new values to controller
   if(isArcRunning == false){
    boolean updatePids = false;
    if((p != drive_kP)) { 
      updatePids = true;
      drive_kP = p; 
    }
    if((i != drive_kI)) { 
      updatePids = true;
      drive_kI = i; 
    }
    if((d != drive_kD)) { 
      updatePids = true;
      drive_kD = d; 
    }
    if((iz != drive_kIz)) { 
      updatePids = true;
      drive_kIz = iz; 
    }
    if((ff != drive_kFF)) { 
      updatePids = true;
      drive_kFF = ff; 
    }
    if((encoderError != drive_encoderError)) { 
      updatePids = true;
      drive_encoderError = encoderError; 
    }
    if((max != drive_kMaxOutput) || (min != drive_kMinOutput)) { 
      updatePids = true;
      drive_kMinOutput = min; 
      drive_kMaxOutput = max; 
    }
    if(updatePids == true){
      setDrivePids(0,drive_kP, drive_kI, drive_kD, drive_kIz, drive_kFF, drive_encoderError, drive_kMaxOutput, drive_kMinOutput);
    }
  }
    if(drivePidTestInches != inches){
      drivePidTestInches = inches;
    }

    if((newState == 0) ) { 
      state = 0;
      SmartDashboard.putNumber("New State", 1);
    }else{
      SmartDashboard.putNumber("New State", newState);
    }

    SmartDashboard.putNumber("State", state);
    SmartDashboard.putNumber("Drive State", drive_state);
    SmartDashboard.putNumber("Drive Left Front Position", leftEncoderF.getPosition());
    SmartDashboard.putNumber("Drive Left Back Position", leftEncoderB.getPosition());
    SmartDashboard.putNumber("Drive Right Front Position", rightEncoderF.getPosition());
    SmartDashboard.putNumber("Drive Right Back Position", rightEncoderB.getPosition());
    SmartDashboard.putNumber("Drive Left Front Target", drive_leftEncoderFFinalPosition);
    SmartDashboard.putNumber("Drive Left Back Target", drive_leftEncoderBFinalPosition);
    SmartDashboard.putNumber("Drive Right Front Target", drive_rightEncoderFFinalPosition);
    SmartDashboard.putNumber("Drive Right Back Target", drive_rightEncoderBFinalPosition);
    SmartDashboard.putNumber("Inches", drivePidTestInches);
    SmartDashboard.putNumber("P Gain", drive_kP);
    SmartDashboard.putNumber("I Gain", drive_kI);
    SmartDashboard.putNumber("D Gain", drive_kD);
    SmartDashboard.putNumber("I Zone", drive_kIz);
    SmartDashboard.putNumber("Feed Forward", drive_kFF);
    SmartDashboard.putNumber("Max Output", drive_kMaxOutput);
    SmartDashboard.putNumber("Min Output", drive_kMinOutput);
    SmartDashboard.putNumber("Encoder Error", drive_encoderError); 
    SmartDashboard.putBoolean("is arc running", isArcRunning);
  }

  public void drivePidTestInit(){
    //PID CONTROLLERS 
    resetDriveEncoders();
    
    leftMotorFPID = leftMotorF.getPIDController();
    leftMotorBPID = leftMotorB.getPIDController(); 
    rightMotorFPID = rightMotorF.getPIDController();
    rightMotorBPID = rightMotorB.getPIDController();


    leftMotorF.restoreFactoryDefaults();
    leftMotorB.restoreFactoryDefaults();
    rightMotorF.restoreFactoryDefaults();
    rightMotorB.restoreFactoryDefaults();

    drive_kP = 0.4; //0.1; 
    drive_kI = 0; //1e-4;
    drive_kD = 0; //1; 
    drive_kIz = 0; 
    drive_kFF = 0; 
    drive_kMaxOutput = .3; 
    drive_kMinOutput = -.3;
    drive_encoderError = .4;
    wheelWidth = 22;
    setDrivePids(0,drive_kP, drive_kI, drive_kD, drive_kIz, drive_kFF, drive_encoderError, drive_kMaxOutput, drive_kMinOutput);
    state = 0;
    drive_state = 0;
    resetDriveEncoders();
    isArcRunning = false;
  }
  double time;
  public void drivePidTestPeriodic(){
    if(state == 0){
      driveDistance(15, 0);
      state++;
    }
    if(state == 1){
      if(driveComplete()){
        state++;
      }
    }
    if(state == 2){
      arcMove(19, 41, .25, .25, 0);
      state++;
    }
    if(state == 3){
      if(driveComplete()){
        state++;
      }
    }
  
   allAuton();
  }

  Timer m_timeoutTimer;
  Double m_timeoutExpires;
  private void timeout_start(double second){
    m_timeoutTimer = new Timer();
    m_timeoutTimer.start();
    m_timeoutExpires = m_timeoutTimer.get();
  }

  private boolean timeout_complete(){
    if (m_timeoutExpires < m_timeoutTimer.get()){
      return true;
    }else{
      return false;
    }
  }
  //remeber dont forget not to not add 22 to the outer wheel
 public void arcMove(double lRadius, double rRadius, double maxSpeed, double cPercent, int slot){
  double leftMin;
  double rightMin;
  double leftMax;
  double rightMax;
  double leftkp;
  double rightkp;
   double lCircumference = 2 * Math.PI * lRadius;
   double rCircumference = 2 * Math.PI * rRadius;
   double lArc = cPercent * lCircumference;
   double rArc = cPercent * rCircumference;
   double speedRatio;
   isArcRunning = true;
   
   if(lCircumference > rCircumference){
    speedRatio = (rCircumference/lCircumference);
    leftMax = maxSpeed;
    rightMax = maxSpeed * speedRatio;
    rightkp = drive_kP * speedRatio; 
    leftkp = drive_kP;
   } else{
    speedRatio = .85 * (lCircumference/rCircumference);
    rightMax = maxSpeed;
    leftMax = maxSpeed * speedRatio; 
    leftkp = maxSpeed * speedRatio;
    rightkp = drive_kP;
   }
   leftMin = leftMax * -1;
   rightMin = rightMax * -1;
   setLeftPids(1,leftkp, drive_kI, drive_kD, drive_kIz, drive_kFF, drive_encoderError, leftMax, leftMin);
   setRightPids(2,rightkp, drive_kI, drive_kD, drive_kIz, drive_kFF, drive_encoderError, rightMax, rightMin);
   double leftEncoderFValue = leftEncoderF.getPosition();
   double leftEncoderBValue = leftEncoderB.getPosition();
   double rightEncoderFValue = rightEncoderF.getPosition();
   double rightEncoderBValue = rightEncoderB.getPosition();
   drive_leftEncoderFFinalPosition = leftEncoderFValue + (drive_ticksPerInch * lArc);
   drive_leftEncoderBFinalPosition = leftEncoderBValue + (drive_ticksPerInch * lArc);
   drive_rightEncoderFFinalPosition = rightEncoderFValue - (drive_ticksPerInch * rArc);
   drive_rightEncoderBFinalPosition = rightEncoderBValue - (drive_ticksPerInch * rArc);
   leftMotorFPID.setReference(drive_leftEncoderFFinalPosition, ControlType.kPosition, slot);
   leftMotorBPID.setReference(drive_leftEncoderBFinalPosition, ControlType.kPosition, slot);
   rightMotorFPID.setReference(drive_rightEncoderFFinalPosition, ControlType.kPosition, slot);
   rightMotorBPID.setReference(drive_rightEncoderBFinalPosition, ControlType.kPosition, slot);
 }

  public void slalomRun(){
    double arcSpeed = .25;
    //Start With Robot Set at E2
    if(state == 0){
      driveDistance(15, 0);
      state++;
    }
    if(state == 1){
      if(driveComplete() == true){
        isArcRunning = false;
        state++;
     }
    }
    if(state == 2){
      arcMove(19, 41, arcSpeed, .25, 0);
      state++;
    }
    if(state == 3){
      if(driveComplete() == true){
        isArcRunning = false;
        state++;
     }
    }
    if(state == 4){
      arcMove(41, 19, arcSpeed, .25, 0);
      state++;
    }
    if(state == 5){
      if(driveComplete() == true){
        isArcRunning = false;
        state++;
     }
    }
    if(state == 6){
      driveDistance(120, 0);
      state++;
    }
    if(state == 7){
      if(driveComplete() == true){
        state++;
     }
    }
    if(state == 8){
      arcMove(41, 19, arcSpeed, .25, 0);
      state++;
    }
    if(state == 9){
      if(driveComplete() == true){
        isArcRunning = false;
        state++;
     }
    }
    if(state == 10){
      arcMove(19, 41, arcSpeed, 1, 0);
      state++;
    }
    if(state == 11){
      if(driveComplete() == true){
        state++;
        isArcRunning = false;
     }
    }
    if(state == 12){
      arcMove(41, 19, arcSpeed, .25, 0);
      state++;
    }
    if(state == 13){
      if(driveComplete() == true){
        state++;
        isArcRunning = false;
     }
    }
    if(state == 14){
      driveDistance(120, 0);
      state++;
    }
    if(state == 15){
      if(driveComplete() == true){
        state++;
     }
    }
    if(state == 16){
    arcMove(41, 19, arcSpeed, .25, 0);
      state++;
    }
    if(state == 17){
      if(driveComplete() == true){
        isArcRunning = false;
        state++;
     }
    }
    if(state == 18){
      arcMove(19, 41, arcSpeed, .25, 0);
        state++;
      }
      if(state == 19){
        if(driveComplete() == true){
          isArcRunning = false;
          state++;
       }
      }
      allAuton();
  }

  public void barrelRun(){
  double arcSpeed = .25;
  if(state == 0){
    driveDistance(105, 0);
    state++;
  }
  if(state == 1){
    if(driveComplete()){
      state++;
    }
  }
  if(state == 2){
    arcMove(41, 19, arcSpeed, 1, 0);
    state++;
  }
  if(state == 3){
    if(driveComplete()){
      isArcRunning = false;
      state++;
    }
  }
  if(state == 4){
    driveDistance(90, 0);
    state++;
  }
  if(state == 5){
    if(driveComplete()){
      state++;
    }
  }
  if(state == 6){
    arcMove(19, 41, arcSpeed, 1, 0);
    state++;
  }
  if(state == 7){
    if(driveComplete()){
      isArcRunning = false;
      state++;
    }
  }
  if(state == 8){
    driveDistance(60, 0);
    state++;
  }
  if(state == 9){
    if(driveComplete()){
      state++;
    }
  }
  if(state == 10){
    arcMove(41, 19, arcSpeed, .5, 0);
    state++;
  }
  if(state == 11){
    if(driveComplete()){
      isArcRunning = false;
      state++;
    }
  }
  if(state == 12){
    turnDegrees(22, 0);
    state++;
  }
  if(state == 13){
    if(driveComplete()){
      state++;
    }
  }
  if(state == 14){
    driveDistance(162, 0);
    state++;
  }
  if(state == 15){
    if(driveComplete()){
      state++;
    }
  }
  if(state == 16){
    turnDegrees(-22, 0);
    state++;
  }
  if(state == 17){
    if(driveComplete()){
      state++;
    }
  }
  if(state == 18){
    driveDistance(150, 0);
    state++;
  }
  if(state == 19){
    if(driveComplete()){
      state++;
    }
  }
  allAuton();
}
  public void BouncePath(){
    double arcSpeed = .25;
    //Start With Robot Set at E2
    if(state == 0){
      driveDistance(15, 0);
      state++;
    }
    if(state == 1){
      if(driveComplete() == true){
        isArcRunning = false;
        state++;
     }
    }
    if(state == 2){
      arcMove(19, 41, arcSpeed, .25, 0);
      state++;
    }
    if(state == 3){
      if(driveComplete() == true){
        isArcRunning = false;
        state++;
     }
    }
    if(state == 4){
     driveDistance(30, 0);
      state++;
    }
    if(state == 5){
      if(driveComplete() == true){
        state++;
     }
    }
    if(state == 6){
      driveDistance(-30, 0);
      state++;
    }
    if(state == 7){
      if(driveComplete() == true){
        state++;
     }
    }
    if(state == 8){
     turnDegrees(-22.5, 0);
      state++;
    }
    if(state == 9){
      if(driveComplete() == true){
        state++;
     }
    }
    if(state == 10){
      driveDistance(-134, 0);
      state++;
    }
    if(state == 11){
      if(driveComplete() == true){
        state++;
     }
    }
    if(state == 12){
      turnDegrees(115, 0);
      state++;
    }
    if(state == 13){
      if(driveComplete() == true){
        state++;
     }
    }
    if(state == 14){
      arcMove(19, 41, arcSpeed, .25, 0);
      state++;
    }
    if(state == 15){
      if(driveComplete() == true){
        state++;
        isArcRunning = false;
     }
    }
    if(state == 16){
      driveDistance(90, 0);
      state++;
    }
    if(state == 17){
      if(driveComplete() == true){
        state++;
     }
    }
    if(state == 18){
      driveDistance(-90, 0);
      state++;
    }
    if(state == 19){
      if(driveComplete() == true){
        state++;
     }
    }
    if(state == 20){
      arcMove(41, 19, arcSpeed, -.25, 0);
        state++;
      }
      if(state == 21){
        if(driveComplete() == true){
          isArcRunning = false;
          state++;
       }
      }
      if(state == 22){
        driveDistance(-30, 0);
        state++;
      }
      if(state == 23){
        if(driveComplete() == true){
          state++;
        }
      }
      if(state == 24){
        arcMove(41, 19, arcSpeed, -.25, 0);
          state++;
        }
        if(state == 25){
          if(driveComplete() == true){
            isArcRunning = false;
            state++;
         }
        }
        if(state == 26){
          driveDistance(-90, 0);
          state++;
        }
        if(state == 27){
          if(driveComplete() == true){
            state++;
          }
        }
        if(state == 28){
          driveDistance(30, 0);
          state++;
        }
        if(state == 29){
          if(driveComplete() == true){
            state++;
          }
        }
        if(state == 30){
          arcMove(19, 41, arcSpeed, .25, 0);
            state++;
          }
          if(state == 31){
            if(driveComplete() == true){
              isArcRunning = false;
              state++;
           }
          }
      allAuton();
  }
}
/*these are comments

Slalom Course
______________________
Start at theredical E-2
0. Arc to left of inner radius 19 to D-3
2. Arc right radius of 19 inches
4. Drive 120 inches forward
6. Arc right inner radius 19
8. Arc Left complete circle inner radius of 19
10. Arc Right inner radius of 19 inches
12. Drive 120 inches forward
14. Arc right inner radius 19 inches
16. Arc left inner radius 19 inches
18.Drive 60 inches forward
Course Complete
______________________




Barrel Racing Course
______________________
Start front center C-2
Drive forward 90 inches
Full arc to the right inner radius 19
Drive 90 forward
Full arc left inner radius 19
Drive forward 60 
Arc right inner radius 19 .5 completion
Turn Degrees 22 right
Drive 162 inches forward
Turn 22 Degrees left
Drive 150 inches forward
Course Complete
______________________



Bounce Path Course
______________________
Start at C-2
Arc left inner radius 19 .25 completion
Drive forward 30 inches
Drive backwards 30 inches
Turn left 22.5 degrees
Drive backwards 134
Turn right 115 degrees
Arc left inside radius 19 .25  completion
Drive forward 90 inches
Drive backwards 90 inches
Arc backwards right inner radius 19 .25 completion
Drive 30 inches backwards
Arc backwards right inner circle 19 .25 completion
Drive backwards 90 inches
Drive forward 30 inches
Arc left inner radius 19 .25 completion
Course Comeplete
______________________
no more comments*/