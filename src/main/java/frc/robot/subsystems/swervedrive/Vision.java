package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.GeneralMethods;
import frc.robot.Robot;
import frc.robot.commands.swervedrive.drivebase.rotateToHeading;

import java.awt.Desktop;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;


/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision
{

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout                     = Constants.FIELD_LAYOUT;
  /**
   * Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}.
   */
  private final       double              maximumAmbiguity                = 0.25;
  /**
   * Photon Vision Simulation
   */
  public              VisionSystemSim     visionSim;
  /**
   * Count of times that the odom thinks we're more than 10meters away from the april tag.
   */
  private             double              longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private             Supplier<Pose2d>    currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private             Field2d             field2d;


  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field)
  {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values())
      {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset)
  {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent())
    {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else
    {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive)
  {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent())
    {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    for (Cameras camera : Cameras.values())
    {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent())
      {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                         pose.timestampSeconds,
                                         camera.curStdDevs);
      }
    }

  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   *  <li> No Pose Estimates could be generated</li>
   * <li> The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera)
  {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation())
    {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est ->
              debugField
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }


  /**
   * Filter pose via the ambiguity and find best estimate between all of the camera's throwing out distances more than
   * 10m for a short amount of time.
   *
   * @param pose Estimated robot pose.
   * @return Could be empty if there isn't a good reading.
   */
  @Deprecated(since = "2024", forRemoval = true)
  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose)
  {
    if (pose.isPresent())
    {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed)
      {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity)
        {
          bestTargetAmbiguity = ambiguity;
        }
      }
      //ambiguity to high dont use estimate
      if (bestTargetAmbiguity > maximumAmbiguity)
      {
        return Optional.empty();
      }

      //est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1)
      {
        longDistangePoseEstimationCount++;

        //if it calculates that were 10 meter away for more than 10 times in a row its probably right
        if (longDistangePoseEstimationCount < 10)
        {
          return Optional.empty();
        }
      } else
      {
        longDistangePoseEstimationCount = 0;
      }
      return pose;
    }
    return Optional.empty();
  }


  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id)
  {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera)
  {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList)
    {
      if (result.hasTargets())
      {
        for (PhotonTrackedTarget i : result.getTargets())
        {
          if (i.getFiducialId() == id)
          {
            return i;
          }
        }
      }
    }
    return target;

  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim()
  {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on localhost.
   */
  private void openSimCameraViews()
  {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE))
    {
//      try
//      {
//        Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
//        Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
//        Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
//      } catch (IOException | URISyntaxException e)
//      {
//        e.printStackTrace();
//      }
    }
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField()
  {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values())
    {
      if (!c.resultsList.isEmpty())
      {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets())
        {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets)
    {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent())
      {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }


    //Inaccurate at lower target pitches.
  public double getDistanceToTarget(Cameras camera) {
    Optional<PhotonPipelineResult> result0 = camera.getLatestResult();
    if (result0.isPresent()) {
      var result = result0.get();

      if (result.hasTargets()) {
        double estimatedTargetPitch = Math.toRadians(result.getBestTarget().getPitch());
        double targetHeight = Constants.APRILTAG_HEIGHTS[result.getBestTarget().getFiducialId() - 1]; //Dummy Value; Change Later
        double estimatedTargetDistance = PhotonUtils.calculateDistanceToTargetMeters
        (
          Cameras.OFFSET_CAM.robotToCamTransform.getZ(),
          targetHeight, 
          0.0,
          estimatedTargetPitch
        );

        return estimatedTargetDistance;
      }
    }
    return -1.0;
  }

  public Transform3d getTargetTransform(Cameras cameraEnum, boolean isSpecificID, int fiducialId) {
    Optional<PhotonPipelineResult> result0 = cameraEnum.getLatestResult();

    if (result0.isEmpty()) {
      DriverStation.reportWarning("Get Target Pose Failed; Is Your Camera On?", false);
      return new Transform3d();
    }

    var result = result0.get();

    if (!result.hasTargets()) {
      DriverStation.reportWarning("Get Target Pose called with no targets in sight.", false);
      return new Transform3d();
    }

    var targetArray = new ArrayList<PhotonTrackedTarget>(10);
    targetArray.addAll(result.getTargets());
    
    if (isSpecificID) {

      if (fiducialId <= 0 || fiducialId > Constants.APRILTAG_HEIGHTS.length) {
        DriverStation.reportWarning("getTargetTransform called while fiducialID is invalid", null);
        return new Transform3d();
      }

      for (int index = 0; index < targetArray.size(); index++) {
        boolean IDExists = (targetArray.get(index).getFiducialId() == fiducialId) ? true : false;
        if (IDExists) {
          PhotonTrackedTarget target = targetArray.get(index);

          Transform3d pose = target.bestCameraToTarget;

          return pose;
        }
      }
      DriverStation.reportWarning("Specific ID Not Found", false);
      return new Transform3d();
    }

    PhotonTrackedTarget target = result.getBestTarget();

    Transform3d pose = target.bestCameraToTarget;

    return pose;
  }

  public ArrayList<PhotonTrackedTarget> getAllTargets(Cameras cameraEnum) {
    Optional<PhotonPipelineResult> result0 = cameraEnum.getLatestResult();

    if (result0.isEmpty()) {
      DriverStation.reportWarning("Get Target Pose Failed; Is Your Camera On?", false);
      return new ArrayList<PhotonTrackedTarget>();
    }

    var result = result0.get();

    if (!result.hasTargets()) {
      DriverStation.reportWarning("Get Target Pose called with no targets in sight.", false);
      return new ArrayList<PhotonTrackedTarget>();
    }

    var targetArray = new ArrayList<PhotonTrackedTarget>(20);
    targetArray.addAll(result.getTargets());

    return targetArray;
  }



  public Rotation2d getAngleToHUB(Cameras cameraEnum, SwerveSubsystem drivebase) {
    ArrayList<PhotonTrackedTarget> targetArray = getAllTargets(cameraEnum);

    //All Red Alliance HUB IDs
    var idArrayRed = new ArrayList<Integer>(Arrays.asList(9,10,8,5,11,2,4,3));

    //All Blue Alliance HUB IDs
    var idArrayBlue = new ArrayList<Integer>(Arrays.asList(19,20,18,27,26,25,24,21));

    var offsetTagArray = new ArrayList<Integer>(Arrays.asList(26, 18, 20, 24, 9, 11, 3, 5));
    var centerTagArray = new ArrayList<Integer>(Arrays.asList(25, 27, 19, 21, 10, 2, 4, 8));

    var idArrayFront = new ArrayList<Integer>(Arrays.asList(9,10,25,26));
    var idArrayRight = new ArrayList<Integer>(Arrays.asList(11,2,18,27));
    var idArrayLeft = new ArrayList<Integer>(Arrays.asList(8,5,21,24));

    if (targetArray.isEmpty()) {
      //return new Translation3d();
    }

    if (drivebase.isRedAlliance()) {
      targetArray.removeIf(target -> !idArrayRed.contains(target.getFiducialId()));
    } else {
      targetArray.removeIf(target -> !idArrayBlue.contains(target.getFiducialId()));
    }

    int targetIndex = 0;
    double lowestTargetAmbiguity = 1;
    boolean targetArrayFlag = false;

    //Gets lowest ambiguity target
    for (PhotonTrackedTarget trackedTarget : targetArray) {
      targetArrayFlag = true;
      //Lower ambiguity values are better.
      double currentTargetAmbiguity = trackedTarget.getPoseAmbiguity();
      if (currentTargetAmbiguity < lowestTargetAmbiguity && Double.compare(currentTargetAmbiguity, -1.0) != 0) {
        lowestTargetAmbiguity = currentTargetAmbiguity;
        targetIndex = targetArray.indexOf(trackedTarget);
      }
    }


    //When on right side, reverse all tagOffsets, and sin and cos are opposite. 

    if (targetArrayFlag == false) {
      DriverStation.reportWarning("No Correct Targets Found", false);
      return new Rotation2d();
    } else {
      var bestTarget = targetArray.get(targetIndex);
      Transform3d HUBCenterTransform = new Transform3d();
      Transform3d targetTransform = bestTarget.getBestCameraToTarget();

      //-1 is left side, 0 is center, 1 is right side.
      int currentSide;

      Translation2d tagOffset = new Translation2d();

      //WARN: Might be presuming unchanging reference point again.
      if (offsetTagArray.contains(bestTarget.getFiducialId())) {
        tagOffset = new Translation2d(Units.inchesToMeters(23.5), Units.inchesToMeters(14));
      } else {
        if (centerTagArray.contains(bestTarget.getFiducialId())) {
          tagOffset = new Translation2d(Units.inchesToMeters(23.5), 0);
        }
      }

      if (idArrayLeft.contains(bestTarget.getFiducialId())) {
        currentSide = -1;
      } else if (idArrayFront.contains(bestTarget.getFiducialId())) {
        currentSide = 0;
      } else if (idArrayRight.contains(bestTarget.getFiducialId())) {
        currentSide = 1;
      } else {
        DriverStation.reportError("ID Found was not front, left, or right. This should never happen.", false);
        return new Rotation2d();
      }

      double targetYaw = bestTarget.getYaw();
      //WARN: This was unknowingly not doing anything in working test. If any errors occur, implement this.
      /*
      int signFlip = 0;
      if (targetYaw >= 0) {
        signFlip = -1;
      } else {
        signFlip = 1;
      }
      */

      Rotation2d abf = drivebase.getHeading();

      double bc = Math.sqrt((Math.pow(targetTransform.getX(), 2) + Math.pow(targetTransform.getY(), 2)));
      Rotation2d fbc = Rotation2d.fromRadians(Math.asin(targetTransform.getY() / bc));
      Rotation2d abc = abf.plus(fbc);

      double ch = 0;
      double bh = 0;
      double ei = 0;
      double bi = 0;

      Rotation2d headingToHUBCenter = new Rotation2d();

      if (currentSide == 0) {
        ch = Math.cos(abc.getRadians()) * bc;
        bh = Math.sin(abc.getRadians()) * bc;
        bi = bh - tagOffset.getY();
        ei = ch + tagOffset.getX();
        return GeneralMethods.calculateAngleToPoint(new Translation2d(ei, bi));
      }

      if (currentSide == 1) {
        ch = Math.sin(abc.getRadians()) * bc;
        bh = Math.cos(abc.getRadians()) * bc;
        bi = bh + tagOffset.getY();
        ei = ch + tagOffset.getX();
        return GeneralMethods.calculateAngleToPoint(new Translation2d(bi, ei));
      }

      if (currentSide == -1) {
        ch = Math.sin(abc.getRadians()) * bc;
        bh = Math.cos(abc.getRadians()) * bc;
        bi = bh - tagOffset.getY();
        ei = ch - tagOffset.getX();
        return GeneralMethods.calculateAngleToPoint(new Translation2d(bi, ei));
      }

      return new Rotation2d();

      /*

      //Rotation2d headingToHUBCenter = GeneralMethods.calculateAngleToPoint(new Translation2d(ei, bi), signFlip);

      //Rotation2d headingToHUBCenter = PhotonUtils.getYawToPose(new Pose2d(), new Pose2d(new Translation2d(bi, ei), new Rotation2d()));
      double robotToCenterDistance = Math.sqrt((Math.pow(bi, 2) + Math.pow(ei, 2)));
      //Rotation2d headingToHUBCenter = Rotation2d.fromRadians(Math.atan(bi / ei) * signFlip);

      System.out.print(" heading: " + abf);
      System.out.print(" bc: " + bc);
      System.out.print(" fbc: " + fbc);
      System.out.print(" abc: " + abc);
      System.out.print(" ch: " + ch);
      System.out.println(" bh: " + bh);
      System.out.print(" bi: " + bi);
      System.out.print(" ei: " + ei);
      System.out.println("robotToCenterDistance: " + robotToCenterDistance);
      System.out.println("headingToHUBCenter: " + headingToHUBCenter);
      //return new Translation3d(HUBCenterTransform.getX(), HUBCenterTransform.getY(), HUBCenterTransform.getZ());

      */
      }
  }

  public Transform3d getTargetTransformOffset(Cameras camera, Translation3d offsetPoint, boolean isSpecificID, int fiducialId) {
    Transform3d camRelativeTransform3d = getTargetTransform(camera, isSpecificID, fiducialId);

    if (camRelativeTransform3d.equals(new Transform3d())) {
      return new Transform3d();
    }

    Rotation3d camRotation = camRelativeTransform3d.getRotation();

    double camRelativeTargetX = camRelativeTransform3d.getX();
    double camRelativeTargetY = camRelativeTransform3d.getY();
    double camRelativeTargetZ = camRelativeTransform3d.getZ();

    double originRelativeTargetX = camRelativeTargetX + Cameras.OFFSET_CAM.robotToCamTransform.getX();
    double originRelativeTargetY = camRelativeTargetY - Cameras.OFFSET_CAM.robotToCamTransform.getY();
    double originRelativeTargetZ = camRelativeTargetZ + Cameras.OFFSET_CAM.robotToCamTransform.getZ();

    double offsetRelativeTargetX = originRelativeTargetX - offsetPoint.getX();
    double offsetRelativeTargetY = originRelativeTargetY + offsetPoint.getY();
    double offsetRelativeTargetZ = originRelativeTargetZ + offsetPoint.getZ();

    //return new Transform3d(originRelativeTargetX, originRelativeTargetY, originRelativeTargetZ, camRotation);
    return new Transform3d(offsetRelativeTargetX, offsetRelativeTargetY, offsetRelativeTargetZ, camRotation);
  }

  //Should provide the actual camera name, not the name property of the camera-- OFFSET_CAMERA instead of offsetCamera
  public Transform3d getCameraTransform(Cameras cameraEnum, String cameraName) {
    //Cameras specificCamera = Cameras.valueOf(cameraName);
    //return specificCamera.robotToCamTransform;
    return Cameras.OFFSET_CAM.robotToCamTransform;
  }

  /**
   * Camera Enum to select each camera
   */
  public static enum Cameras
  {
    /**
     * Left Camera
     */
    /*
    LEFT_CAM("left",
             new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
             new Translation3d(Units.inchesToMeters(12.056),
                               Units.inchesToMeters(10.981),
                               Units.inchesToMeters(8.44)),
             VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    
    /**
     * Right Camera
     */
    
    OFFSET_CAM("offsetCamera",
              new Rotation3d(0, 0, 0),
              new Translation3d(Units.inchesToMeters(7.0),
                                Units.inchesToMeters(7.75),
                                Units.inchesToMeters(29)),
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    /**
     * Center Camera
     */
    CENTER_CAM("centerCam",
               new Rotation3d(0, 30, 0),
               new Translation3d(Units.inchesToMeters(13.25),
                                 0,
                                 Units.inchesToMeters(11.00)),
               VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));


    /**
     * Latency alert to use when high latency is detected.
     */
    public final  Alert                        latencyAlert;
    /**
     * Camera instance for comms.
     */
    public final  PhotonCamera                 photonCamera;
    /**
     * Pose estimator for camera.
     */
    public final  PhotonPoseEstimator          poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1>               singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1>               multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of the robot
     */
    private final Transform3d                  robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public        Matrix<N3, N1>               curStdDevs;
    /**
     * Estimated robot pose.
     */
    public        Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /**
     * Simulated camera instance which only exists during simulations.
     */
    public        PhotonCameraSim              cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary queries.
     */
    public        List<PhotonPipelineResult>   resultsList       = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private       double                       lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix)
    {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      photonCamera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout,
                                              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                              robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation())
      {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(photonCamera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim)
    {
      if (Robot.isSimulation())
      {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target. This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult()
    {
      if (resultsList.isEmpty())
      {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult       = resultsList.get(0);
      double               amiguity         = bestResult.getBestTarget().getPoseAmbiguity();
      double               currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList)
      {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0)
        {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is there.
     */
    public Optional<PhotonPipelineResult> getLatestResult()
    {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose()
    {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp.
     */
    private void updateUnreadResults()
    {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      
      for (PhotonPipelineResult result : resultsList)
      {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }

        resultsList = Robot.isReal() ? photonCamera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
        if (!resultsList.isEmpty())
        {
          updateEstimatedGlobalPose();
        }

    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should only be called once
     * per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets used for
     * estimation.
     */
    private void updateEstimatedGlobalPose()
    {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList)
      {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets)
    {
      if (estimatedPose.isEmpty())
      {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else
      {
        // Pose present. Start running Heuristic
        var    estStdDevs = singleTagStdDevs;
        int    numTags    = 0;
        double avgDist    = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets)
        {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty())
          {
            continue;
          }
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0)
        {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else
        {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1)
          {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
          {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else
          {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }
}