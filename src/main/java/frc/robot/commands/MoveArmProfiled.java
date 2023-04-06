// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.subsystems.ArmGripper;
// import frc.robot.utils.BeaverLogger;

public class MoveArmProfiled extends CommandBase {
  double[][] currentPath;
  double UpperError, UpperArm_P;
  double LowerError, LowerArm_P;
  ArmGripper m_armGripper;
  HashMap<String,double[][]> paths = new HashMap<String,double[][]>();
  int i = 0;
  double startTime;
  double jointErrorTolerance = Math.sqrt(50+Math.pow(3*Tolerances.EXTENSION_LENGTH,2)); // 5 Degrees each way
  boolean isReverse = false;
  String currProfile;

  public void createMap(){
    paths.put("LongThrowMid", new double[][]{{104.6,21.09000000000001,0.0},
      {104.55465357828778,22.123830386726777,0.0},
      {104.42968490717931,25.016554159876087,0.0},
      {104.24655879877662,29.45016131784148,0.0},
      {104.0269242740065,35.1064576501918,0.0},
      {103.78978299250845,41.66989630778309,0.0},
      {103.55113998390166,48.82792707149196,0.0},
      {103.32522138670555,56.2697786132945,0.0},
      {103.127014988728,63.683917955878435,0.0},
      {102.9769462107873,70.75337448892111,0.0},
      {102.61134083371442,77.1452772420874,0.0},
      {101.81240351436242,82.4808763690195,0.0},
      {101.5147767947738,86.22898413817083,0.0},
      {101.16870314197386,87.34281289301157,0.0},
      {100.64298677151268,90.85701322848735,0.0},
      {99.5,93.99999999999999,0.0},
      {96.16269579954607,97.48016134331105,0.0},
      {92.1119805843657,101.93087655849142,0.0},
      {88.35620596272244,106.30093689442043,0.0},
      {84.67620776469111,110.76664937816604,0.0},
      {81.24662571216858,115.11051714497427,0.0},
      {78.03995458995445,119.31718826718841,0.0},
      {75.07650515594196,123.32349484405803,0.0},
      {72.37039801244245,127.07245913041471,0.0},
      {69.94613909373992,130.49671804911722,0.0},
      {67.83772299823622,133.51941985890667,0.0},
      {66.08942786865668,136.05342927420045,0.0},
      {64.75573571530187,138.00140714184099,0.0},
      {63.90103108877357,139.2561117683693,0.0},
      {63.599444373251984,139.700555626748,0.0}});
    
    paths.put("ShortThrowMidPrep", new double[][]{{104.6,21.09000000000001,0.0},
      {104.59125364431486,22.094518950437347,0.0},
      {104.56676384839649,24.907172011661814,0.0},
      {104.52915451895043,29.226603498542268,0.0},
      {104.48104956268222,34.751457725947525,0.0},
      {104.42507288629739,41.18037900874635,0.0},
      {104.36384839650145,48.21201166180757,0.0},
      {104.3,55.54500000000001,0.0},
      {104.23615160349854,62.87798833819244,0.0},
      {104.17492711370262,69.90962099125365,0.0},
      {104.11895043731779,76.33854227405249,0.0},
      {104.07084548104957,81.86339650145771,0.0},
      {104.0332361516035,86.18282798833819,0.0},
      {104.00874635568515,88.99548104956267,0.0},
      {104.0,90.0,0.0},
    }
      );
    paths.put("ReachMid", new double[][]{{104.0,90.0,0.0},
    {102.62825788751715,91.81755829903977,0.0},
    {98.9519890260631,96.6886145404664,0.0},
    {93.62962962962963,103.74074074074073,0.0},
    {87.31961591220852,112.10150891632372,0.0},
    {80.6803840877915,120.89849108367628,0.0},
    {74.37037037037037,129.25925925925927,0.0},
    {69.04801097393691,136.3113854595336,0.0},
    {65.37174211248285,141.18244170096023,0.0},
  {64.0, 143.0,0.0}}); // 143
    paths.put("PickToStow", new double[][]{{98.9,91.3,0.0},
    {99.0954732510288,88.89224965706447,0.0},
    {99.61934156378601,82.43947873799725,0.0},
    {100.37777777777777,73.09740740740742,0.0},
    {101.27695473251029,62.02175582990397,0.0},
    {102.22304526748972,50.36824417009601,0.0},
    {103.12222222222222,39.2925925925926,0.0},
    {103.88065843621399,29.950521262002766,0.0},
    {104.4045267489712,23.49775034293552,0.0}});
    paths.put("LongThrowPickup", new double[][]{
      {104.6,21.09000000000001,0.0},
      {104.5767071759259,21.500713252314835,0.0},
      {104.50949074074073,22.685914351851853,0.0},
      {104.40234375,24.57519531250001,0.0},
      {104.25925925925925,27.098148148148148,0.0},
      {104.08423032407407,30.184364872685183,0.0},
      {103.88125000000001,33.76343750000003,0.0},
      {103.65431134259258,37.76495804398148,0.0},
      {103.40740740740739,42.11851851851852,0.0},
      {103.14453125,46.753710937499996,0.0},
      {102.86967592592592,51.60012731481483,0.0},
      {102.58683449074074,56.58735966435185,0.0},
      {102.3,61.64500000000001,0.0},
      {102.01316550925925,66.70264033564817,0.0},
      {101.73032407407406,71.68987268518518,0.0},
      {101.45546875000001,76.5362890625,0.0},
      {101.19259259259259,81.17148148148149,0.0},
      {100.94568865740742,85.52504195601851,0.0},
      {100.71875,89.52656250000001,0.0},
      {100.51576967592592,93.10563512731481,0.0},
      {100.34074074074073,96.19185185185187,0.0},
      {100.19765625000001,98.7148046875,0.0},
      {100.09050925925926,100.60408564814814,0.0},
      {100.02329282407408,101.78928674768521,0.0},
      {100.0,102.2,0.0},
  });
    paths.put("PickupToHigh", new double[][]{
      {102.4,100.4,0.016},
      {102.34453841511615,100.47875545053508,0.01649792178429056},
      {102.18121360997544,100.71067667383488,0.01796421559044276},
      {101.91461550884412,101.08924597744138,0.020357674098377375},
      {101.54933403598841,101.60794566889646,0.02363708998801519},
      {101.08995911567459,102.26025805574209,0.027761255939277},
      {100.5410806721689,103.03966544552016,0.03268896463208357},
      {99.90728862973761,103.93965014577259,0.03837900874635568},
      {99.19317291264694,104.95369446404135,0.04479018096201412},
      {98.40332344516317,106.07528070786833,0.05188127395897967},
      {97.5423301515525,107.29789118479545,0.059611080417173105},
      {96.61478295608123,108.61500820236466,0.06793839301651522},
      {95.62527178301559,110.02011406811786,0.07682200443692679},
      {94.57838655662182,111.50669108959701,0.08622070735832857},
      {93.47871720116618,113.06822157434404,0.0960932944606414},
      {92.33085364091494,114.69818782990083,0.10639855842378598},
      {91.1393858001343,116.3900721638093,0.11709529192768316},
      {89.90890360309056,118.13735688361143,0.12814228765225372},
      {88.64399697404993,119.93352429684911,0.13949833827741837},
      {87.34925583727868,121.7720567110643,0.15112223648309803},
      {86.02927011704308,123.64643643379885,0.16297277494921328},
      {84.68862973760933,125.55014577259476,0.1750087463556851},
      {83.33192462324372,127.47666703499392,0.18718894338243414},
      {81.96374469821248,129.41948252853828,0.19947215870938126},
      {80.58867988678186,131.37207456076976,0.21181718501644722},
      {79.21132011321814,133.32792543923026,0.22418281498355278},
      {77.83625530178753,135.28051747146174,0.23652784129061863},
      {76.46807537675629,137.22333296500608,0.24881105661756572},
      {75.11137026239068,139.14985422740526,0.2609912536443149},
      {73.77072988295694,141.05356356620118,0.27302722505078664},
      {72.4507441627213,142.92794328893572,0.2848777635169019},
      {71.1560030259501,144.76647570315092,0.2965016617225815},
      {69.89109639690948,146.56264311638856,0.3078577123477461},
      {68.66061419986572,148.3099278361907,0.3189047080723168},
      {67.46914635908507,150.00181217009919,0.32960144157621396},
      {66.32128279883382,151.631778425656,0.3399067055393586},
      {65.22161344337819,153.19330891040298,0.3497792926416713},
      {64.17472821698443,154.67988593188215,0.35917799556307317},
      {63.18521704391877,156.08499179763535,0.36806160698348483},
      {62.257669848447506,157.40210881520454,0.3763889195828268},
      {61.39667655483686,158.62471929213166,0.38411872604102026},
      {60.60682708735306,159.74630553595867,0.3912098190379859},
      {59.8927113702624,160.7603498542274,0.3976209912536443},
      {59.258919327831094,161.66033455447985,0.4033110353679164},
      {58.71004088432541,162.43974194425792,0.4082387440607231},
      {58.25066596401162,163.09205433110353,0.41236291001198466},
      {57.88538449115591,163.6107540225586,0.4156423259016226},
      {57.61878639002456,163.98932332616513,0.4136423259016226},
      {57.45546158488384,164.22124454946493,0.41250207821570945},
      {57.40000000000002,164.29999999999998,0.40}
        });
    paths.put("ExtendToPickup", new double[][]{
      {100.0,102.2,0.0},
      {99.99999999999999,102.18885995370371,0.0002025462962962963},
      {100.0,102.15671296296297,0.000787037037037037},
      {100.0,102.10546875,0.00171875},
      {99.99999999999999,102.03703703703704,0.002962962962962963},
      {100.00000000000001,101.95332754629631,0.004484953703703704},
      {100.00000000000001,101.85625,0.00625},
      {100.0,101.74771412037038,0.008223379629629629},
      {100.0,101.62962962962963,0.01037037037037037},
      {100.00000000000001,101.50390625,0.01265625},
      {100.0,101.37245370370371,0.015046296296296295},
      {100.00000000000001,101.23718171296296,0.017505787037037035},
      {100.0,101.1,0.02},
      {100.0,100.96281828703705,0.022494212962962966},
      {100.0,100.8275462962963,0.0249537037037037},
      {100.00000000000001,100.69609375,0.027343750000000003},
      {99.99999999999999,100.57037037037037,0.029629629629629634},
      {100.0,100.45228587962964,0.031776620370370365},
      {100.00000000000001,100.34375,0.03375},
      {100.0,100.2466724537037,0.03551504629629629},
      {100.0,100.16296296296296,0.03703703703703704},
      {100.0,100.09453125,0.03828125},
      {100.0,100.04328703703703,0.039212962962962956},
      {100.0,100.01114004629629,0.03979745370370372},
      {100.0,100.0,0.04}
    });
    paths.put("PickupPullback", new double[][]{
      {100.0,102.2,0.05},
      {100.12659143518518,102.13822337962964,0.04974681712962963},
      {100.49189814814815,101.95995370370372,0.04901620370370371},
      {101.07421875,101.67578125,0.04785156250000001},
      {101.85185185185185,101.29629629629629,0.0462962962962963},
      {102.80309606481482,100.83208912037037,0.04439380787037037},
      {103.90625,100.29375,0.0421875},
      {105.13961226851852,99.69186921296296,0.039720775462962966},
      {106.48148148148148,99.03703703703704,0.037037037037037035},
      {107.91015625,98.33984375,0.0341796875},
      {109.40393518518519,97.61087962962964,0.03119212962962963},
      {110.94111689814817,96.8607349537037,0.028117766203703713},
      {112.5,96.10000000000001,0.025},
      {114.05888310185186,95.3392650462963,0.021882233796296297},
      {115.59606481481482,94.58912037037038,0.01880787037037038},
      {117.08984375000001,93.86015625,0.015820312500000003},
      {118.51851851851853,93.16296296296296,0.012962962962962959},
      {119.86038773148148,92.50813078703705,0.010279224537037042},
      {121.09375000000001,91.90625,0.0078125},
      {122.19690393518519,91.36791087962963,0.00560619212962964},
      {123.14814814814815,90.9037037037037,0.003703703703703698},
      {123.92578125,90.52421875,0.0021484375},
      {124.50810185185186,90.24004629629628,0.000983796296296302},
      {124.87340856481484,90.06177662037037,0.0002531828703703609},
      {125.00000000000001,90.0,0.0}
    });
    paths.put("LongThrowHigh", new double[][]{
      {104.6,21.09000000000001,0.0},
{104.60708912037036,21.51489149305558,0.0},
{104.6275462962963,22.74100694444446,0.0},
{104.66015624999999,24.695507812500033,0.0},
{104.7037037037037,27.305555555555564,0.0},
{104.75697337962963,30.498311631944436,0.0},
{104.81875,34.200937500000016,0.0},
{104.88781828703704,38.340594618055555,0.0},
{104.96296296296295,42.84444444444445,0.0},
{105.04296875,47.6396484375,0.0},
{105.12662037037036,52.65336805555555,0.0},
{105.2127025462963,57.81276475694444,0.0},
{105.29999999999998,63.044999999999995,0.0},
{105.38729745370371,68.27723524305556,0.0},
{105.47337962962963,73.43663194444443,0.0},
{105.55703125,78.45035156249999,0.0},
{105.63703703703703,83.24555555555555,0.0},
{105.71218171296296,87.74940538194444,0.0},
{105.78124999999999,91.88906250000001,0.0},
{105.84302662037037,95.59168836805554,0.0},
{105.89629629629628,98.78444444444445,0.0},
{105.93984375,101.3944921875,0.0},
{105.97245370370369,103.34899305555555,0.0},
{105.99291087962962,104.57510850694446,0.0},
{106.0,105.0,0.0},
{106.0,105.0,0.0},
{105.84302662037035,105.17722800925925,0.0},
{105.39004629629629,105.6886574074074,0.0},
{104.66796875,106.50390625,0.0},
{103.7037037037037,107.59259259259258,0.0},
{102.52416087962963,108.92433449074072,0.0},
{101.15625,110.46874999999999,0.0},
{99.62688078703704,112.19545717592592,0.0},
{97.96296296296295,114.07407407407406,0.0},
{96.19140625,116.07421875,0.0},
{94.33912037037037,118.16550925925925,0.0},
{92.4330150462963,120.3175636574074,0.0},
{90.5,122.50000000000001,0.0},
{88.56698495370371,124.6824363425926,0.0},
{86.66087962962965,126.83449074074075,0.0},
{84.80859375000001,128.92578125,0.0},
{83.03703703703704,130.92592592592592,0.0},
{81.37311921296298,132.80454282407408,0.0},
{79.84375,134.53124999999997,0.0},
{78.47583912037037,136.07566550925924,0.0},
{77.2962962962963,137.40740740740742,0.0},
{76.33203125,138.49609375000003,0.0},
{75.60995370370371,139.3113425925926,0.0},
{75.15697337962963,139.82277199074073,0.0},
{75.0,140.0,0.0},
{75.0,140.0,0.0},
{74.91087962962963,140.123046875,0.0022280092592592594},
{74.6537037037037,140.478125,0.008657407407407407},
{74.24375,141.044140625,0.01890625},
{73.6962962962963,141.79999999999998,0.03259259259259259},
{73.02662037037038,142.724609375,0.049334490740740734},
{72.25,143.796875,0.06875},
{71.38171296296296,144.995703125,0.0904571759259259},
{70.43703703703703,146.3,0.11407407407407408},
{69.43125,147.68867187499998,0.13921875},
{68.37962962962963,149.140625,0.16550925925925924},
{67.29745370370372,150.634765625,0.19256365740740738},
{66.2,152.15,0.22},
{65.1025462962963,153.665234375,0.24743634259259262},
{64.02037037037037,155.159375,0.2744907407407407},
{62.96875000000001,156.611328125,0.30078125},
{61.962962962962976,158.0,0.32592592592592595},
{61.01828703703704,159.304296875,0.34954282407407405},
{60.15,160.503125,0.37125},
{59.37337962962964,161.575390625,0.39066550925925914},
{58.7037037037037,162.50000000000003,0.4074074074074075},
{58.15625,163.255859375,0.42109375},
{57.74629629629631,163.821875,0.4313425925925925},
{57.48912037037037,164.17695312500004,0.4377719907407408},
{57.400000000000006,164.3,0.44}
    });

    
  }

  public double[] getNearestSetpoint(double dt) {
    double predicted_lower = (m_armGripper.getLowerAngleRelative()+(dt*m_armGripper.getLowerJointAngularVelocity()));
    double predicted_upper = (m_armGripper.getUpperAngleRelative()+(dt*m_armGripper.getUpperJointAngularVelocity()));
    double predicted_extension = (m_armGripper.getExtensionDistance()+(dt*m_armGripper.getExtensionVelocity()));
    double curr_lowerError = Math.abs(predicted_lower-currentPath[i][0]);
    double curr_upperError = Math.abs(predicted_upper-currentPath[i][1]);
    double curr_extensionError = Math.abs(predicted_extension-currentPath[i][2]);
    // Note: i>0 always since we do i++ in initialize.
    double curr_to_prev_jointError = Math.sqrt(Math.pow(curr_lowerError,2)+Math.pow(curr_upperError,2)+Math.pow(curr_extensionError, 2));
    if(i+1<currentPath.length-1){
      double next_lowerError = Math.abs(predicted_lower-currentPath[i+1][0]); 
      double next_upperError = Math.abs(predicted_upper-currentPath[i+1][1]); 
      double next_extensionError = Math.abs(predicted_extension-currentPath[i+1][2]);
      double curr_to_next_jointError = Math.sqrt(Math.pow(next_lowerError,2)+Math.pow(next_upperError,2)+Math.pow(next_extensionError, 2));
      if(curr_to_next_jointError<curr_to_prev_jointError){
        i++;
        return getNearestSetpoint(dt);
      }
    }
    return currentPath[i];

  }
  /** Moves arm to given setpoint. Finishes once within tolerance */
  public MoveArmProfiled(ArmGripper armGripper, String path, boolean isReversed) {
    createMap();
    m_armGripper = armGripper;
    currentPath = paths.getOrDefault(path, new double[][]{{0.0,0.0,0.0},{0.0,0.0,0.0}});
    addRequirements(armGripper);
    startTime = Timer.getFPGATimestamp();
    this.currProfile = path;
    this.isReverse = isReversed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i=0;
    if(isReverse){
      m_armGripper.setLowerTargetAngle(currentPath[currentPath.length-1][0]);
      m_armGripper.setUpperTargetAngle(currentPath[currentPath.length-1][1]);
      m_armGripper.setExtensionTargetLength(currentPath[currentPath.length-1][2]);

    }else{
      m_armGripper.setLowerTargetAngle(currentPath[i][0]);
      m_armGripper.setUpperTargetAngle(currentPath[i][1]);
      m_armGripper.setExtensionTargetLength(currentPath[i][2]);

    }
    startTime = Timer.getFPGATimestamp();
  }
  public int getReverseIndex(int i){
    return (currentPath.length-1)-i;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance){
    //   i++;
    // }
    // if(currProfile == "ShortThrowMid" || currProfile == "LongThrowHighHD"){
    //   i++;
    // }else{
    //   getNearestSetpoint(Timer.getFPGATimestamp()-prevLoop);
    // }
    i=(int) Math.ceil(Timer.getFPGATimestamp()-startTime)*50; // 50 loops per second = 0.02 seconds per loop
    // System.out.println("LOOP");
    // System.out.println(i);

    if(i <= currentPath.length-1){
      if(isReverse){
        m_armGripper.setLowerTargetAngle(currentPath[getReverseIndex(i)][0]);
        m_armGripper.setUpperTargetAngle(currentPath[getReverseIndex(i)][1]);
        m_armGripper.setExtensionTargetLength(currentPath[getReverseIndex(i)][2]);
      }else{
        m_armGripper.setLowerTargetAngle(currentPath[i][0]);
        m_armGripper.setUpperTargetAngle(currentPath[i][1]);
        m_armGripper.setExtensionTargetLength(currentPath[i][2]);

      }
      
      // BeaverLogger.getInstance().logArm(currentPath[i], m_armGripper);
    }else{
      i = currentPath.length-1;
      if(isReverse){
        m_armGripper.setLowerTargetAngle(currentPath[0][0]);
        m_armGripper.setUpperTargetAngle(currentPath[0][1]);
        m_armGripper.setExtensionTargetLength(currentPath[0][2]);
      }else{
        m_armGripper.setLowerTargetAngle(currentPath[i][0]);
        m_armGripper.setUpperTargetAngle(currentPath[i][1]);
        m_armGripper.setExtensionTargetLength(currentPath[i][2]);
      }
      // BeaverLogger.getInstance().logArm(currentPath[i], m_armGripper);
    }
    

    // SmartDashboard.putNumber("lowerSetp", currentPath[i][0]);
    // SmartDashboard.putNumber("upperSetp", currentPath[i][1]);
    // SmartDashboard.putNumber("extSetp", currentPath[i][2]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("PROFILED ARM TO MID FINISHED");
      if(isReverse){
        m_armGripper.setLowerTargetAngle(currentPath[0][0]);
        m_armGripper.setUpperTargetAngle(currentPath[0][1]);
        m_armGripper.setExtensionTargetLength(currentPath[0][2]);
      }else{
        m_armGripper.setLowerTargetAngle(currentPath[currentPath.length-1][0]);
        m_armGripper.setUpperTargetAngle(currentPath[currentPath.length-1][1]);
        m_armGripper.setExtensionTargetLength(currentPath[currentPath.length-1][2]);
      }
      
      // BeaverLogger.getInstance().logArm(currentPath[currentPath.length-1], m_armGripper);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return i>lowerSetpoint.length-1 // try this if it finishes too early
    
    boolean isLowerInTolerance = Math.abs(m_armGripper.getLowerAngleRelative()-currentPath[currentPath.length-1][0])<Tolerances.LOWER_ANGLE;
    boolean isUpperInTolerance = Math.abs(m_armGripper.getUpperAngleRelative()-currentPath[currentPath.length-1][1])<Tolerances.UPPER_ANGLE;
    boolean isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-currentPath[currentPath.length-1][2])<Tolerances.EXTENSION_LENGTH;
    if(isReverse){
      isLowerInTolerance = Math.abs(m_armGripper.getLowerAngleRelative()-currentPath[0][0])<Tolerances.LOWER_ANGLE;
      isUpperInTolerance = Math.abs(m_armGripper.getUpperAngleRelative()-currentPath[0][1])<Tolerances.UPPER_ANGLE;
      isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-currentPath[0][2])<Tolerances.EXTENSION_LENGTH;
    }
    return i>=currentPath.length-1 && (isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance);
  }
}
