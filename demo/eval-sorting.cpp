#include "eval-sorting.h"

#include <Kin/kin.h>
#include <Kin/animation.h>
#include <Kin/frame.h>
#include <Kin/forceExchange.h>
#include <Kin/F_forces.h>

#include <KOMO/komo.h>

void randomizePose(rai::Frame* obj, FrameL regions){
  rai::Frame *region = regions.rndElem();
  arr p = region->getSize();
  p = rand(3)%p - .5*p;
  p = (region->ensure_X().rot*rai::Vector(p)).getArr();
  obj->setPosition(p + region->getPosition());

  arr q = 3.*randn(3);
  clip(q(0), -.3, .3);
  clip(q(1), -.3, .3);

  obj->set_Q()->rot.setVec(q);
}

void generateResortingCases(uint K, const char* animFile, bool view){
  rai::Configuration C;
  C.addFile("resorting.g");
  C.sortFrames();
  C.optimizeTree();

  rai::Animation A;
  A.A.resize(1);
  A.A(0).X.resize(K, 2, 7);
  A.A(0).frameIDs = framesToIndices( C.getFrames({"box", "target"}) );

  for(uint k=0;k<K;k++){
    randomizePose(C["box"], {C["startRegion"]});
    randomizePose(C["target"], C.getFrames({"targetRegion1", "targetRegion2"}));

    A.A(0).X(k,0,{}) = C["box"]->getPose();
    A.A(0).X(k,1,{}) = C["target"]->getPose();

    if(view) C.watch(true);
  }
  FILE(animFile) <<A;
}

//===========================================================================

void playCases(){
  rai::Configuration C;
  C.addFile("resorting.g");
  C.sortFrames();
  C.optimizeTree();

  rai::Animation A;
  FILE("cases") >>A;

  A.play(C, true);
}

//===========================================================================

void graspObjectives(KOMO& komo, double start, double end, double sc){
  komo.addSwitch_mode(SY_none, SY_stable, start, end, NULL, "gripper", "box");
  komo.addObjective({start}, FS_positionDiff, {"gripper", "box"}, OT_eq, {sc});
  komo.addObjective({start}, FS_vectorZDiff, {"gripper", "box"}, OT_eq, {sc});

  //slow - down - up
  komo.addObjective({start}, FS_qItself, {}, OT_eq, {sc}, {}, 1);
  komo.addObjective({start-.1,start+.1}, FS_position, {"gripper"}, OT_sos, {sc}, {0.,0.,.1}, 2);
}

void placeObjectives(KOMO& komo, double start, double end, double sc){
  komo.addSwitch_mode(SY_none, SY_stable, start, end, NULL, "world", "box");
//  komo.addObjective({2.}, FS_positionDiff, {"box", "world"}, OT_eq, arr({1,3},{0,0,1e2}), {0,0,.1});
  komo.addObjective({start}, FS_positionDiff, {"box", "target"}, OT_eq, {sc});
  komo.addObjective({start}, FS_vectorZ, {"gripper"}, OT_eq, {sc}, {0., 0., 1.});

  //slow - down - up
  komo.addObjective({start}, FS_qItself, {}, OT_eq, {sc}, {}, 1);
  komo.addObjective({start-.1,start+.1}, FS_position, {"gripper"}, OT_sos, {sc}, {0.,0.,.1}, 2);

//  komo.addSwitch(end, true, new rai::KinematicSwitch(rai::SW_noJointLink, rai::JT_none, "world", "box", komo.world));
  komo.addSwitch(end, true, rai::JT_rigid, rai::SWInit_zero, "world", "box");

}

void collisionObjectives(KOMO& komo){
  StringA collisions = {
    "box", "S_tote1",
    "box", "S_tote2",
    "box", "S_tote3",
    "box", "S_tote4",
    "box", "T1_tote1",
    "box", "T1_tote2",
    "box", "T1_tote3",
    "box", "T1_tote4",
    "box", "T2_tote1",
    "box", "T2_tote2",
    "box", "T2_tote3",
    "box", "T2_tote4",

    "suctionCup", "S_tote1",
    "suctionCup", "S_tote2",
    "suctionCup", "S_tote3",
    "suctionCup", "S_tote4",
    "suctionCup", "T1_tote1",
    "suctionCup", "T1_tote2",
    "suctionCup", "T1_tote3",
    "suctionCup", "T1_tote4",
    "suctionCup", "T2_tote1",
    "suctionCup", "T2_tote2",
    "suctionCup", "T2_tote3",
    "suctionCup", "T2_tote4"
  };

  for(uint i=0;i<collisions.N;i+=2){
    komo.addObjective({}, FS_distance, {collisions(i), collisions(i+1)}, OT_ineq, {1e0});
  }
}

//===========================================================================

void evaluateResorting(const char* modelFile, const char* animFile, EvalMode mode, const rai::String& pathPrefix){
  StringA deforms = {"deform_mount0", "deform_mount1", "deform1", "deform2", "deform3", "deform4", "deform5", "deform6"};
  StringA joints = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6"};

  //-- setup configuration
  rai::Configuration C;
  C.addFile(modelFile);
  if(mode==evalBaseline){ //baseline: remove deformations DOFs
    for(auto& fname:deforms){
      auto* f = C[fname];
      f->setJoint(rai::JT_none);
    }
  }

  //add force DOFs so that torques are computed -- only for evalTorques we'll put an objective/cost on them, otherwise they're just computed via optimization
  for(uint i=1;i<joints.N;i++)  new rai::ForceExchange(*C[joints(i-1)], *C[joints(i)], rai::FXT_torque);
  C.sortFrames();
  C.optimizeTree();

  //-- load animation (=varying pick and place poses)
  rai::Animation A;
  A.read(FILE(animFile));

  //-- setup komo
  KOMO komo;
  komo.setModel(C, false);
  uint K = A.getT();
  double len=4.;
  komo.setTiming(len*K, 15, 5., 2);

  double sc = 1e1;

  //-- global control costs (continuously going through all cases)
  komo.add_qControlObjective({}, 2);
  komo.add_qControlObjective({}, 1, 0.2);

  //-- global objectives: norms, limits, forces
  komo.addObjective({}, FS_qQuaternionNorms, {}, OT_eq, {1e2});
  //deformation limits
  komo.addObjective({}, FS_jointLimits, deforms, OT_ineq, {sc});

  //collisions
  collisionObjectives(komo);

  //-- forces
  //for(uint i=1;i<joints.N;i++)  komo.addObjective({}, make_shared<F_ObjectTotalForce>(), {joints(i)}, OT_eq);
  for(uint i=1;i<joints.N;i++)  komo.addObjective({}, make_shared<F_NewtonEuler>(), {joints(i)}, OT_eq);
  if(mode==evalTorques){
    for(uint i=1;i<joints.N;i++)  komo.addObjective({}, make_shared<F_HingeXTorque>(), {joints(i-1), joints(i)}, OT_sos, {1e-1});
  }else if(mode==evalTorquesLight){
    for(uint i=1;i<joints.N;i++)  komo.addObjective({}, make_shared<F_HingeXTorque>(), {joints(i-1), joints(i)}, OT_sos, {3e-2});
  }

  //-- task-wise objectives: pick & place
  for(uint k=0;k<K;k++){
    graspObjectives(komo, len*k+1., len*(k+1)-1., sc);
    placeObjectives(komo, len*(k+1)-1., len*(k+1)-.1, sc);
  }

  //-- after kinematic switches are in place, set the initial object poses for each case
  komo.retrospectApplySwitches2();
  for(uint k=0;k<K;k++){
    double start = len*k;
    double end = len*(k+1);
    int t0 = conv_time2step(start, komo.stepsPerPhase);
    int t1 = conv_time2step(end, komo.stepsPerPhase);
    if(!k) t0=-komo.k_order;
    if(k==K-1) t1=komo.T;
    for(int t=t0;t<t1;t++){
      komo.pathConfig.setFrameState(A.A(0).X[k], komo.timeSlices[komo.k_order+t].sub(A.A(0).frameIDs));
    }
  }

  //-- optimize
  komo.view(false, "initialization");
  komo.optimize();
  rai::Graph report = komo.getReport();

  //-- compute evaluation metrics
  double torqueCosts=0.;
  for(rai::ForceExchange* ex:komo.pathConfig.forces){
    arr jointAxis = ex->b.ensure_X().rot.getX().getArr();
    double torque = scalarProduct(jointAxis, ex->torque);
    torqueCosts += torque*torque;
  }
  double torqueCosts2=0.;
  for(rai::Node *n:report) if(n->key.startsWith("F_HingeXTorque/0")){
    torqueCosts2 += n->get<double>("sos");
  }
  double maxVel=0.;
  arr q = komo.getPath( framesToIndices( C.getFrames(joints) ) ); //exclude first
  for(uint t=1;t<q.d0;t++){
    for(uint i=0;i<q.d1;i++){
      double v = fabs(q(t,i) - q(t-1,i));
      if(v>maxVel) maxVel=v;
    }
  }
  maxVel /= komo.tau;

  rai::String output;
  output <<"torqueCosts: " <<torqueCosts;
  output <<"  torqueCosts2: " <<torqueCosts2;
  output <<"  accCosts: " <<report["F_qItself/2-#14"]->get<double>("sos");
  output <<"  velCosts: " <<report["F_qItself/1-#14"]->get<double>("sos");
  output <<"  maxVel: " <<maxVel <<endl;
  cout <<"**********************" <<endl;
  cout <<output;

  //-- save
  C.setFrameState(komo.getFrameState(0));
  if(pathPrefix(-1)=='/') rai::system(STRING("mkdir -p " <<pathPrefix));
  FILE(STRING(pathPrefix <<"opt.g")) <<C;
  FILE(STRING(pathPrefix <<"opt.path")) <<komo.getPath_frames();
  FILE(STRING(pathPrefix <<"opt.result")) <<output;

  //-- display
  rai::ConfigurationViewer V;
  V.setConfiguration(komo.pathConfig, "optimized -- press 'q' to move on!", true);
  while(V.playVideo(C.frames.N, true, K*len));
}
