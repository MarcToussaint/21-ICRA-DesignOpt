#include "eval.h"

#include <Kin/kin.h>
#include <Kin/animation.h>
#include <Kin/frame.h>
#include <Kin/forceExchange.h>
#include <Kin/F_forces.h>

#include <KOMO/komo.h>


void evaluateScrew(const char* modelFile, EvalMode mode, const rai::String& pathPrefix="z."){
  StringA deforms = {"deform_mount0", "deform1", "deform2", "deform3", "deform4", "deform5", "deform6", "deform7"};
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

  //-- setup komo
  KOMO komo;
  komo.setModel(C, false);
  double len=4.;
  komo.setTiming(len, 25, 5., 2);

  double sc = 1e1;

  //-- global control costs (continuously going through all cases)
  komo.add_qControlObjective({}, 2);
  komo.add_qControlObjective({}, 1, 0.2);

  //-- global objectives: norms, limits, forces
  komo.addObjective({}, FS_qQuaternionNorms, {}, OT_eq, {1e2});
  //deformation limits
  komo.addObjective({}, FS_jointLimits, deforms, OT_ineq, {sc});

  //-- forces
  for(uint i=1;i<joints.N;i++)  komo.addObjective({}, make_shared<F_ObjectTotalForce>(), {joints(i)}, OT_eq);
  if(mode==evalTorques){
    for(uint i=1;i<joints.N;i++)  komo.addObjective({}, make_shared<F_HingeXTorque>(), {joints(i-1), joints(i)}, OT_sos, {1e-1});
  }

  //-- position wrench
  komo.addSwitch_mode(SY_none, SY_stable, 1., 3., NULL, "wrenchTip", "bolt");
  komo.addSwitch_mode(SY_none, SY_stable, 3., -1., NULL, "world", "bolt");
  komo.addObjective({1.}, FS_scalarProductXY, {"wrenchTip", "boltTip"}, OT_eq, {sc});
  komo.addObjective({1., 3.}, FS_positionDiff, {"wrenchTip", "boltTip"}, OT_eq, {sc});
  komo.addObjective({1., 3.}, FS_vectorZDiff, {"wrenchTip", "boltTip"}, OT_eq, {sc});

  komo.addObjective({.7,1.}, FS_positionRel, {"boltTip", "wrenchTip"}, OT_sos, {sc}, {0.,.05,0}, 2);
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {sc}, {}, 1);

  komo.addObjective({3.}, FS_qItself, {}, OT_eq, {sc}, {}, 1);
  komo.addObjective({3.,3.3}, FS_positionRel, {"boltTip", "wrenchTip"}, OT_sos, {sc}, {0.,.05,0}, 2);

//  //-- after kinematic switches are in place, set the initial object poses for each case
//  komo.retrospectApplySwitches2();
//  for(uint k=0;k<K;k++){
//    double start = len*k;
//    double end = len*(k+1);
//    int t0 = conv_time2step(start, komo.stepsPerPhase);
//    int t1 = conv_time2step(end, komo.stepsPerPhase);
//    if(!k) t0=-komo.k_order;
//    if(k==K-1) t1=komo.T;
//    for(int t=t0;t<t1;t++){
//      komo.pathConfig.setFrameState(A.A(0).X[k], komo.timeSlices[komo.k_order+t].sub(A.A(0).frameIDs));
//    }
//  }

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
  arr q = komo.getPath( framesToIndices( C.getFrames(joints) ) );
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
  V.setConfiguration(komo.pathConfig, "optimized", true);
  while(V.playVideo(C.frames.N, true, 2.*len));

  //  komo.view(true, "optimized");
  //  while(komo.view_play(.1, true, "optimized"));

}
