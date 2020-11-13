#include "eval-wrench.h"

#include <Kin/kin.h>
#include <Kin/animation.h>
#include <Kin/frame.h>
#include <Kin/forceExchange.h>
#include <Kin/F_forces.h>

#include <KOMO/komo.h>

void randomizePose(rai::Frame* obj, rai::Frame* obj2, rai::Frame* region){

  arr q = 30.*(rand(1)-.5);
  obj->set_Q()->rot.setDeg(q(0),.0,.0,1.);
  obj2->set_Q()->rot.setDeg(q(0)+30,.0,.0,1.);

  arr p = region->getSize().sub(0,2);
  p = 0.9*rand(3)%p - .5*p;
  obj->setRelativePosition(p);
  p += arr({.013*cos(q*RAI_PI/180.)(0),
            .013*sin(q*RAI_PI/180.)(0),
            .12});
  obj2->setRelativePosition(p);

}

void generateWrenchToolCases(uint K, const char* animFile, bool view){
  rai::Configuration C;
  C.addFile("evalWrenchTool.g");
  C.sortFrames();
  C.optimizeTree();

  rai::Animation A;
  A.A.resize(1);
  A.A(0).X.resize(K, 2, 7);
  A.A(0).frameIDs = framesToIndices( C.getFrames({"bolt", "boltTip"}) );

  for(uint k=0;k<K;k++){
    randomizePose(C["bolt"], C["boltTip"], C["screwTable"]);

    A.A(0).X(k,0,{}) = C["bolt"]->getPose();
    A.A(0).X(k,1,{}) = C["boltTip"]->getPose();

    if(view) C.watch(true);
  }
  FILE(animFile) <<A;
}

//===========================================================================

void playCases2(){
  rai::Configuration C;
  C.addFile("evalWrenchTool.g");
  C.sortFrames();
  C.optimizeTree();

  rai::Animation A;
  FILE("cases") >>A;

  A.play(C, true);
}

//===========================================================================

void screwObjectives(KOMO& komo, double start, double end, double sc){
  //-- position wrench
  komo.addSwitch_mode(SY_none, SY_stable, start, end, NULL, "wrenchTip", "bolt");
  komo.addObjective({start}, FS_scalarProductXY, {"wrenchTip", "boltTip"}, OT_eq, {sc});
  komo.addObjective({start, end}, FS_positionDiff, {"wrenchTip", "boltTip"}, OT_eq, {sc});
  komo.addObjective({start, end}, FS_vectorZDiff, {"wrenchTip", "boltTip"}, OT_eq, {sc});

  komo.addObjective({start-.3,start}, FS_positionRel, {"boltTip", "wrenchTip"}, OT_sos, {sc}, {0.,.05,0}, 2);
  komo.addObjective({start}, FS_qItself, {}, OT_eq, {sc}, {}, 1);

  komo.addObjective({end}, FS_qItself, {}, OT_eq, {sc}, {}, 1);
  komo.addObjective({end}, FS_scalarProductXY, {"wrenchTip", "screwTable"}, OT_sos, {sc});
  komo.addObjective({end, end+.3}, FS_positionRel, {"wrenchTip", "screwTable"}, OT_sos, {sc}, {0.,-.05,0}, 2);
  komo.addSwitch(end, true, rai::JT_rigid, rai::SWInit_zero, "screwTable", "bolt");
}


void collisionObjectives2(KOMO& komo){
  StringA collisions = {
    "pandaTable", "screwTable",
  };

  for(uint i=0;i<collisions.N;i+=2){
    komo.addObjective({}, FS_distance, {collisions(i), collisions(i+1)}, OT_ineq, {1e0});
  }
}

//===========================================================================

void evaluateWrenchTool(const char* modelFile, const char* animFile, EvalMode mode, const rai::String& pathPrefix){
  StringA deforms = {"deform1", "deform2", "deform3", "deform4", "deform5", "deform6", "deform7", "deform8", "deform_mount8"};
  StringA joints = {"panda_link0", "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7", "wrenchTip"};

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
  //add also a force DOF between wrenchTip and bolt
  new rai::ForceExchange(*C["wrenchTip"], *C["bolt"], rai::FXT_torque);
  C.sortFrames();
  C.optimizeTree();

  //-- load animation (=varying bolt poses)
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
  collisionObjectives2(komo);

  //-- forces
  for(uint i=1;i<joints.N;i++)  komo.addObjective({}, make_shared<F_ObjectTotalForce>(), {joints(i)}, OT_eq, {1e0});
  if(mode==evalTorques){
    for(uint i=1;i<joints.N-1;i++)  komo.addObjective({}, make_shared<F_HingeXTorque>(), {joints(i-1), joints(i)}, OT_sos, {2e-1});
  }else if(mode==evalTorquesLight){
    for(uint i=1;i<joints.N-1;i++)  komo.addObjective({}, make_shared<F_HingeXTorque>(), {joints(i-1), joints(i)}, OT_sos, {5e-2});
  }

  //-- task-wise objectives: pick & place
  for(uint k=0;k<K;k++){
    screwObjectives(komo, len*k+1., len*(k+1)-1., sc);
    //fix the external wrench with eq constraints
    komo.addObjective({len*k, len*k+1.}, make_shared<F_fex_Wrench>(), {"wrenchTip", "bolt"}, OT_eq, {1e1}, {}, 0, 0, -1);
    komo.addObjective({len*k+1., len*(k+1)-1.}, make_shared<F_fex_Wrench>(), {"wrenchTip", "bolt"}, OT_eq, {1e1}, {0.,0.,0., 0.,0.,-1.}); //excert external wrench during the screw touch
    komo.addObjective({len*(k+1)-1., len*(k+1.)}, make_shared<F_fex_Wrench>(), {"wrenchTip", "bolt"}, OT_eq, {1e1}, {}, 0, +1, 0);
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
    if(!ex->b.joint || ex->b.joint->type!=rai::JT_hingeX) continue;
    arr jointAxis = ex->b.ensure_X().rot.getX().getArr();
    double torque = scalarProduct(jointAxis, ex->torque);
    torqueCosts += torque*torque;
  }
  double torqueCosts2=0.;
  for(rai::Node *n:report) if(n->key.startsWith("F_HingeXTorque/0")){
    torqueCosts2 += n->get<double>("sos");
  }
  double maxVel=0.;
  arr q = komo.getPath( framesToIndices( C.getFrames(joints({1,-2})) ) ); //exclude first and last
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
