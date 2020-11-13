#include <Kin/viewer.h>
#include <Kin/forceExchange.h>
#include <Gui/opengl.h>

#include <KOMO/komo.h>

//===========================================================================

void testGlobalParameters() {

  rai::Configuration C("model.g");
  C.sortFrames();

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "initial model", false);

  KOMO komo;

  komo.setModel(C, false);
  komo.setTiming(1., 20, 5., 2);
  komo.add_qControlObjective({}, 2);

//  animateConfiguration(komo.pathConfig);

  //trivial task:
  komo.addObjective({1.}, FS_positionDiff, {"gripper", "box"}, OT_eq, {1e2});
  komo.verbose = 4;
  komo.optimize();

  //display result
  V.setPath(komo.getPath_frames(), "optimized motion", true);
  while(V.playVideo(true));

//  animateConfiguration(komo.pathConfig);
}

//===========================================================================

void testMorphoOptim() {
  rai::Configuration C("model.g");
  C.sortFrames();
  C.optimizeTree();

  //add new force DOFs
//  new rai::ForceExchange(*C["panda_joint1"], *C["panda_joint2"]);
//  new rai::ForceExchange(*C["panda_joint2"], *C["panda_joint3"]);
//  new rai::ForceExchange(*C["panda_joint3"], *C["panda_joint4"]);
//  new rai::ForceExchange(*C["panda_joint4"], *C["panda_joint5"]);
//  new rai::ForceExchange(*C["panda_joint5"], *C["panda_joint6"]);

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "initial model", false);

  KOMO komo;
  double len=4.;

  komo.setModel(C, false);
  komo.setTiming(len, 15, 5., 2);

  double sc = 1e1;

  //** global objectives: norms, limits, forces
  komo.add_qControlObjective({}, 2);
  komo.addObjective({}, FS_qQuaternionNorms, {}, OT_sos, {sc});
  //deformation limits
  komo.addObjective({}, FS_jointLimits, {"pandaPosition", "deform1", "deform2", "deform3", "deform4", "deform5", "deform6"}, OT_ineq, {sc});

  //grasp
  komo.addSwitch_stable(1., len-1., "gripper", "box");
  komo.addObjective({1.}, FS_positionDiff, {"gripper", "box"}, OT_eq, {sc});
  komo.addObjective({1.}, FS_scalarProductXX, {"gripper", "box"}, OT_eq, {sc}, {0.});
  komo.addObjective({1.}, FS_vectorZ, {"gripper"}, OT_eq, {sc}, {0., 0., 1.});

  //slow - down - up
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {sc}, {}, 1);
  komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_sos, {sc}, {0.,0.,.1}, 2);

  //place
  komo.addSwitch_stable(len-1., -1., "table", "box");
//  komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, arr({1,3},{0,0,1e2}), {0,0,.1});
  komo.addObjective({len-1.}, FS_positionDiff, {"box", "target"}, OT_eq, {sc});
  komo.addObjective({len-1.}, FS_vectorZ, {"gripper"}, OT_eq, {sc}, {0., 0., 1.});

  //slow - down - up
  komo.addObjective({len-1.}, FS_qItself, {}, OT_eq, {sc}, {}, 1);
  komo.addObjective({len-1.1,len-.9}, FS_position, {"gripper"}, OT_sos, {sc}, {0.,0.,.1}, 2);


  //forces
//  komo.addObjective({}, make_feature<F_netForce>({"panda_joint2"}, C), OT_eq);
//  komo.addObjective({}, make_feature<F_netForce>({"panda_joint3"}, C), OT_eq);
//  komo.addObjective({}, make_feature<F_netForce>({"panda_joint4"}, C), OT_eq);
//  komo.addObjective({}, make_feature<F_netForce>({"panda_joint5"}, C), OT_eq);
//  komo.addObjective({}, make_feature<F_netForce>({"panda_joint6"}, C), OT_eq);

//  komo.addObjective({}, make_feature<F_HingeXTorque>({"panda_joint1", "panda_joint2"}, C), OT_sos);
//  komo.addObjective({}, make_feature<F_HingeXTorque>({"panda_joint2", "panda_joint3"}, C), OT_sos);
//  komo.addObjective({}, make_feature<F_HingeXTorque>({"panda_joint3", "panda_joint4"}, C), OT_sos);
//  komo.addObjective({}, make_feature<F_HingeXTorque>({"panda_joint4", "panda_joint5"}, C), OT_sos);
//  komo.addObjective({}, make_feature<F_HingeXTorque>({"panda_joint5", "panda_joint6"}, C), OT_sos);

  komo.verbose = 4;

  listWrite(komo.pathConfig.forces, cout, "\n## ");

  komo.optimize();
//  komo.checkGradients();

  listWrite(komo.pathConfig.forces, cout, "\n## ");

  C.setFrameState(komo.getFrameState(0));
  C.writeCollada("z.opt.dae");
  FILE("z.opt.g") <<C;
  FILE("z.opt.path") <<komo.getPath_frames();

  V.setConfiguration(komo.pathConfig, "pathConfig", true);
//  V.setConfiguration(C, "initial model", false);
//  V.setPath(komo.getPath_frames(), "optimized motion", true);
  while(V.playVideo(C.frames.N, true, 3.));
}

//===========================================================================

void deformMesh(rai::Mesh& mesh, const rai::Transformation& deform, const arr& p0, const arr& p1){
  arr norm = p1-p0;
  norm /= sumOfSqr(norm);
  rai::Transformation t;
  arr d;
  for(uint i=0;i<mesh.V.d0;i++){
    arr v = mesh.V[i];
    d = v-p0;
    double s = scalarProduct(d, norm);

    t=deform;
    t.pos *= s;
    t.rot.multiply(s);
    t.applyOnPoint(v);
  }
}

void deformMeshOfFrame(rai::Frame *f){
  //find the previous joint frame
  rai::Frame *fstart = f->parent->getUpwardLink();
  rai::Transformation Q = f->parent->get_X() / fstart->get_X();

  //find the first child with mesh -> to be deformed
  rai::Frame *fmesh = 0;
  for(rai::Frame *ch:fstart->children) if(ch->shape){ fmesh = ch; break; }
  rai::Mesh& mesh = fmesh->shape->mesh();

  rai::Transformation deform = f->get_Q(); // inverse deformation
  deform.rot.normalize();
  rai::Transformation meshT = fmesh->get_Q();
  deform = (-meshT) * Q * deform * (-Q) * meshT;
  arr start = ((fmesh->get_X() / fstart ->get_X().pos)).getArr(); // -> full inverse deformation
  arr end = ((fmesh->get_X() / f->parent->get_X().pos)).getArr(); // -> no deformation
  deformMesh(mesh, deform, start, end);
}

void testDeformation(){
  rai::Configuration C("model.g");

#if 0
  rai::Frame *f = C["panda_link1_1"];

  rai::Mesh& mesh = f->shape->mesh();


  OpenGL gl;
  gl.add(glStandardLight);
  gl.add(glStandardOriginAxes);
  gl.add(mesh);
  gl.watch();

  rai::Transformation deform, t;
  deform.setRandom();
  double s = .1;
  deform.pos *= s;
  deform.rot *= s;

  deformMesh(mesh, deform, {0.,0.,-.3}, {0.,0.,0.});

  gl.watch();
#endif

  rai::Frame *f = C["deform2"];

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "initial", true);

  arr q = C.getJointState({f});
  q(0) += .05;
//  q(2) -= .05;
  q(5) += .2;
  q(6) += .2;
  C.setJointState(q, {f});

  V.setConfiguration(C, "deformed", true);


  deformMeshOfFrame(f);

  V.recopyMeshes(C);
  V.setConfiguration(C, "deformed", true);

}

//===========================================================================

void renderDeformed(const char* prefix, bool deform){
  rai::Configuration C(STRING(prefix<<"opt.g"));
  arr X;
  FILE(STRING(prefix<<"opt.path")) >>X;
  X.reshape(-1, C.frames.N, 7);

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "initial", true);
//  V.drawFrameLines=false;
//  V.gl->resize(600,600);
//  V.gl->camera.setPosition(-2., 7., 3.);
//  V.gl->camera.focus(0.,0.,1.);
//  V.gl->camera.upright();

  if(deform){
    for(uint i=1;;i++){
      rai::Frame *f = C[STRING("deform"<<i)];
      if(!f) break;
      if(f->joint && f->joint->dim)
        deformMeshOfFrame(f);
    }
    V.recopyMeshes(C);
    V.setConfiguration(C, "deformed", true);
  }

  V.setPath(X);
  while(V.playVideo(true, 5., "z.vid/"));

  C.writeCollada("z.dae", "collada");
  C.writeCollada("z.fbx", "fbx");
}

