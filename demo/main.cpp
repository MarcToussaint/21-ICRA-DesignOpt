#include <Core/util.h>

#include "test.h" // basic tests
#include "eval-sorting.h"
#include "eval-wrench.h"

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc, argv);

  //-- initial tests
//  testGlobalParameters();
//  testMorphoOptim();
//  testDeformation();
//  renderDeformed(); return 0;
//  return 0;

  uint seed = rai::getParameter<double>("seed");
  uint mode = rai::getParameter<double>("mode");
  uint numCases = rai::getParameter<double>("numCases");

  rai::String scenario = rai::getParameter<rai::String>("scenario");

  rnd.seed(seed);
  rai::String prefix = STRING("dat." <<mode <<'.' <<seed <<'/');
  rai::system(STRING("mkdir -p " <<prefix));

  //-- evaluations
  if(scenario=="resorting"){
    generateResortingCases(numCases, STRING(prefix <<"cases"), false);
//    playCases();
    evaluateResorting("resorting.g", STRING(prefix <<"cases"), EvalMode(mode), prefix);
    if(mode>0)
      renderDeformed(prefix, true);
  }

  else if(scenario=="wrenchTool"){
    generateWrenchToolCases(numCases, STRING(prefix <<"cases"), false);
//    playCases2();
    evaluateWrenchTool("evalWrenchTool.g", STRING(prefix <<"cases"), EvalMode(mode), prefix);
    if(mode>0)
      renderDeformed(prefix, true);
  }

  else HALT("which scenario?")

  cout <<"--bye bye--" <<endl;
  return 0;
}
