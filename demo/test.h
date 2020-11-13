#pragma once

enum EvalMode{ evalBaseline, evalControls, evalTorques, evalTorquesLight };

void testGlobalParameters();
void testMorphoOptim();
void testDeformation();
void renderDeformed(const char* prefix, bool deform=true);
