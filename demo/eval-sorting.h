#pragma once

#include "test.h"

#include <Core/util.h>

void generateResortingCases(uint K, const char* animFile, bool view);
void playCases();
void evaluateResorting(const char* modelFile, const char* animFile, EvalMode mode, const rai::String& pathPrefix="z.");
