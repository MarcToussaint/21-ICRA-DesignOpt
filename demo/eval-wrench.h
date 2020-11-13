#pragma once

#include "test.h"

#include <Core/util.h>

void generateWrenchToolCases(uint K, const char* animFile, bool view);
void playCases2();
void evaluateWrenchTool(const char* modelFile, const char* animFile, EvalMode mode, const rai::String& pathPrefix="z.");
