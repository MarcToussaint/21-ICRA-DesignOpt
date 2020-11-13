Wrench 	{  X:[0, 0, 0, 0.707107, 0.707107, 0, 0] }
Wrench_03 (Wrench) 	{ Q:[0.011, 0, -0.112, 0, 0.707107, 0, -0.707107], shape:mesh, mesh:'meshes/Wrench_03.ply', meshscale:.25 }
Wrench_04 (Wrench) 	{ Q:[0, 0, 0.05, -0.5, 0.5, -0.5, -0.5], shape:mesh, mesh:'meshes/Wrench_04.ply', meshscale:.25 }

Wrench_joint(Wrench) { Q:[-0.02, 0, -0.125, -0.707107, 0, -0.707107, 0] }
Wrench_02 (Wrench_joint) 	{ joint:transZ, shape:mesh, mesh:'meshes/Wrench_02.ply', limits:[-.03 .01], meshscale:.25 }

