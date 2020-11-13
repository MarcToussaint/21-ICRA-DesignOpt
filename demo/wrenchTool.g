world {}

pandaTable (world){ Q:<t(0 0 .5)>, shape:ssBox, size:[1.5 1.5 .05 .02], color:[.3 .3 .3] }


Include: '../rai-robotModels/panda/panda_deformable.g'
Edit panda_finger_joint1 { joint:rigid Q:<t(.01 0 0)> }
Edit panda_finger_joint2 { joint:rigid, mimic!, Q:<t(.01 0 0)> }


#Edit panda_link0{ X:<t(-.3 0 .65)> }
#pandaMount (table) { Q:<t(-.3 0 .05)> }
pandaMount (pandaTable) { Q:<t(0 -.5 0) d(90 0 0 1)> }
Edit panda_joint2{ q:.2 }
deform_mount0 (pandaMount) { joint:transXY, constant }
deform_mount1 (deform_mount0 panda_link0) { } #joint:hingeY, constant, limits:[-.5 .5] }
Edit deform1 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .8 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform2 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .8 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform3 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .8 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform4 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .8 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform5 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .8 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform6 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .8 1.05 -.3 .3 -.3 .3 -.3 .3] }

gripper (panda_joint7){
  shape:marker, size:[.03], color:[.9 .9 .5],
  Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.21) >
}

wrench(gripper) { Q:<d(60 1 0 0)> joint:rigid }

wrenchShape(wrench) { Q:< t(0 0 -.04)>, mesh:'../scenarios/wrench/wrench.ply', color:[.9 .9 .9]  }
wrenchEnd(wrench) { Q:<t(-.003 0 -.2) d(-90 1 0 0)> }
deform7(wrenchEnd) { } #shape:marker, size:[.25] }
wrenchTip(deform7) { shape:marker, size:[.03], color:[.9 .9 .5] }

Edit deform7 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .8 1.05 -.3 .3 -.3 .3 -.3 .3] }

bolt (pandaTable) { Q:<t(.2 .2 0)>,   joint:rigid,  mesh:'../scenarios/wrench/bolt.ply', color:[.9 .9 .9]  }
boltTip(pandaTable) { Q:<t(.2 .2 0) t(.013 0 .12) d(30 0 0 1)>shape:marker, size:[.03], color:[.9 .9 .5] }
