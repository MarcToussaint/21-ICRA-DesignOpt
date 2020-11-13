world {}

conveyorCenter (world) { shape:ssBox, Q:<t(0 0. .6)>, size:[.5 1.4 .1 .02], color:[.3 .3 .3] }
conveyorRight (world) { shape:ssBox, Q:<t(.6 0. .6)>, size:[.5 1.4 .1 .02], color:[.3 .3 .3] }
conveyorLeft (world) { shape:ssBox, Q:<t(-.6 0. .6)>, size:[.5 1.4 .1 .02], color:[.3 .3 .3] }

pandaTable (conveyorCenter){ Q:<t(0 -.25 .35)>, shape:ssBox, size:[.4 .8 .05 .02], color:[.3 .3 .3] }

(conveyorCenter){ Q:<t( .3 .13 -.15)> shape:ssBox, size:[.03 .03 .9 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t( .23 .13 .32) d(-60 0 1 0)> shape:ssBox, size:[.03 .03 .2 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t( -.3 .13 -.15)> shape:ssBox, size:[.03 .03 .9 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t( -.23 .13 .32) d(60 0 1 0)> shape:ssBox, size:[.03 .03 .2 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t( .3 -.63 -.15)> shape:ssBox, size:[.03 .03 .9 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t( .23 -.63 .32) d(-60 0 1 0)> shape:ssBox, size:[.03 .03 .2 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t( -.3 -.63 -.15)> shape:ssBox, size:[.03 .03 .9 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t( -.23 -.63 .32) d(60 0 1 0)> shape:ssBox, size:[.03 .03 .2 .02], color:[.3 .3 .3] }

(conveyorCenter){ Q:<t(-.9 0 .2)> shape:ssBox, size:[.03 1.5 .03 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t(-.3 0 .2)> shape:ssBox, size:[.03 1.5 .03 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t( .3 0 .2)> shape:ssBox, size:[.03 1.5 .03 .02], color:[.3 .3 .3] }
(conveyorCenter){ Q:<t( .9 0 .2)> shape:ssBox, size:[.03 1.5 .03 .02], color:[.3 .3 .3] }

(conveyorCenter){ Q:<t( 0 .73 .2)> shape:ssBox, size:[1.8 .03 .03 .02], color:[.3 .3 .3] }

startRegion (conveyorCenter){
  shape:box, Q:<t(0 .5 .15)>, size:[.2 .15 .1], color:[.4 .4 .8 .2]
  joint:rigid
}

Prefix: "S_"
Include: 'tote.g'
Prefix!
Edit S_tote (startRegion) {}

targetRegion1 (conveyorRight){
  shape:box, Q:<t(0 0 .15) d(90 0 0 1)>, size:[.2 .15 .1], color:[.4 .8 .4 .2]
  joint:rigid
}

Prefix: "T1_"
Include: 'tote.g'
Prefix!
Edit T1_tote (targetRegion1) {}

targetRegion2 (conveyorLeft){
  shape:box, Q:<t(0 0 .15) d(90 0 0 1)>, size:[.2 .15 .1], color:[.4 .8 .4 .2]
  joint:rigid
}

Prefix: "T2_"
Include: 'tote.g'
Prefix!
Edit T2_tote (targetRegion2) {}

box (startRegion){
  shape:ssBox, size:[.12 .08 .04 .02], color:[.4 .4 .8]
  joint:rigid
}

target(targetRegion1){
  shape:sphere, size:[.02], color:[.4 .8 .4]
}

Include: '../rai-robotModels/panda/panda_deformable.g'
Delete panda_link7>panda_joint8
Delete panda_joint8
Delete panda_link8>panda_hand_joint
Delete panda_hand_joint
Delete panda_hand_1
Delete panda_hand>panda_finger_joint1
Delete panda_hand>panda_finger_joint2
Delete panda_finger_joint1
Delete panda_finger_joint2
Delete panda_leftfinger_1
Delete panda_rightfinger_1


#Edit panda_link0{ X:<t(-.3 0 .65)> }
#pandaMount (table) { Q:<t(-.3 0 .05)> }
pandaMount (pandaTable) { Q:<t(0 .2 0) d(90 0 0 1)> }
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
  Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.23) t(0 0 -.05)>
}

suctionTube(gripper) { Q:<t(0 0 .15)>, shape:capsule, size:[.2 .01], color:[.9 .9 .9]  }
suctionCup(gripper) { Q:<t(0 0 .03)>, shape:ssCvx, mesh:'asciiSuctionCup.arr', size:[.002], color:[.9 .9 .9]  }


