world {}

table (world){
  shape:ssBox, Q:<t(0 0. .6)>, size:[1. 1.5 .1 .02], color:[.3 .3 .3]
}

startRegion (table){
  shape:box, Q:<t(.2 -.4 .15)>, size:[.5 .5 .2], color:[.4 .4 .8 .2]
  joint:rigid
}

targetRegion (table){
  shape:box, Q:<t(.2 .4 .15)>, size:[.5 .5 .1], color:[.4 .8 .4 .2]
  joint:rigid
}

box (table){
  shape:ssBox, Q:<t(.3 -.5 .08) d(120 0 0 1)>, size:[.16 .1 .06 .02], color:[.4 .4 .8]
  joint:rigid
}

target(table){
  shape:sphere, Q:<t(.3 .5 .08) d(120 0 0 1)>, size:[.02], color:[.4 .8 .4]
}

Include: '../rai-robotModels/panda/panda_deformable.g'
#Edit panda_link0{ X:<t(-.3 0 .65)> }
pandaMount (table) { Q:<t(-.3 0 .05)> }
pandaMount1 (pandaMount) { joint:transXY, constant }
pandaPosition (pandaMount1 panda_link0) { joint:hingeY, constant, limits:[-.5 .5] }
Edit deform1 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .6 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform2 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .6 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform3 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .6 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform4 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .6 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform5 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .6 1.05 -.3 .3 -.3 .3 -.3 .3] }
Edit deform6 { joint:free, constant, limits:[-.05 .05 -.05 .05 -.05 .05 .6 1.05 -.3 .3 -.3 .3 -.3 .3] }

gripper (panda_joint7){
  shape:marker, size:[.03], color:[.9 .9 .5],
  Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.155) t(0 0 -.05)>
}
