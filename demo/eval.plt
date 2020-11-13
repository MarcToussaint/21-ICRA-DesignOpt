reset

set style fill solid 0.5 border -1
set style boxplot outliers pointtype 7
set style data boxplot
set boxwidth  0.2
set pointsize 0.5

#unset key
#set border 2
set ylabel "sum of squares"
set xtics ("torques" 1, "controls" 2, "velocities" 3, "max vel" 4) scale 0.0
#set xtics nomirror
#set ytics nomirror
#set yrange [0:100]
set bmargin 3

a1=.02
a2=1.
a3=5.
a4=10.

plot \
  'dat.0'  using (0.75):($2*a1) ls 1 title 'no deform, opt ctrl',\
  'dat.1'  using (1.00):($2*a1) ls 2 title 'deform, opt ctrl',\
  'dat.2'  using (1.25):($2*a1) ls 3 title 'deform, opt ctrl & torque',\
  'dat.0'  using (1.75):($6*a2) ls 1 not,\
  'dat.1'  using (2.00):($6*a2) ls 2 not,\
  'dat.2'  using (2.25):($6*a2) ls 3 not,\
  'dat.0'  using (2.75):($8*a3) ls 1 not,\
  'dat.1'  using (3.00):($8*a3) ls 2 not,\
  'dat.2'  using (3.25):($8*a3) ls 3 not,\
  'dat.0'  using (3.75):($10*a4) ls 1 not,\
  'dat.1'  using (4.00):($10*a4) ls 2 not,\
  'dat.2'  using (4.25):($10*a4) ls 3 not
