#==========================================================================
#  plot.gnu - renders four PNGs from the data files written by example.cpp
#  (base name 'example').
#
#  Invocation (from the Makefile target 'plot'):
#    gnuplot plot/plot.gnu
#
#  Expected in the current directory:
#    example_airfoil.dat      columns 1/2 = airfoil x/y, 3/4 = thickness x/y
#    example_camber.dat       columns 1/2 = camber line x/y
#    example_circ_source.dat  block 0 (index 0) = circulation x/y,
#                             block 1 (index 1) = source x/y
#    example_grad_curv.dat    1/2/3 = upper x/slope/curvature,
#                             4/5/6 = lower x/slope/curvature
#==========================================================================

set datafile separator whitespace
set grid lw 1
set key top right font "sans,30"
# ~300 dpi at the same print size (900x520 px at 96 dpi -> 3x pixels).
set terminal pngcairo size 2700,1560 enhanced font "sans,33" linewidth 4.5 fontscale 1

BASE   = "example"
outdir = "plot"

# Zero line (y=0) as a dark-gray line, for all plots.
set arrow from graph 0, first 0 to graph 1, first 0 nohead \
    lc "black" lw 1.5 dashtype "-" front

# Default: single y-axis, no mirrored y2-axis.
set ytics nomirror
unset y2tics
unset y2label

# (a) Airfoil + thickness distribution
set title "Airfoil + thickness distribution"
set xlabel "x"
set ylabel "y"
set output outdir . "/" . BASE . "_airfoil.png"
plot BASE . "_airfoil.dat" using 1:2 with lines title "Airfoil", \
     BASE . "_airfoil.dat" using 3:4 with lines title "Thickness"

# (b) Camber line
set title "Camber line"
set ylabel "y"
set output outdir . "/" . BASE . "_camber.png"
plot BASE . "_camber.dat" using 1:2 with lines title "Camber line"

# (c) Circulation + source strength: one shared y-axis [-0.3,3].
#     The source distribution runs negative (closure condition: as many
#     sources as sinks); the circulation leading-edge peak is clipped at 3.
set title "Normalized circulation / scaled source distribution"
set xlabel "x"
set ylabel "Normalized / scaled singularity strength"
set yrange [-0.3:3]
unset y2tics
unset y2label
set output outdir . "/" . BASE . "_circ_source.png"
plot BASE . "_circ_source.dat" index 0 using 1:2 with lines title "k / U_inf (c_l = 0.5)", \
     BASE . "_circ_source.dat" index 1 using 1:2 with lines title "q / (2 U_inf) = dz^(t)/dx"

# (d) Slope + curvature (upper/lower): two y-axes.
#     left  (y1): slope,     range [-2,2]
#     right (y2): curvature, range [-5,5]
set title "Slope / curvature"
set xlabel "x"
set yrange [-2:2]
set y2range [-5:5]
set ylabel "Slope"
set y2label "Curvature"
set y2tics
set output outdir . "/" . BASE . "_grad_curv.png"
plot BASE . "_grad_curv.dat" using 1:2 with lines axes x1y1 title "Slope upper", \
     BASE . "_grad_curv.dat" using 1:3 with lines axes x1y2 title "Curvature upper", \
     BASE . "_grad_curv.dat" using 4:5 with lines axes x1y1 title "Slope lower", \
     BASE . "_grad_curv.dat" using 4:6 with lines axes x1y2 title "Curvature lower"

set output
