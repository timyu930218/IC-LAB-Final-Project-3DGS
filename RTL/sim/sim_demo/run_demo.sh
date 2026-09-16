#!/bin/bash

# Default to Stage 8 (Render) for demo
STAGE="STAGE8_RENDER"

if [ "$1" == "stage1" ]; then
    STAGE="STAGE1_ROT"
elif [ "$1" == "stage2" ]; then
    STAGE="STAGE2_SCALE"
elif [ "$1" == "stage3" ]; then
    STAGE="STAGE3_CAM_POS"
elif [ "$1" == "stage4" ]; then
    STAGE="STAGE4_JACOBIAN"
elif [ "$1" == "stage5" ]; then
    STAGE="STAGE5_COV2D"
elif [ "$1" == "stage6" ]; then
    STAGE="STAGE6_CONIC"
elif [ "$1" == "stage7" ]; then
    STAGE="STAGE7_SORT"
elif [ "$1" == "stage8" ]; then
    STAGE="STAGE8_RENDER"
elif [ "$1" == "stage_bbox" ]; then
    STAGE="STAGE_BBOX"
elif [ "$1" == "stage_param" ]; then
    STAGE="STAGE_RASTER_PARAM"
fi

echo "Running Demo Simulation (Patterns 0-63) for $STAGE..."


# Check for Dump Flag
DUMP_DEF=""
if [ "$2" == "dump" ]; then
    DUMP_DEF="+define+FLAG_DUMP_IMAGE=0"
    echo "Frame Dumping Enabled."
    mkdir -p output_demo/stage8_render
fi

# Compile and Run using VCS
vcs -R +v2k -full64 -debug_acc+all -f presim_demo.f \
    +define+$STAGE \
    +define+PAT_L=0 \
    +define+PAT_U=63 \
    +define+FLAG_VERBOSE=1 \
    +define+FLAG_DUMPWV=1 \
    $DUMP_DEF \
    -l sim_demo.log

# Check for errors in log
if grep -q "Error" sim_demo.log; then
    echo "Simulation FAILED. Check sim_demo.log for details."
else
    echo "Simulation PASSED."
fi
