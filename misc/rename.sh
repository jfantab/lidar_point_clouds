#!/bin/bash

DIRECTORY="/home/012392471/pointpillars/data/new_point_clouds"

folders=("training" "testing")
dirs=("image_2" "label_2" "calib" "velodyne")

function getTxtName {
    str="$1"
    length=${#str}
    n=$((10-$length))

    output=""
    for i in $(seq 1 $n)
    do
        output+="0"
    done

    output+="$str"
    echo $output
}

function getFormat {
    sub=$1
    format=""
    if [[ "$sub" == "calib" ]]; then
        format="txt"
    elif [[ "$sub" == "image_2" ]]; then
        format="png"
    elif [[ "$sub" == "velodyne" ]]; then
        format="bin"
    elif [[ "$sub" == "label_2" ]]; then
        format="txt"
    fi
    echo $format
}

for f in "${folders[@]}"
do
    for d in "${dirs[@]}"
    do
       if [[ "$f" == "testing" ]] && [[ "$d" == "label_2" ]] ; then
           continue
       fi 
       format=$(getFormat $d)
       
       cd "$DIRECTORY/$f/$d"
      
       for file in *
       do
           mv "$file" "${file:4}"
       done

     done
done

