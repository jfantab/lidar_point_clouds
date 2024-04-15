#!/bin/bash

prefix_src="/scratch/cmpe297-sp24/data_object_velodyne"
prefix_dest="/scratch/cmpe297-sp24/point_cloud_samples"

folders=("training", "testing")
subdirs=("calib", "image_2", "velodyne", "label_2")
formats=("txt", "png", "bin", "txt")

function getTxtName {
    str="$1"
    length=${#str}
    n=$((6-$length))

    output=""
    for i in $(seq 1 $n)
    do
        output+="0"
    done

    output+="$str"
    echo $output
}

// TODO: get format to work
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

for folder  in "${folders[@]}"
do
    for sub in "${subdirs[@]}":
    do   
        format=$(getFormat $sub)	
        echo "Format: $format"
        for i in {0..10}
	do
            num=$(getTxtName $i) 
            echo "$prefix_src/$folder/$sub/$num.$format $prefix_dest/$folder/$sub/$num.$format"
        done	    
    done
done


