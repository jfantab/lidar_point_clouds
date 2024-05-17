The `generate_samples.py` file generates the list of sample IDs that the PointPillar data loader uses. The dataset expects the length of the filename to be length six and left padded with zeros.
 
The `rename.sh` file fixes some of the filename discrepancies between both datasets, since the Object Detection dataset's Velodyne file names have length ten rather than length six.

The `script.sh` file grabs a subset of the Object Detection dataset and puts it into a separate folder tilted `point_clouds`. Change the folder names at the top to fit your needs. I ran it in the HPC so the folder names are listed as such.