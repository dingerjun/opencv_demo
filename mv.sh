#!/bin/bash

files='*.jpg'
echo $files

#for file in $files; do
#echo $file
#target='awk '{split($file, array,[_:]+); print $2}''
#echo $target
#done

for((i=1;i<36;i++));
do
mv left_$i.jpg Image_$i\L.jpg
mv right_$i.jpg Image_$i\R.jpg

done
