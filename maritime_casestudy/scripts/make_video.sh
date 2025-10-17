rm output_video/*.png
python3 make_frames.py $2 $3
cd output_video
rm $1.mp4
ffmpeg -framerate 30 -pattern_type glob -i 'temp_image_*.png' -c:v libx264 $1.mp4
rm *.png
cd ../