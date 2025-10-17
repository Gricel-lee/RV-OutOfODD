rm output_video/temp_image*
python3 make_frames.py $1 $2 $3
rm output_video/$4.mp4
ffmpeg -framerate 30 -pattern_type glob -i 'output_video/temp_image_*.png' -c:v libx264 output_video/$4.mp4
