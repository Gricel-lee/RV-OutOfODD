source run_simulation.sh 
echo "Completed simulations; making video"
source make_video.sh $1 $2 $3
echo "Video completed; saved as: $1.mp4"