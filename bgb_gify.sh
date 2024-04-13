# this script records 4 second gameplay of each of the examples and creates a
# that account for window decorations
# if using i3 you'll need to skip these


# -y == yes always overwrite files
# -t (time) H:MM:SS.DCM
# avfoundation (grab input from macOS screen)
# -r 20 (input framerate of 20)
# -s (size) (aka 620x480)
# -i (input?? which screen, I suppose) :0.0+offset (:0.0 is screen I believe)
# -qscale 0 (quality -- prevent artifacts? No compression, I think)
#ffmpeg -y -t $record_time -f avfoundation -r 20 -s $dimensions -i :0.0+$xoffset,$yoffset -qscale 0 $videofile


## =========== STEP 3: GIF-ify it

videofile=video.mov
imageprefix=frame

# ratio and delay are interlinked. 5 frames per 1 second?
# then better make the delay 200ms
ratio=3/1
delay=30

ffmpeg -i $videofile -r $ratio $imageprefix%03d.png

# now let's recombine those frames into a gif
echo ... converting to pngs ...

## ============= STEP 3.3: GIF-ify resized images
# [optional] pngquantize it first to reduce # of colors
echo resizing images
for imagefile in $imageprefix*.png; do
    # -sample (instead of -resize) resizes but with pixel sampling
    # so it'll retain the same colors
    convert $imagefile -sample 160x144 $imagefile
done

echo ...gifying resized images...
convert -loop 0 -delay $delay $imageprefix*.png $clip2

# delete temporary images
rm $imageprefix*.png

