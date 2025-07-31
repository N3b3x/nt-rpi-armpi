Date:2021/05/27
This is a program for camera calibration using OpenCV's built-in function library. The steps are as follows:

1 First, prepare a calibration board. You can run GenerateCalibrationPlate.py to generate the required calibration board.
By default, it generates an 8x6 calibration board. If you need to modify parameters, you can modify the CalibrationConfig.py file.

2 After generating the calibration board, you need to print it with a printer. When printing, select 1:1 printing. After printing, attach the calibration board
to a flat object, then move it to collect images.

3 Run CollectCalibrationPicture.py to collect images. Move the calibration board to make it appear in various positions on the image,
avoiding positions that are too close to the edges. Around 20-30 images should be sufficient. If the effect is not good, consider increasing the number of images.

4 Run the Calibration.py program to perform calibration. The calibration program will filter out unqualified images and delete them. 20-30 qualified
images are ideal. If there are not enough, you can continue collecting more and then recalibrate.

5 Run TestCalibration.py to test the calibration effect. The calibrated image will show a smaller field of view than the original. If you want to
call it in a program, you can refer to this program.
