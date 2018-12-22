set(OPENCV_FOUND TRUE)

set(OPENCV_INCLUDE_DIRS /usr/local/include/opencv /usr/local/include/opencv2)

set(ALL_LIBRARIES opencv_calib3d opencv_contrib opencv_core opencv_features2d opencv_flann opencv_highgui 
opencv_imgproc opencv_legacy opencv_nonfree opencv_ocl opencv_photo opencv_stitching opencv_superres opencv_ts 
opencv_video opencv_videostab rt m dl)

foreach(lib ${ALL_LIBRARIES})
    find_library(FOUND_LIB_${lib} ${lib} PATHS /usr/local/lib) 
    set(OPENCV_LIBS ${OPENCV_LIBS} ${FOUND_LIB_${lib}})
endforeach(lib)
