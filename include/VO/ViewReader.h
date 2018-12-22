//
//  ImageReader.h
//  VisualSLAM
//
//  Created by Rong Yuan on 2/22/17.
//  Modified by Hongyi Fan on 06/04/18
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef ViewReader_h
#define ViewReader_h

#include "SystemParameters.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "Canvas.h"


using namespace std;
using namespace cv;
namespace blindfind
{
    class ViewReader
    {
    public:
        ViewReader(string _dataset, string _track, bool _stream ,bool _stereo);
        // get next view
        vector<Mat> next();
        vector<Mat> current();
        string getTrack(){return track;}
        
    private:
        string dataset;
        string track;
		bool stream;
        bool stereo;
        int index;
        string left_right;
    };

}
#endif /* ViewReader_h */
