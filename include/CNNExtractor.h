#ifndef CAFFE_EXTRACTOR
#define CAFFE_EXTRACTOR

#include<caffe/caffe.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<algorithm>
#include<iosfwd>
#include<memory>
#include<string>
#include<utility>
#include<vector>

using namespace caffe;
using std::string;
class CNNExtractor{
    public:
        CNNExtractor(const string& modelFile,
                    const string& trainedFile,
                    const string& meanFile);

        std::vector<cv::Mat> extractFeatures(cv::Mat img, const std::string& blobName);

    private:
        void setMean_(const string& meanFile);
        void wrapInputLayer_(std::vector<cv::Mat>* inputChannels);
        void preProcess_(const cv::Mat& img, std::vector<cv::Mat>* inputChannels);

    private:
        boost::shared_ptr<caffe::Net<float> > _net;
        cv::Size _inputGeometry;
        int _numChannels;
        cv::Mat _mean;
};



#endif // CAFFE_EXTRACTOR
