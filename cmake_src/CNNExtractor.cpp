/**
 * CNNExtractor inplmentation by Han Deng 
 * Using Caffe
**/
#include "CNNExtractor.h"

CNNExtractor::CNNExtractor(const string& modelFile, const string& trainedFile, const string& meanFile)
{
    #ifdef CPU_ONLY
        Caffe::set_mode(Caffe::CPU);
    #else
        Caffe::set_mode(Caffe::GPU);
    #endif

    this->_net.reset(new Net<float>(modelFile, TEST));
    this->_net->CopyTrainedLayersFrom(trainedFile);

    /**Just do some check, can later be deleted**/
    CHECK_EQ(this->_net->num_inputs(), 1);
    CHECK_EQ(this->_net->num_outputs(),1);

    Blob<float>* inputLayer = this->_net->input_blobs()[0];
    this->_numChannels=inputLayer->channels();

    CHECK(this->_numChannels==3 || this->_numChannels==1)
        <<"Input layer should have 1 or 3 channels";
    this->_inputGeometry=cv::Size(inputLayer->width(), inputLayer->height());
    this->setMean_(meanFile);
}


std::vector<cv::Mat> CNNExtractor::extractFeatures(cv::Mat img, const std::string& blobName)
{
    Blob<float>* inputLayer=this->_net->input_blobs()[0];
    inputLayer->Reshape(1, this->_numChannels, this->_inputGeometry.height, this->_inputGeometry.width);
    this->_net->Reshape();
    std::vector<cv::Mat> inputChannels;
    this->wrapInputLayer_(&inputChannels);
    this->preProcess_(img, &inputChannels);
    this->_net->Forward();
    const boost::shared_ptr<Blob<float> > lastConvLayer = this->_net->blob_by_name(blobName);
    const boost::shared_ptr<Blob<float> > fullConnectionLayer = this->_net->blob_by_name("fc7");
    cv::Mat prevConvFeature;
    if(blobName=="pool5")
        prevConvFeature = cv::Mat(lastConvLayer->channels(), lastConvLayer->height()*lastConvLayer->width(), CV_32FC1, lastConvLayer->mutable_cpu_data());
    /**
     * This is to treat each local feature as feature of the local area, using depth as the feature vector
     * Currently the blindfind3 system uses this method to extract features from pool3, which can provide good performance.
     **/
    else if(blobName=="pool3")
    {
       prevConvFeature = cv::Mat::zeros(lastConvLayer->height()*lastConvLayer->width(), lastConvLayer->channels(), CV_32FC1);
       float* fpr = (float*)(prevConvFeature.data);
       int pointer=0;
       for(size_t i=0; i<lastConvLayer->height(); i++)
           for(size_t j=0; j<lastConvLayer->width(); j++)
               for(size_t k=0; k<lastConvLayer->channels(); k++)
               {
                   memcpy(&(fpr[pointer]), &lastConvLayer->mutable_cpu_data()[lastConvLayer->offset(0,k,j,i)], sizeof(float)/sizeof(char));
                pointer+=1;
               }
    }
    /*

    /*Now we use PCA to reduce the Dimension*/
    /*512->256*/
    /*
    cv::PCA pca(prevConvFeature, cv::Mat(), CV_PCA_DATA_AS_COL, 256);
    cv::Mat convFeature = pca.project(prevConvFeature);
    */

    cv::Mat fullConnectionFeature(1,4096, CV_32FC1, fullConnectionLayer->mutable_cpu_data());
    std::vector<cv::Mat> featureSet;
    featureSet.push_back(prevConvFeature.clone());
    featureSet.push_back(fullConnectionFeature.clone());
    return featureSet;
}

                
void CNNExtractor::setMean_(const string& meanFile)
{
    caffe::BlobProto blobProto;
    caffe::ReadProtoFromBinaryFileOrDie(meanFile.c_str(), &blobProto);

    Blob<float> meanBlob;
    meanBlob.FromProto(blobProto);
    CHECK_EQ(meanBlob.channels(), this->_numChannels)
        <<"Numbers of channels of mean file doesn't match input layer";

    std::vector<cv::Mat> channels;
    float* data = meanBlob.mutable_cpu_data();
    for(int i=0; i<this->_numChannels; ++i)
    {
        cv::Mat channel(meanBlob.height(), meanBlob.width(), CV_32FC1, data);
        channels.push_back(channel);
        data+=meanBlob.height()*meanBlob.width();
    }

    cv::Mat mean;
    cv::merge(channels, mean);
    cv::Scalar channelMean = cv::mean(mean);
    this->_mean = cv::Mat(this->_inputGeometry, mean.type(), channelMean);

}

void CNNExtractor::wrapInputLayer_(std::vector<cv::Mat>* inputChannels)
{
    Blob<float>* inputLayer = this->_net->input_blobs()[0];
    int width = inputLayer->width();
    int height = inputLayer->height();
    float* inputData = inputLayer->mutable_cpu_data();

    for(int i=0; i<inputLayer->channels(); ++i)
    {
        cv::Mat channel(height, width, CV_32FC1, inputData);
        inputChannels->push_back(channel);
        inputData+=width*height;
    }
}

void CNNExtractor::preProcess_(const cv::Mat& img, std::vector<cv::Mat>* inputChannels)
{
    cv::Mat sample;
    if(img.channels()==3 && this->_numChannels==1)
        cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
    else if (img.channels()==1 && this->_numChannels==3)
        cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
    else
        sample=img;
    cv::Mat sampleResized;
    if(sample.size()!=this->_inputGeometry)
        cv::resize(sample, sampleResized, this->_inputGeometry);
    else
        sampleResized = sample;
    cv::Mat sampleFloat;
    if(this->_numChannels==3)
        sampleResized.convertTo(sampleFloat, CV_32FC3);
    else    
        sampleResized.convertTo(sampleFloat, CV_32FC1);
    cv::Mat sampleNormalized;
    cv::subtract(sampleFloat, this->_mean, sampleNormalized);
    cv::split(sampleNormalized, *inputChannels);
    CHECK(reinterpret_cast<float*>(inputChannels->at(0).data)==this->_net->input_blobs()[0]->cpu_data())
        <<"Not the same for the memory location of input and blob";
}
