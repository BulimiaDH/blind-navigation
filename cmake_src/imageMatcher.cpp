#include <imageMatcher.h>
#include <sstream>

void binaryToFeature(std::string featurePath, cv::Mat& feature)
{
    const char* featurePathChar=featurePath.c_str();
    FILE* fpr = fopen(featurePathChar, "rb");
    if(fpr==NULL)
    {
        std::cerr<<"Failed to open the file"<<std::endl;
        fclose(fpr);
        exit(-1);
    }
    int channel(0);
    int imageRows(0);
    int imageCols(0);
    fread(&channel, sizeof(char), 4, fpr);
    fread(&imageRows, sizeof(char), 4, fpr);
    fread(&imageCols, sizeof(char), 4, fpr);
    if(channel==1)
    {
        feature = Mat::zeros(imageRows, imageCols, CV_32FC1);
        float* pData = (float*)(feature.data);
        for(int i=0; i<imageRows*imageCols; i++)
        {
            fread(&pData[i], sizeof(float), 1, fpr);
        }
    }
    else
    {
        std::cout<<"there is something wrong about the channels of feature"<<std::endl;
        exit(-1);
    }
    fclose(fpr);
}
template<typename numberType>
std::string Number2String(numberType number)
{
	std::stringstream strStream;
	strStream<<number;
	std::string returnValue = strStream.str();
	return returnValue;
}


int id = 0;
imageMatcher::imageMatcher(){};

imageMatcher::imageMatcher(string query_image_path, string indexed_image_list_file, 
			   			   string database_file_path, string feature_name,
				 		   string matching_method)
{
	query_Images_Path_ = query_image_path;
	indexed_image_information_ = indexed_image_list_file;
	feature_name_ = feature_name;
	matching_method_ = matching_method;
	if(feature_name_=="SURF")
	{
		detector_ = SurfFeatureDetector::create(400,4,2,false);
		db_.load(database_file_path);
	}
	else if(feature_name_=="ORB")
	{
		detector_=ORB::create();
		this->vocab.load("/home/blindfind/Documents/Blindfind3/data/Vocabulary_IDOL.yml.gz");
		db_.load(database_file_path);
	}
	else if(feature_name_=="CNN")
	{
		//std::string prototxtFile = "/home/blindfind/Documents/Blindfind3/TestCode/CNNExtractor/vgg16_place365.prototxt";
    	//std::string modelFile = "/home/blindfind/Documents/Blindfind3/TestCode/CNNExtractor/vgg16_places365.caffemodel";
    //	std::string meanFile = "/home/blindfind/Documents/Blindfind3/TestCode/CNNExtractor/places365CNN_mean.binaryproto";
		std::string prototxtFile = "/home/blindfind/Documents/Blindfind3/data/CNNModel/vgg16_place365.prototxt";
		std::string modelFile = "/home/blindfind/Documents/Blindfind3/data/CNNModel/vgg16_places365.caffemodel";
		std::string meanFile = "/home/blindfind/Documents/Blindfind3/data/CNNModel/places365CNN_mean.binaryproto";
		this->CNNDetector_ = new CNNExtractor(prototxtFile, modelFile, meanFile);
		this->vocab.load("/home/blindfind/Documents/Blindfind3/testDataset/CNN_Vocabulary_224.yml.gz");
	}
	
	
	if(vocab.empty())
	{
		std::cerr<<"Vocabulary does not exist."<<std::endl;
		exit(-1);
	}
	this->db_= DBoW3::Database(this->vocab);
	cout<<"Construct finish"<<std::endl;
}

imageMatcher::imageMatcher(string indexed_image_list_file, string database_file_path, string vocabulary_file_path, string feature_name, string matching_method, bool whetherNew=true)
{
	//cout << "Construct" << endl;
	indexed_image_information_ = indexed_image_list_file;
	//db_.load(database_file_path);
	feature_name_ = feature_name;
	matching_method_ = matching_method_;
	if(feature_name_=="SURF") //SURF is currently unavailable. use ORB or CNN
	{
		std::cout<<"SURF is currently unavailable. use ORB or CNN"<<std::endl;
		exit(0);
		/*
		detector_ = SurfFeatureDetector::create(400,4,2,false);
		this->db_.load(database_file_path);
		*/
	}
	else if(feature_name_=="ORB")
	{
		detector_=ORB::create();
		this->vocab.load(vocabulary_file_path); 
	}
	else if(feature_name_=="CNN")
	{
		std::string prototxtFile = "/home/blindfind/Documents/Blindfind3/data/CNNModel/vgg16_place365.prototxt";
    	std::string modelFile = "/home/blindfind/Documents/Blindfind3/data/CNNModel/vgg16_places365.caffemodel";
    	std::string meanFile = "/home/blindfind/Documents/Blindfind3/data/CNNModel/places365CNN_mean.binaryproto";
		this->CNNDetector_ = new CNNExtractor(prototxtFile, modelFile, meanFile);
		this->vocab.load(vocabulary_file_path);
	}
	if(vocab.empty())
	{
		std::cerr<<"Vocabulary does not exist."<<std::endl;
		exit(-1);
	}
	if(whetherNew)
	{
		std::cout<<"New one"<<std::endl;
		this->db_ = DBoW3::Database(this->vocab);
	}
	else
		this->db_.load(database_file_path);
	cout<<"Construct finish"<<std::endl;
}

imageMatcher::~imageMatcher()
{
//	delete detector_; 
}

vector<string> imageMatcher::load_image_list(string path)
{
	vector<string> filenames;
	ifstream filename_file;
	filename_file.open(path.c_str());
	string temp;
	while(getline(filename_file,temp))
	{    
		filenames.push_back(temp);
	}   
	return filenames;
};


//List file names of one folder. This function used for load name of images
//target_path: the path constains images
vector<string> imageMatcher::list_names(const char* target_path)
{
	DIR *dir;
	struct dirent *ent;
	std::vector<std::string> fileList;

	/* Open directory stream */
	dir = opendir (target_path);
	if (dir != NULL) {
		/* Add all images within the directory to a vector list */
		while ((ent = readdir (dir)) != NULL) {
			std::string s;
			switch (ent->d_type) {
				case DT_REG:{
					//Get the name of the file and extract the characters to the right of the last period
					s = (ent->d_name);
					unsigned posOfPeriod = s.find_last_of(".");
					s  = s.substr(posOfPeriod+1);
					//Check file extension
					if (s.compare("bmp") == 0 || s.compare("BMP") == 0 || s.compare("jpg") == 0 || s.compare("JPG") == 0 || s.compare("jpeg") == 0 || s.compare("JPEG") == 0 ) {
						fileList.push_back(std::string(target_path) + (ent->d_name));
					}
					break; } 
				case DT_DIR:{
					// Do Nothing. Not required.
					break;
				}
				default:
					// Do Nothin
					break;
			}
		}
		closedir (dir);
	} else {
		/* Could not open directory */
		printf ("Cannot open images directory %s\n", target_path);
		exit (EXIT_FAILURE);
	}
	//Check if there were any supported image formats found
	if (fileList.empty()){
		std::cout << "No supported image formats found" << std::endl;
		//exit (EXIT_FAILURE);
	}
	sort(fileList.begin(), fileList.end());
	return fileList;
};

void imageMatcher::changeStructure(const vector<float> &plain, vector<vector<float> > &out,int L)
{
	out.resize(plain.size() / L);
	unsigned int j = 0;
	for(unsigned int i = 0; i < plain.size(); i += L, ++j)
	{
		out[j].resize(L);
		std::copy(plain.begin() + i, plain.begin() + i + L, out[j].begin());
	}
};

void imageMatcher::addFeatures()//std::vector<cv::Mat> imgSet)
{
	for(size_t k=0; k<this->feature_in_one_track_.size(); k++)
	{
		this->db_.add(this->feature_in_one_track_[k]);
		std::cout<<"Adding images: "<<k+1<<"/"<<this->feature_in_one_track_.size()<<std::endl;
	}
	this->feature_in_one_track_.clear();
}

void imageMatcher::addFeatures(std::shared_ptr<blindfind::View> image)
{
	if(feature_name_=="SURF" || feature_name_ == "ORB")
	{
		cv::Mat descriptors;
		cv::Mat mask;
		vector<cv::KeyPoint> keypoints;	
		detector_->detectAndCompute(image->getImgs()[0], mask, keypoints, descriptors);
		this->db_.add(descriptors);
	}
	else if(feature_name_=="CNN")
	{
		std::vector<cv::Mat> feature = this->CNNDetector_->extractFeatures(image->getImgs()[0], this->_featureLayer);
		image->setPoolingFeature(feature[0]);
		image->setFullConnectionFeature(feature[1]);
		this->db_.add(feature[0].clone());
	}
}
void imageMatcher::addFeatures(cv::Mat image)
{
	if(feature_name_=="SURF" || feature_name_ == "ORB")
	{
		cv::Mat descriptors;
		cv::Mat mask;
		vector<cv::KeyPoint> keypoints;	
		detector_->detectAndCompute(image, mask, keypoints, descriptors);
		this->db_.add(descriptors);
	}
	else if(feature_name_=="CNN")
	{
		std::vector<cv::Mat> feature = this->CNNDetector_->extractFeatures(image, this->_featureLayer);
		this->db_.add(feature[0].clone());
		this->full_connection_feature_set_.push_back(feature[1].clone());
	}
}


void imageMatcher::saveDatabase(std::string databaseDir)
{
	this->db_.save(databaseDir);
}

vector<pair<int, float> > imageMatcher::matching_one_image(string query_image)
{
	vector<pair<int, float> > result_index;
	vector<vector<float> > feature;
	QueryResults results;

	cv::Mat image = cv::imread(query_image, CV_LOAD_IMAGE_GRAYSCALE);

	if (image.empty())
	{
		cout << "The image is empty" << endl;
		exit(1);
	}
	cv::Mat mask;
	vector<cv::KeyPoint> keypoints;
	cv::Mat  descriptors;
	detector_->detectAndCompute(image, mask, keypoints, descriptors);
	db_.query(descriptors, results, 20);
	vector<int> indexes;
	for (int j = 0; j < results.size(); j++)
	{
		pair<int, float> temp;
		temp.first = results[j].Id;
		temp.second = results[j].Score;
		result_index.push_back(temp);
	}
	return result_index;
}

float imageMatcher::compareTwoImages(cv::Mat image1, cv::Mat image2)
{
	if(image1.empty() || image2.empty())
	{
		cout<<"The image is empty"<<endl;
		exit(1);
	}
	cv::Mat mask1;
	cv::Mat mask2;
	vector<cv::KeyPoint> keypoints1;
	vector<cv::KeyPoint> keypoints2;
	cv::Mat descriptors1;
	cv::Mat descriptors2;
	detector_->detectAndCompute(image1, mask1, keypoints1, descriptors1);
	detector_->detectAndCompute(image2, mask2, keypoints2, descriptors2);
	DBoW3::BowVector v1;
	DBoW3::BowVector v2;
	vocab.transform(descriptors1, v1);
	vocab.transform(descriptors2, v2);
	return vocab.score(v1,v2);
}


vector<pair<int, float> >imageMatcher::matching_one_image(std::shared_ptr<blindfind::View> image, double threshold)
{
	vector<pair<int, float> > result_index;
	QueryResults results;
	cv::Mat queryImage = image->getImgs()[0];
	if(queryImage.empty())
	{
		cout<<"The image is empty"<<endl;
		exit(1);
	}
	if(feature_name_=="SURF" || feature_name_=="ORB")
	{
		cv::Mat descriptors;
		cv::Mat mask;
		vector<cv::KeyPoint> keypoints;
		detector_->detectAndCompute(queryImage, mask, keypoints, descriptors);
		db_.query(descriptors,results, 20);
	}
	else if(feature_name_=="CNN")
	{
		std::vector<cv::Mat> feature = this->CNNDetector_->extractFeatures(image->getImgs()[0], this->_featureLayer);
		this->feature_in_one_track_.push_back(feature[0].clone());
		image->setPoolingFeature(feature[0]);
		image->setFullConnectionFeature(feature[1]);
		db_.query(feature[0], results, 20);
	}
	vector<int> indexes;
	for (int j = 0; j < results.size(); j++)
	{
		pair<int, float> temp;
		temp.first = results[j].Id;
		temp.second = results[j].Score;
		result_index.push_back(temp);
	}
	vector<pair<int, float> > shortList1 = thresholding(result_index,threshold);
	return shortList1;
}
vector<pair<int, int> >imageMatcher::matching_images(std::shared_ptr<blindfind::View> image1, std::shared_ptr<blindfind::View> image2, double threshold)
{
	vector< pair<int, int> > returnVector;

	vector<pair<int,float> > full_list_1;
	vector<pair<int,float> > full_list_2;

	full_list_1 = matching_one_image(image1, threshold);
	full_list_2 = matching_one_image(image2, threshold);

	vector<pair<int, float> > shortList1 = thresholding(full_list_1,0.2f);
	vector<pair<int, float> > shortList2 = thresholding(full_list_2, 0.2f);

	if(shortList1.size()==0&&shortList2.size()==0)
	{
		vector<pair<int, int> > emptyResult;
		return emptyResult;
	}

	//Refine
	if (shortList1.size() != 0 && shortList2.size() != 0) 
	{
		int size1 = shortList1.size();
		int size2 = shortList2.size();
	
		int sizeFinal;
		if (size1 > size2)
		{
			sizeFinal = size2;
		}
		else
		{
			sizeFinal = size1;
		}
			
		for (int k = 0; k < sizeFinal; k++)
		{
			returnVector.push_back(pair<int,int>(shortList1[k].first, shortList2[k].first));
		}
	}
	else
	{
		if (shortList1.size() == 0) 
		{
			for (int k = 0; k < shortList2.size(); k++)
			{
				returnVector.push_back(pair<int,int>(shortList2[k].first, shortList2[k].first));
			}
		}
		
		if (shortList2.size() == 0)
		{
			for (int k = 0; k < shortList1.size(); k++)
			{
				returnVector.push_back(pair<int,int>(shortList1[k].first, shortList1[k].first));
			}
		}	
	}
	return returnVector;
}

vector<pair<int, float> > imageMatcher::matching_one_image(cv::Mat image, double threshold)
{
	vector<pair<int, float> > result_index;
	vector<vector<float> > feature;
	QueryResults results;

	if (image.empty()) 
	{
		cout << "The image is empty" << endl;
		exit(1);
	}
	
	cv::Mat queryFCFeature;
	if(feature_name_=="SURF" || feature_name_=="ORB")
	{
		cv::Mat descriptors;
		cv::Mat mask;
		vector<cv::KeyPoint> keypoints;	
		detector_->detectAndCompute(image, mask, keypoints, descriptors);
		db_.query(descriptors,results,20);
	}
	else if(feature_name_=="CNN")
	{
		std::vector<cv::Mat> descriptors = this->CNNDetector_->extractFeatures(image, this->_featureLayer);
		//std::vector<cv::Mat> descriptors = this->CNNDetector_->extractFeatures(image, "pool3");
		this->feature_in_one_track_.push_back(descriptors[0].clone());
		this->full_connection_feature_set_.push_back(descriptors[1].clone());
		db_.query(descriptors[0],results,20);
		queryFCFeature = descriptors[1].clone();
		//std::cout<<"result_index size: "<<results.size()<<std::endl;
	}
	
	vector<int> indexes;
	for (int j = 0; j < results.size(); j++)
	{
		pair<int, float> temp;
		temp.first = results[j].Id;
		temp.second = results[j].Score;
		result_index.push_back(temp);
	}
	/**The following code is for BH dataset**/
	/*
	if(feature_name_=="CNN")
	{
		for(size_t k=0; k<result_index.size(); k++)
		{
			cv::Mat candidateFCFeature; 
			binaryToFeature("/home/blindfind/Documents/Blindfind3/tools/allFeatures/"+Number2String<int>(result_index[k].first), candidateFCFeature);
			if (candidateFCFeature.empty())
			{
				std::cerr<<"Empty feature"<<std::endl;
				exit(-1);
			}
			result_index[k].second = cv::norm(candidateFCFeature.clone(), queryFCFeature.clone(), CV_L2);
		}
		std::sort(result_index.begin(), result_index.end(), [](const std::pair<int, float> a, const std::pair<int, float> b)->bool{return a.second<b.second;});
	}
	*/
	vector<pair<int, float> > shortList1 = thresholding(result_index, threshold);
	if(feature_name_== "CNN")
	{
		for(size_t k=0; k<shortList1.size(); k++)
		{
			cv::Mat candidateFCFeature = this->full_connection_feature_set_[shortList1[k].first].clone();
			if(candidateFCFeature.empty())
			{
				std::cerr<<"Empty Feature"<<std::endl;
				exit(-1);
			}
			shortList1[k].second = cv::norm(candidateFCFeature.clone(), queryFCFeature.clone(), CV_L2);
		}
		std::sort(shortList1.begin(), shortList1.end(), [](const std::pair<int, float> a, const std::pair<int, float> b)->bool{return a.second<b.second;});
	}
	return shortList1;
}

vector<pair<int, int> > imageMatcher::matching_images(cv::Mat image1, cv::Mat image2, double threshold)
{	
	vector< pair<int, int> > returnVector;

	vector<pair<int,float> > shortList1;
	vector<pair<int,float> > shortList2;

	shortList1 = matching_one_image(image1, threshold);
	shortList2 = matching_one_image(image2, threshold);

	if(shortList1.size()==0&&shortList2.size()==0)
	{
		vector<pair<int, int> > emptyResult;
		return emptyResult;
	}
	//Refine
	if (shortList1.size() != 0 && shortList2.size() != 0) 
	{
		int size1 = shortList1.size();
		int size2 = shortList2.size();
	
		int sizeFinal;
		if (size1 > size2)
		{
			sizeFinal = size2;
		}
		else
		{
			sizeFinal = size1;
		}
			
		for (int k = 0; k < sizeFinal; k++)
		{
			returnVector.push_back(pair<int,int>(shortList1[k].first, shortList2[k].first));
		}
	}
	else
	{
		if (shortList1.size() == 0) 
		{
			for (int k = 0; k < shortList2.size(); k++)
			{
				returnVector.push_back(pair<int,int>(shortList2[k].first, shortList2[k].first));
			}
		}
		
		if (shortList2.size() == 0)
		{
			for (int k = 0; k < shortList1.size(); k++)
			{
				returnVector.push_back(pair<int,int>(shortList1[k].first, shortList1[k].first));
			}
		}	
	}
	return returnVector;
}

vector<pair<int, int> > imageMatcher::matching_images(string query1, string query2)
{
	vector< pair<int, int> > returnVector;

	vector<pair<int,float> > full_list_1;
	vector<pair<int,float> > full_list_2;

	full_list_1 = matching_one_image(query1);
	full_list_2 = matching_one_image(query2);

	
	vector<pair<int, float> > shortList1 = thresholding(full_list_1,0.2f);
	vector<pair<int, float> > shortList2 = thresholding(full_list_2, 0.2f);
	
	if(shortList1.size()==0&&shortList2.size()==0)
	{
		vector<pair<int, int> > emptyResult;
		return emptyResult;
	}
	//Refine
	if (shortList1.size() != 0 && shortList2.size() != 0) 
	{
		int size1 = shortList1.size();
		int size2 = shortList2.size();
	
		int sizeFinal;
		if (size1 > size2)
		{
			sizeFinal = size2;
		}
		else
		{
			sizeFinal = size1;
		}
			
		for (int k = 0; k < sizeFinal; k++)
		{
			returnVector.push_back(pair<int,int>(shortList1[k].first, shortList2[k].first));
		}
	}
	else
	{
		if (shortList1.size() == 0) 
		{
			for (int k = 0; k < shortList2.size(); k++)
			{
				returnVector.push_back(pair<int,int>(shortList2[k].first, shortList2[k].first));
			}
		}
		
		if (shortList2.size() == 0)
		{
			for (int k = 0; k < shortList1.size(); k++)
			{
				returnVector.push_back(pair<int,int>(shortList1[k].first, shortList1[k].first));
			}
		}	
	}
	return returnVector;
}
vector<pair<int, float> > imageMatcher::thresholding(vector<pair<int,float> > full_list, float threshold)
{
	vector<pair<int,float> > short_list;
	for (int i = 0; i < full_list.size(); i++)
	{
		if(full_list[i].second >= threshold)
		{
			short_list.push_back(full_list[i]);
		}
		else
		{
			return short_list;
		}
	}
};

