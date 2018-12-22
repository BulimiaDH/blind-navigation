#include <index_builder.h>

using namespace std;
using namespace DBoW3;
//using namespace DUtils;

//Constructor
index_builder::index_builder(){};

//Constructor
index_builder::index_builder(string path_of_dataset_image, string feature_name, string matching_method,string feature_output, bool cross_validation=false)
{
	path_of_dataset_ = path_of_dataset_image;
	feature_name_ = feature_name;
	matching_method_ = matching_method;
	feature_output_ = feature_output;
	ofstream i_list;
	cross_validation_ = cross_validation;
	string image_list_file = string(feature_output_ + "dataset_list.txt"); //Which saves name of images without directory
	image_list_ = load_image_list(image_list_file);
	indexed_image_list_ = load_image_list(image_list_file);
	i_list.close();
	if(feature_name=="SURF")
	{
		detector_ = SurfFeatureDetector::create(400,4,2,false);
	}
	else if(feature_name == "ORB")
	{
		detector_ = ORB::create();
	}
	else
	{
		cerr << "Not supported yet" << endl;
	}
};

index_builder::~index_builder(){};

//Set parameter for vocabulary tree method
//K: cluster in each level
//L: The depth of one level
void index_builder::setParam(int K, int L)
{
	K_ = K;
	L_ = L;
};

void index_builder::setCrossNum(int cross_num)
{
	cross_num_ = cross_num;
}


void index_builder::changeStructure(const vector<float> &plain, vector<vector<float> > &out, int L)
{
	out.resize(plain.size() / L);
	unsigned int j = 0;
	for(unsigned int i = 0; i < plain.size(); i += L, ++j)
	{
		out[j].resize(L);
		copy(plain.begin() + i, plain.begin() + i + L, out[j].begin());
	}
};

vector<string> index_builder::load_image_list(string path)
{
	vector<string> filenames;/*{{{*/
	ifstream filename_file;
	filename_file.open(path.c_str());
	string temp;
	while(getline(filename_file,temp))
	{			
		filenames.push_back(temp);
	}
	return filenames;
	/*}}}*/
};

//List file names of one folder. This function used for load name of images
//target_path: the path constains images
vector<string> index_builder::list_names(const char* target_path)
{
	DIR *dir;/*{{{*/
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
                    // Do Nothing
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
/*}}}*/
};

void index_builder::extract_features()
{
	//image_list_ = list_names(path_of_dataset_.c_str());
	if(feature_name_=="SURF")
	{
		extract_SURF();
	}
	else if(feature_name_=="ORB")
	{
		extract_ORB();
	}
	else
	{
		cerr << "Not supported yet" << endl;
	}
}

void index_builder::extract_SURF()
{
	
	const int NIMAGE = image_list_.size();
	vector<vector<vector<float> > > features;
	cv::Mat emptyResult;
	cv::Mat mask;
	vector<cv::KeyPoint> keypoints;
	cv::Mat descriptor;
	//ofstream o_list;
	//string image_list_file = string(feature_output_ + "images_list.txt");
	//o_list.open(image_list_file.c_str());

	//vector<string>	image_list_refined;
	for (int i = 0; i < NIMAGE; i++)
	{
		std::cout<<"Cross validation::"<<cross_validation_<<std::endl;
		if(cross_validation_)
		{
			std::cout<<"We are in cross_validation mode"<<std::endl;
			if((cross_num_+i)%10==0)
			{
				descriptors_.push_back(emptyResult);
				cout << "Used as test dataset: "<<i + 1 << "/" << NIMAGE << endl;
				continue;
			}
		}
		cv::Mat image = cv::imread(path_of_dataset_ + image_list_[i], CV_LOAD_IMAGE_GRAYSCALE);
		if (image.empty()) cerr << "empty image" << endl;
		detector_->detectAndCompute(image,mask,keypoints,descriptor);
		if (keypoints.size() == 0)
		{
			descriptors_.push_back(emptyResult);
			cout << "No features to extract: "<<i + 1 << "/" << NIMAGE << endl;
			continue;
		}
		else
		{
		//	image_list_refined.push_back(image_list_[i]);
			descriptors_.push_back(descriptor);
		
		//o_list << image_list_[i] << endl;

		//changeStructure(descriptors, features.back(), surf.descriptorSize());
	//	cout << i + 1 << "/" << NIMAGE << endl;
		}
	}
};

void index_builder::build_index()
{
	const int NIMAGE = image_list_.size();
	if (matching_method_ == "VocabularyTree")
	{

		const WeightingType weight = TF_IDF;
		const ScoringType score = L1_NORM;

		Vocabulary voc(K_,L_,weight, score);
		cout << "Creating a " << K_ << "^" << L_ << "vocabulary tree....." << endl;
		vector<cv::Mat> old_descriptors;
		for(size_t k = 0; k<descriptors_.size(); k++)
		{
			old_descriptors.push_back(descriptors_[k].clone());
		}
		voc.create(descriptors_);
		cout << "...done!" << endl;
		voc.save(feature_output_ + feature_name_+"_Vocabulary.yml.gz");
		cout << "Creating database....." << endl;
		Database db(voc,false,0);
		cout << "Finished" << endl;
		for (int i = 0; i < NIMAGE; i++)
		{
			db.add(old_descriptors[i]);
		}
		db.save(feature_output_+feature_name_+"database.yml.gz");
		/*
		Database db_;
		db_.load(feature_output_+feature_name_+"database.yml.gz");
		QueryResults ret;
		for(size_t i = 0; i < 10; i++)
    	{
			cv::Mat image = cv::imread(path_of_dataset_ + image_list_[i], CV_LOAD_IMAGE_GRAYSCALE);
			std::cout<<path_of_dataset_ + image_list_[i]<<std::endl;
			if (image.empty()) cerr << "empty image" << endl;
			cv::Mat mask;
		//	std::cout<<"mask rows: "<<mask.rows<<" mask cols: "<<mask.cols<<std::endl;
			vector<cv::KeyPoint> keypoints;
			cv::Mat descriptor; //= old_descriptors[i].clone();
			detector_->detectAndCompute(image,mask,keypoints,descriptor);
			//cv::Mat diff = descriptor != descriptors_[i];
			//std::cout<<"different points number: "<<cv::countNonZero(diff)<<std::endl;
		//	std::cout<<" New descriptor rows: "<<descriptor.rows<<" columns: "<<descriptor.cols<<std::endl;
		//	std::cout<<" Old descriptor rows: "<<descriptors_[i].rows<<" columns: "<<descriptors_[i].cols<<std::endl;
			//std::cout<<keypoints.size()<<std::endl;
			//std::cout<<descriptor.channels()<<std::endl;
       	 	db_.query(descriptor, ret, 4);
			/*
			int diff_number=0;
			
			for(size_t k=7; k<8; k++)
			{
				for(size_t j = 0; j<descriptor.cols; j++)
				{
					if((descriptor.at<bool>(k, j)!=descriptors_[i].at<bool>(k, j)))
					{
						std::cout<<"row: "<<k<<" cols: "<<j<<" new value: "<<descriptor.at<bool>(k, j)<<" old value: "<<descriptors_[i].at<bool>(k, j)<<std::endl;
						diff_number+=1;
					}
				}
			}
			*/
			/*
			vector<cv::KeyPoint> oldkeypoints = _keypoints[i];
			for(size_t k=0; k<keypoints.size(); k++)
			{
				//if(keypoints[k].pt.x!=oldkeypoints[k].pt.x || keypoints[k].pt.y!=oldkeypoints[k].pt.y)
				//{
					std::cout<<"x_coor: "<<keypoints[k].pt.x<<" old x_coor: "<<oldkeypoints[k].pt.x<<" y_coor "<<keypoints[k].pt.y<<" old y_coor "<<oldkeypoints[k].pt.y<<std::endl;
					diff_number++;
			//	}
			}
			*/
			//std::cout<<"different number: "<<diff_number<<std::endl;;
/*
			for(size_t k=0; k<1; k++)
			{
				for(size_t j = 0; j<descriptors_[i].cols; j++)
				{
					std::cout<<descriptors_[i].at<double>(k, j)<<std::endl;
				}
				std::cout<<endl;
			}
*/			

        // ret[0] is always the same image in this case, because we added it to the
        // database. ret[1] is the second best match.

        //cout << "Searching for Image " << i << ". " << ret << endl;
//		}
//		db.save(feature_output_+feature_name_+"database.yml.gz");
	}
	else
	{
		cerr << "Not supported yet" << endl;
	}
};


void index_builder::extract_ORB()
{
	const int NIMAGE = image_list_.size();
	for (int i = 0; i < NIMAGE; i++)
	{
		cv::Mat emptyResult;
		if(cross_validation_)
		{
			if((cross_num_+i)%10==0)
			{
				descriptors_.push_back(emptyResult);
				cout << "Used as test dataset: "<<i<< "/" << NIMAGE << endl;
				continue;
			}
		}
		cv::Mat image = cv::imread(path_of_dataset_ + image_list_[i], CV_LOAD_IMAGE_GRAYSCALE);
		std::cout<<path_of_dataset_ + image_list_[i]<<std::endl;
		if (image.empty()) cerr << "empty image" << endl;
		cv::Mat mask;
		vector<cv::KeyPoint> keypoints;
		cv::Mat descriptor;
		detector_->detectAndCompute(image,mask,keypoints,descriptor);
		if (keypoints.size() == 0)
		{
			descriptors_.push_back(emptyResult);
			cout << "No features to extract: "<<i + 1 << "/" << NIMAGE << endl;
			continue;
		}
		else
		{
			descriptors_.push_back(descriptor);
		}
	}
}
