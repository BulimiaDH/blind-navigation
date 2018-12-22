#include"index_builder.h"
#include<vector>
int main()
{
    std::string path_of_dataset_image = "/home/blindfind/Documents/Blindfind3/data/trackAll/";
    std::string feature_name = "ORB";
    std::string matching_method = "VocabularyTree";
    std::string feature_output = "/home/blindfind/Documents/Blindfind3/data/";
    index_builder builder(path_of_dataset_image, feature_name, matching_method, feature_output, false);
    builder.setParam(6,9);
    builder.extract_features();
    builder.build_index();
    return 0;
}
