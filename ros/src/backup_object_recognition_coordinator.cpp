#include <mcr_object_recognition/object_recognition_coordinator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>

using namespace mcr_object_recognition;

ObjectRecognitionCoordinator::ObjectRecognitionCoordinator()
{
}
ObjectRecognitionCoordinator::~ObjectRecognitionCoordinator()
{
}
void ObjectRecognitionCoordinator::onInit()
{
    nh_ = getPrivateNodeHandle();

    nh_.param<int>("number_of_recognition_nodes", number_of_recognition_nodes_, 1);

    pub_event_in_ = nh_.advertise<std_msgs::String>("object_detector_event_in", 1);
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    pub_object_list_ = nh_.advertise<std_msgs::String>("output_object_list", 1);

    sub_event_in_ = nh_.subscribe("event_in", 1, &ObjectRecognitionCoordinator::eventInCallback, this);
    sub_object_list_ = nh_.subscribe("input_object_list", 1, &ObjectRecognitionCoordinator::objectListCallback, this);
    sub_recognition_result_ = nh_.subscribe("recognition_result", number_of_recognition_nodes_, &ObjectRecognitionCoordinator::recognitionResultCallback, this);
}

void ObjectRecognitionCoordinator::eventInCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "e_trigger")
    {
        object_list_.objects.clear();
        results_.clear();
        std_msgs::String event_in;
        event_in.data = "e_trigger";
        pub_event_in_.publish(event_in);
    }
}

void ObjectRecognitionCoordinator::objectListCallback(const mcr_perception_msgs::ObjectList::ConstPtr &msg)
{
    object_list_ = *msg;
    if (results_.size() == number_of_recognition_nodes_ && object_list_.objects.size() > 0)
    {
        mergeResultsAndPublish();
    }
}

void ObjectRecognitionCoordinator::recognitionResultCallback(const mcr_perception_msgs::RecognitionResultList::ConstPtr &msg)
{
    results_.push_back(msg);
    if (results_.size() == number_of_recognition_nodes_ && object_list_.objects.size() > 0)
    {
        mergeResultsAndPublish();
    }
}

void ObjectRecognitionCoordinator::mergeResultsAndPublish()
{
    int number_of_objects = results_[0]->results.size();
    for (int i = 0; i < number_of_objects; i++)
    {
        std::map<std::string, std::vector<double> > unique_objects;
        // compile map of label -> list of probabilities
        for (int j = 0; j < results_.size(); j++)
        {
            std::string label = results_[j]->results[i].label;
            double probability = results_[j]->results[i].probability;
            if (unique_objects.find(label) == unique_objects.end())
            {
                std::vector<double> prob_list;
                prob_list.push_back(probability);
                unique_objects[label] = prob_list;
            }
            else
            {
                unique_objects[label].push_back(probability);
            }
        }
        // for each label, combine probabilities
        // eg. if probabilities are 0.8 and 0.9,
        // resultant probability = (1 - ((1 - 0.8) * (1 - 0.9)) = 0.98
        // and find label with highest probability
        std::map<std::string, std::vector<double> >::iterator it;
        double max_probability = 0.0;
        std::string best_object;
        for (it = unique_objects.begin(); it != unique_objects.end(); ++it)
        {
            std::vector<double> probabilities = it->second;
            double result = 1.0;
            for (int k = 0; k < probabilities.size(); k++)
            {
                result *= (1.0 - probabilities[k]);
            }
            result = 1.0 - result;
            if (result > max_probability)
            {
                max_probability = result;
                best_object = it->first;
            }
        }
        object_list_.objects[i].name = best_object;
        object_list_.objects[i].probability = max_probability;
    }
    pub_object_list_.publish(object_list_);
}
