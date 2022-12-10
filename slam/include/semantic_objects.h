#include<vector>
#include<string>
#include<map>
using namespace std;
vector<string> object_{"void", "wall", "floor", "cabinet", "bed", "chair",
                           "sofa", "table", "door", "window", "bookshelf",
                           "picture", "counter", "blinds", "desk", "shelves",
                           "curtain", "dresser", "pillow", "mirror",
                           "floor mat", "clothes", "ceiling", "books",
                           "fridge", "tv", "paper", "towel", "shower curtain",
                           "box", "whiteboard", "person", "night stand",
                           "toilet", "sink", "lamp", "bathtub", "bag","center"};

map<size_t,string> objects_map={
    {0,"wall"},
    {1,"floor"}, 
    {4,"chair"},
    {5,"sofa"},
    {6,"table"},
    {7,"door"},
    {8, "window"}, 
    {9,"bookshelf"},
    {13,"desk"},
    {14,"shelves"},
    {21,"ceiling"}, 
    {22,"books"},
    {24,"tv"},  
    {30,"person"},
    {35,"lamp"}};

vector<uint8_t> objects_15{0,1,4,5,6,7,8,9,13,14,21,22,24,30,35};

