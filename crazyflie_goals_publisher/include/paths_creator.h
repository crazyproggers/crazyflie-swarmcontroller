#ifndef PATHS_CREATOR_H
#define PATHS_CREATOR_H

#include "goal.h"

class PathsCreator {
    bool readTable(const std::string &pathToMap, 
                   const std::string &worldFrame, 
                   const std::vector<std::string> &frames);

    void createPaths(bool splinesMode);

public:
    PathsCreator() = delete;
    PathsCreator(const PathsCreator &) = delete;
    PathsCreator & operator=(const PathsCreator &) = delete;

    PathsCreator(const std::string &worldFrame,
                 const std::vector<std::string> &frames,
                 const std::string &pathToMap,
                 bool  splinesMode = false);

    std::vector<std::list<Goal>> paths;

    ~PathsCreator();
};

#endif // PATHS_creator_H
