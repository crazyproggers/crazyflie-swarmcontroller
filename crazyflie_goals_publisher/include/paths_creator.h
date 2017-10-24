#ifndef PATHS_CREATOR_H
#define PATHS_CREATOR_H

#include "goal.h"

class PathsCreator {
    std::string m_worldFrame;
    std::vector<std::string> m_frames;

    bool readTable(const std::string &pathToMap);
    void createPaths(bool splinesMode);

public:
    PathsCreator() = delete;
    PathsCreator(const PathsCreator &) = delete;
    PathsCreator & operator=(const PathsCreator &) = delete;

    PathsCreator(const  std::string &worldFrame,
                 const  std::vector<std::string> &frames,
                 const  std::string &pathToMap,
                 bool   splinesMode = false);

    std::vector<std::list<Goal>> paths;

    ~PathsCreator();
};

#endif // PATHS_creator_H
