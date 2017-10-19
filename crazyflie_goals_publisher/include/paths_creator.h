#ifndef PATHS_CREATOR_H
#define PATHS_CREATOR_H

#include "goal.h"

class PathsCreator {
    std::vector<std::list<Goal>> m_goalsTable;
    std::string m_worldFrame;
    std::vector<std::string> m_frames;

    bool readTable(const std::string &mapPath);
    void createPaths(bool splinesMode);

public:
    PathsCreator() = delete;
    PathsCreator(const PathsCreator &) = delete;
    PathsCreator & operator=(const PathsCreator &) = delete;

    PathsCreator(const  std::string &worldFrame,
                 const  std::vector<std::string> &frames,
                 const  std::string &mapPath,
                 bool   splinesMode = false);

    std::list<Goal> getPath(const std::string &frame) const;
    ~PathsCreator();
};

#endif // PATHS_creator_H
