#ifndef PATHS_CREATOR_H
#define PATHS_CREATOR_H

#include <goal.h>

class PathsCreator {
    typedef unsigned int uint;

    std::vector<std::vector<Goal>> m_goalsTable;
    std::string m_worldFrame;
    std::vector<std::string> m_frames;

    bool readGoals(const std::string &mapPath);
    void createPaths(bool splinesMode);

public:
    PathsCreator() = delete;
    PathsCreator(const PathsCreator &) = delete;
    PathsCreator & operator=(const PathsCreator &) = delete;

    PathsCreator(const  std::string &worldFrame,
                 const  std::vector<std::string> &frames,
                 const  std::string &mapPath,
                 bool   splinesMode = false);

    std::vector<Goal> getPath(const std::string &frame) const;
    ~PathsCreator();
};

#endif // PATHS_creator_H
