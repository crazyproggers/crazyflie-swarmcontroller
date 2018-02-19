#ifndef PATHS_CREATOR_H
#define PATHS_CREATOR_H

#include "goal.h"

class PathsCreator {
    std::string                     m_worldFrame;
    std::vector<std::list<Goal>>    m_paths;
    bool                            m_splinesMode;
    bool                            m_canGenPaths;

    bool readTable(const std::string &pathToMap);

public:
    PathsCreator()                                  = delete;
    PathsCreator(const PathsCreator &)              = delete;
    PathsCreator & operator=(const PathsCreator &)  = delete;

    PathsCreator(const std::string &pathToMap,
                 const std::string &worldFrame,
                 bool  splinesMode = false);
    ~PathsCreator();

    bool canGenPaths() const noexcept;
    std::list<Goal> genPath(const std::string &frame);
};

#endif // PATHS_CREATOR_H
