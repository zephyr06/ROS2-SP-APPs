#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "sources/Utils/Parameters.h"
#include "sources/Utils/testMy.h"

namespace SP_OPT_PA {
std::vector<double> ReadTxtFile(std::string path);

inline std::string RelativePathToAbsolutePath(std::string path) {
    if (path[0] != '/') {
        path = GlobalVariables::PROJECT_PATH + path;
    }
    return path;
}
}  // namespace SP_OPT_PA