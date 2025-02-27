#ifndef GAFFERSCOTCH_SCENEPATHUTIL_H
#define GAFFERSCOTCH_SCENEPATHUTIL_H

#include "GafferScene/ScenePlug.h"
#include "IECore/StringAlgo.h"

namespace GafferScotch
{

/**
 * Utility function to convert a string path to a ScenePath.
 * @param p The string path with '/' separators
 * @return A ScenePath object containing the tokenized path
 */
inline GafferScene::ScenePlug::ScenePath makeScenePath(const std::string &p)
{
    GafferScene::ScenePlug::ScenePath output;
    IECore::StringAlgo::tokenize<IECore::InternedString>(p, '/', std::back_inserter(output));
    return output;
}

} // namespace GafferScotch

#endif // GAFFERSCOTCH_SCENEPATHUTIL_H 