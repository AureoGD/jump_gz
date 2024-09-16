#include <fcntl.h>
#include <sys/stat.h>

#include <string>
#include <vector>

#include <gz/msgs/joy.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>

#include "jump_adapter/jump_load_adapter.h"

using namespace gz::launch;

JumpLoadAdapter::JumpLoadAdapter()
    : Plugin()
{
}

JumpLoadAdapter::~JumpLoadAdapter()
{
}

bool JumpLoadAdapter::Load(const tinyxml2::XMLElement *_elem)
{
    const tinyxml2::XMLElement *elem;

    std::cout << "Loading Jump Adapter" << std::endl;

    jumpAdapter.init();

    return true;
}
