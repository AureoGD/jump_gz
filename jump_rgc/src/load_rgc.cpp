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

#include "jump_rgc/load_rgc.h"

using namespace gz::launch;

LoadRGC::LoadRGC()
    : Plugin()
{
    rgc_controller = new RefGovCon();
}

LoadRGC::~LoadRGC()
{
}

bool LoadRGC::Load(const tinyxml2::XMLElement *_elem)
{
    const tinyxml2::XMLElement *elem;

    elem = _elem->FirstChildElement("action_service");

    if (elem)
        this->service_name = elem->GetText();

    // std::cout << this->service_name << std::endl;

    return true;
}
