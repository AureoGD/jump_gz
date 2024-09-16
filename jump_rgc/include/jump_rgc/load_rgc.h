#ifndef GZ_LAUNCH_LOAD_RGC_HH_
#define GZ_LAUNCH_LOAD_RGC_HH_

#include <string>
#include <thread>

#include <gz/launch/Plugin.hh>
#include <gz/plugin/Register.hh>

#include "jump_rgc/rgc_controller.h"

namespace gz
{
    namespace launch
    {
        class LoadRGC : public gz::launch::Plugin
        {
        public:
            LoadRGC();

            virtual ~LoadRGC();

            virtual bool Load(
                const tinyxml2::XMLElement *_elem) override final;

        private:
            void Run();

            RefGovCon *rgc_controller;

            std::string service_name;
        };
    }
}
// Register the plugin
GZ_ADD_PLUGIN(gz::launch::LoadRGC, gz::launch::Plugin)

#endif